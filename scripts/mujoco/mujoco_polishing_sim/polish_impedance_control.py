"""
Cartesian impedance control + explicit normal-force regulation for a MuJoCo
RealMan RM65-B with a pneumatic compliant sanding head, polishing a workpiece that is flat on
one half and a true circular arc on the other half.

This is a hybrid position/force controller (Raibert-Craig style), the same
structural idea used by robosuite's OSC controller and deoxys_control's
Cartesian impedance mode, split into two phases:

Phase 1 (t < APPROACH_TIME, "approach"): pure task-space impedance control on all of
x, y, z and orientation, smoothly lowering the tool onto the surface:

    tau = J^T [ Kp_pos * (x_d - x) + Kd_pos * (0 - xdot)
                Kp_rot * ori_err   + Kd_rot * (0 - omega) ]  + qfrc_bias

Phase 2 (t >= APPROACH_TIME, "polishing"): a Raibert-Craig hybrid decomposition done
in the *local surface frame* (not world axes), using the per-x surface normal
estimated from a depth-camera point cloud (see the perception section):

  * NORMAL direction  -> explicit force control. The contact force is projected
    onto the local surface normal, low-pass filtered, and driven to f_target by
    a PI law plus damping. Commanding force along the true normal (rather than
    world -z) keeps the *actual* normal force at target even on the tilted arc,
    where regulating world-z would over-press by a factor 1/cos(tilt).
  * TANGENT plane     -> impedance control tracking the raster path. The raster
    target error and the tool velocity are projected onto the tangent plane so
    the position spring only ever acts within the surface, never fighting the
    force-controlled normal axis.

On the flat half (normal = +z) this is algebraically identical to the earlier
world-frame version (world-z force + world-xy impedance). Note: mixing a stiff
position spring with a force-integral correction on the *same* axis is a classic
source of chatter/limit cycles -- the tangent/normal split keeps them strictly
orthogonal, so they never fight each other.

    f_err   = f_target - (contact_force . n)_filtered
    Fn_cmd  = -(f_target + Kp_f * f_err + Ki_f * integral(f_err)) - Bz * (xdot . n)
    F_task  = Fn_cmd * n  +  [ Kp_pos * proj_tangent(x_d - x) - Kd_pos * proj_tangent(xdot) ]

qfrc_bias (gravity + Coriolis/centrifugal torques, computed by mj_forward)
is added as feed-forward compensation in both phases -- the standard trick
so the task-space law only has to fight the *impedance/force* error, not
gravity.
"""

import sys
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = "PingFang SC"  # renders the CJK labels in the
# steady-state stats box (标准差/方差/极差) instead of falling back to tofu boxes
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt

MODEL_PATH = "polish_scene.xml"
# Run with `python polish_impedance_control.py --view` on a machine with a
# display to watch the polishing motion live in MuJoCo's interactive viewer.
# The default GUI camera is free and mouse-adjustable.  Use `--camera=cell_view`,
# `--camera=tool_view`, or `--camera=camera_view` for repeatable inspections.
LIVE_VIEW = "--view" in sys.argv
VIEW_CAMERA = "free"
for arg in sys.argv:
    if arg.startswith("--camera="):
        VIEW_CAMERA = arg.split("=", 1)[1]

# ----------------------------- tunable gains -----------------------------
# Tangential stiffness for the full-width raster.  The earlier 60 N/m setting
# was safe but lagged more than 100 mm behind a 4 s row, so the expected path
# changed rows before the real pad reached the edge.  400 N/m plus endpoint
# dwell tracks the 310 mm sweep while keeping normal-force control independent.
KP_POS_XY = np.array([400.0, 400.0])
KD_POS_XY = 2.0 * np.sqrt(KP_POS_XY) * 1.0
KP_ROT = np.array([30.0, 30.0, 30.0])      # Nm/rad, rotational stiffness
KD_ROT = 2.0 * np.sqrt(KP_ROT) * 1.0
KP_POS_Z_APPROACH = 100.0                  # gentler descent for the lighter RM65-B
KD_POS_Z_APPROACH = 2.0 * np.sqrt(KP_POS_Z_APPROACH)

F_TARGET = 8.0      # N, desired normal (world -z into the surface) contact force
FORCE_RAMP_TIME = 1.5  # s, smoothly ramp 0 -> F_TARGET after entering force control
KP_F = 0.1          # N per N of force error (explicit force-loop proportional gain)
KI_F = 0.2          # N per (N*s) of accumulated force error
BZ = 70.0           # N*s/m, z-axis damping (critical for contact stability)
F_FILT_ALPHA = 0.03  # low-pass filter coefficient for the noisy contact-force reading.
# 0.1 let too much of MuJoCo's per-timestep contact-solver noise through (peak-to-peak
# force noise ~9N around the 8N target); 0.03 filters harder and cut that to ~2.5N
# peak-to-peak without hurting -- in fact slightly helping -- steady-state tracking.
I_CLAMP = 15.0      # anti-windup clamp on the force integral term
F_MAX = F_TARGET + 1.0  # N, soft ceiling on the COMMANDED normal force (Fn_cmd below).
# Under-pressing is recoverable (just polish that spot again); over-pressing risks
# gouging/damaging the workpiece, so the two failure directions are not symmetric --
# cap the PI+damping law's output so it can never ASK for more than a fixed margin
# above target. Empirically, clamping with ZERO margin (F_MAX=F_TARGET exactly) is
# actively worse, not safer: it removes the small "extra push" the control law needs
# to converge quickly, so tracking noise gets *larger* (measured peak-to-peak rose
# from 2.6N to 3.5N in testing) -- some headroom above target is required for the loop
# to do its job. A +1N margin (12.5% above target) leaves that headroom for normal
# operation untouched while still bounding routine over-commanding.
#
# F_MAX alone is NOT a safety guarantee, though -- it bounds what the controller ASKS
# FOR, not what the contact dynamics actually DELIVER (a transient impact can still
# produce more real force than was ever commanded). The mechanism below is the actual
# safety net: an independent, FAST, lightly-filtered monitor of the real measured
# force, dedicated only to tripping an emergency retreat -- not used for tracking.
F_FAST_ALPHA = 0.2       # low-lag EMA for the safety monitor ONLY -- about 6x less
# filtering than F_FILT_ALPHA's 0.03, so it reacts in ~5 control steps (~10ms) instead
# of dozens, catching a real spike quickly. It is deliberately NOT used for f_err --
# mixing a twitchy, higher-noise signal into the tracking loop would just reintroduce
# the chatter that F_FILT_ALPHA=0.03 was tuned to remove (chapter 10).
F_RETREAT_TRIGGER = 10.0  # N, fast-filtered MEASURED force, 25% above the 8N target.
F_RESULTANT_TRIGGER = 20.0  # N, perception-independent contact-force magnitude trip.
# This threshold is intentionally above F_RETREAT_TRIGGER: raster motion creates
# legitimate tangential friction, so the vector magnitude is normally larger than
# its normal component. It is a last-resort guard for a corrupted estimated normal.
# The old 25N threshold sat above the force this robot can normally produce and was
# therefore effectively unreachable. 10N is below the sustained-fault range, but
# MuJoCo's contact solver can produce very short numerical spikes above it. Require
# persistence so those solver artifacts do not stop an otherwise healthy run.
F_RETREAT_PERSIST_TIME = 0.025  # s above threshold before tripping (about 13 steps)
# In a fixed-seed normal run, f_fast excursions above 10N lasted at most 14ms. A
# 25ms qualifier rejects those brief spikes while still stopping a real sustained
# over-force in a small fraction of a second.
# The threshold remains below the 14-19N sustained-fault range measured in stress tests.
F_RETREAT_CLEAR = 2.0  # N, contact is considered released below this level.
# This must stay well below F_RETREAT_TRIGGER. Once a trip occurs we do NOT resume
# polishing in the same run: after releasing contact the controller latches in a
# Cartesian hold, because under-polishing is preferable to returning to a possibly
# persistent fault and removing too much material.
F_RETREAT_FORCE = 20.0   # N, constant force pulling the tool away along +normal while
# retreating -- deliberately not PI/tuned, this is a blunt "get away from the surface
# now" response, not a tracking law.
KP_FAULT_HOLD = 120.0
KD_FAULT_HOLD = 2.0 * np.sqrt(KP_FAULT_HOLD)

Z_SURFACE_GUESS = 0.30           # physical flat-region surface height
PAD_CONTACT_OFFSET = 0.014       # TCP-to-pad-bottom distance in rm65b_pneumatic.xml
APPROACH_CONTACT_Z = Z_SURFACE_GUESS + PAD_CONTACT_OFFSET
APPROACH_Z = 0.36                # 46 mm pad clearance at the full-raster start point
APPROACH_TIME = 2.5              # smooth but visibly quicker descent

# --------------------- full-surface raster (boustrophedon) scan ---------------------
# The physical workpiece spans x=[0.32, 0.68], y=[-0.18, 0.18] m.  The 50 mm
# backing pad has a 25 mm radius.  X uses the exact 25 mm inset; Y intentionally
# uses an 11 mm inset so the pad overhangs each turnaround edge by 14 mm. That is
# normal edge-finishing practice and compensates the measured per-row dynamic
# tangential tracking offset without moving the contact centre off the part.
# Thirteen rows give 25.8 mm x spacing: roughly 48% overlap, with no strips.
RASTER_X_MIN, RASTER_X_MAX = 0.345, 0.655
RASTER_Y_MIN, RASTER_Y_MAX = -0.169, 0.169
RASTER_ROWS = 13
RASTER_SWEEP_TIME = 4.5                      # 2.67x faster than the old 12 s sweep
RASTER_END_DWELL = 1.0                       # brief edge dwell before reversing
RASTER_ROW_PERIOD = RASTER_SWEEP_TIME + RASTER_END_DWELL
RASTER_TRANSITION_TIME = 0.25
POLISH_TIME = RASTER_ROWS * RASTER_ROW_PERIOD

# ----------------- workpiece arc geometry (must match polish_scene.xml) -------------
# Original workpiece: flat up to x=0.50, then a true circular arc tangent to it.
# The geometry is unchanged; the right-side pedestal makes the full raster
# reachable on one continuous branch while preserving surface-normal alignment.
WORKPIECE_CENTER_X = 0.50
WORKPIECE_HALF_X = 0.18
WORKPIECE_HALF_Y = 0.18
ARC_RADIUS = 0.425
ARC_ELEVATION = 0.04  # must match the hfield elevation in polish_scene.xml

SIM_TIME = APPROACH_TIME + POLISH_TIME
ARM_JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
PNEUMATIC_PRELOAD = 4.0  # N, regulated cylinder thrust in the compliant tool slide


def _raster_row_x(row):
    if RASTER_ROWS > 1:
        return RASTER_X_MIN + (RASTER_X_MAX - RASTER_X_MIN) * row / (RASTER_ROWS - 1)
    return RASTER_X_MIN


def raster_xy(t_polish):
    """Boustrophedon (back-and-forth) scan spanning the whole workpiece footprint.

    Steps x once per row (covering both the flat and curved halves) and sweeps
    y across the full row each time, alternating direction each row so
    consecutive rows connect without doubling back.

    Two smoothing tricks avoid the sharp force spikes a naive constant-velocity
    raster produces:
      - the x step between rows is slid smoothly over RASTER_TRANSITION_TIME
        instead of jumping instantly (avoids a sideways lurch when a row starts);
      - y within a row follows a smoothstep, not a linear ramp, so the tool's
        y-velocity eases to zero at each end of the row instead of reversing
        direction instantly at the turnaround (that instant reversal, not the
        x step, turned out to be the main source of force spikes).
    """
    row = min(int(t_polish // RASTER_ROW_PERIOD), RASTER_ROWS - 1)
    t_in_row = t_polish - row * RASTER_ROW_PERIOD
    row_t = np.clip(t_in_row / RASTER_SWEEP_TIME, 0.0, 1.0)
    row_t_smooth = row_t * row_t * (3 - 2 * row_t)  # zero velocity at row_t=0 and 1

    x = _raster_row_x(row)
    if row > 0 and t_in_row < RASTER_TRANSITION_TIME:
        blend = t_in_row / RASTER_TRANSITION_TIME
        x = _raster_row_x(row - 1) + (x - _raster_row_x(row - 1)) * blend

    if row % 2 == 0:
        y = RASTER_Y_MIN + (RASTER_Y_MAX - RASTER_Y_MIN) * row_t_smooth
    else:
        y = RASTER_Y_MAX - (RASTER_Y_MAX - RASTER_Y_MIN) * row_t_smooth
    return np.array([x, y])


def orientation_error(R_d, R_cur):
    """so(3) log-map style orientation error used by OSC controllers."""
    err = 0.5 * (np.cross(R_cur[:, 0], R_d[:, 0])
                 + np.cross(R_cur[:, 1], R_d[:, 1])
                 + np.cross(R_cur[:, 2], R_d[:, 2]))
    return err


def get_contact_force_on_pad(model, data, pad_geom_id):
    """Sum contact forces (world frame) acting on the polishing pad geom."""
    f_world_total = np.zeros(3)
    for i in range(data.ncon):
        con = data.contact[i]
        if con.geom1 != pad_geom_id and con.geom2 != pad_geom_id:
            continue
        force6 = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force6)  # contact-frame [f;torque]
        f_contact = force6[:3]
        R_c = con.frame.reshape(3, 3)  # rows are contact frame axes in world coords
        f_world = R_c.T @ f_contact
        # mj_contactForce reports the force the *first* geom exerts on the
        # second one along the contact normal; flip sign so we consistently
        # report the force exerted ON the pad.
        if con.geom1 == pad_geom_id:
            f_world = -f_world
        f_world_total += f_world
    return f_world_total


def fill_workpiece_hfield(model):
    """Bake a flat -> true circular arc height profile into workpiece_hf.

    For local x <= 0 (world x <= WORKPIECE_CENTER_X): flat, height = 0.
    For local x > 0: height = ARC_RADIUS - sqrt(ARC_RADIUS^2 - x^2),
    i.e. the flat plane is exactly tangent to a circle of radius ARC_RADIUS at
    x=0 (zero slope there, so no crease), and the surface then rises along that
    real arc all the way to the workpiece edge -- it never flattens out again.
    """
    hid = model.hfield("workpiece_hf").id
    nrow = model.hfield_nrow[hid]
    ncol = model.hfield_ncol[hid]
    adr = model.hfield_adr[hid]
    x_half = model.hfield_size[hid][0]
    elevation = model.hfield_size[hid][2]

    xs_local = np.linspace(-x_half, x_half, ncol)
    s = np.clip(xs_local, 0.0, ARC_RADIUS)  # only x > 0 rises; flat for x <= 0
    height_above_flat = ARC_RADIUS - np.sqrt(ARC_RADIUS ** 2 - s ** 2)
    profile = np.clip(height_above_flat / elevation, 0.0, 1.0)
    model.hfield_data[adr:adr + nrow * ncol] = np.tile(profile, (nrow, 1)).flatten()


# --------------------- perception: depth camera -> point cloud -> normals ------------
# The controller does NOT get to use ARC_RADIUS/ARC_ELEVATION for orientation control.
# Instead it takes one depth-camera snapshot of the workpiece before polishing starts
# (a real system would do the same: scan first, then work), reconstructs a 3D point
# cloud from it, and fits local surface normals from that point cloud alone. This is
# a genuine perception pipeline, not the geometry formula in disguise.
SCAN_CAM_NAME = "scan_cam"
# Gemini 336L operating mode used by the physical data-collection stack.
SCAN_IMG_WIDTH = 640
SCAN_IMG_HEIGHT = 480
SCAN_MAX_DEPTH = 0.9          # drop anything farther than this (the floor plane)
SCAN_NUM_X_BINS = 40          # how many local-plane fits to make across the footprint
SCAN_BIN_HALF_WIDTH = 0.012   # m, points within this +/- x of a bin center feed its plane fit

DEPTH_NOISE_STD = 0.002  # m, per-pixel depth noise added to emulate a realistic
# (imperfect) depth sensor -- MuJoCo's renderer is itself noise-free, so without this
# the "camera" is unrealistically perfect and the perception pipeline never gets
# tested against anything a real sensor would actually produce. 2mm is representative
# of a decent depth camera at this ~0.5-0.7m working distance.
#
# This number is NOT arbitrary -- fit_normals_along_x() fits a plane per x-bin from
# SCAN_BIN_HALF_WIDTH*2 = 24mm of x-extent, whose points have a natural x-spread of
# about 24mm/sqrt(12) =~ 6.9mm (uniform distribution). Once per-pixel z-noise
# approaches that same ~7mm, the SVD plane fit can no longer reliably tell "flat in z"
# apart from "tilted 90 degrees toward x" -- both directions have comparable spread --
# and the fitted normal can be wrong by 50-90 degrees, not just noisy. Measured
# empirically: fits are still reliable (angle error a few degrees at most) through
# about 2-5mm of single-frame noise, then degrade sharply, and by 7mm the controller
# loses contact with the workpiece entirely. 2mm keeps a comfortable margin below that
# cliff; SCAN_NUM_FRAMES below buys additional margin for noisier real cameras.
SCAN_NUM_FRAMES = 4  # average this many independent noisy depth frames before
# reconstructing the point cloud -- the same temporal-averaging trick real depth
# cameras/SDKs use internally, which reduces the EFFECTIVE noise by sqrt(SCAN_NUM_FRAMES)
# (4 frames -> half the noise). Cheap to do here since perception runs once, before
# polishing starts (see main()), not per control step.


def capture_point_cloud(model, data):
    """Render a depth image from scan_cam and reconstruct a world-frame point cloud.

    MuJoCo's Renderer, with depth rendering enabled, returns true metric distance
    (in meters, along the camera's viewing axis) per pixel -- not a normalized
    OpenGL depth buffer -- so reconstruction is a plain pinhole-camera unprojection.
    """
    cam_id = model.camera(SCAN_CAM_NAME).id
    renderer = mujoco.Renderer(model, height=SCAN_IMG_HEIGHT, width=SCAN_IMG_WIDTH)
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=cam_id)
    depth_clean = renderer.render().copy()
    renderer.close()

    # Emulate SCAN_NUM_FRAMES independent noisy real-camera frames of the same,
    # static scene, then average them -- MuJoCo's own render is noise-free, so the
    # noise here is injected explicitly to keep the simulated sensor honest.
    noisy_frames = depth_clean[None, :, :] + np.random.normal(
        0.0, DEPTH_NOISE_STD, size=(SCAN_NUM_FRAMES,) + depth_clean.shape)
    depth = noisy_frames.mean(axis=0)

    h, w = depth.shape
    fovy = model.cam_fovy[cam_id]
    # MuJoCo specifies vertical field of view, so fy follows image height. Square
    # pixels give fx=fy, matching the calibrated-pinhole form used by OrbbecSDK.
    fx = fy = h / (2 * np.tan(np.deg2rad(fovy) / 2))
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    cx, cy = w / 2, h / 2
    x_cam = (us - cx) / fx * depth
    y_cam = -(vs - cy) / fy * depth
    z_cam = -depth
    pts_cam = np.stack([x_cam, y_cam, z_cam], axis=-1).reshape(-1, 3)

    valid = depth.reshape(-1) < SCAN_MAX_DEPTH
    pts_cam = pts_cam[valid]

    cam_pos = data.cam_xpos[cam_id].copy()
    cam_mat = data.cam_xmat[cam_id].reshape(3, 3).copy()
    return pts_cam @ cam_mat.T + cam_pos  # world-frame Nx3 point cloud


def fit_normals_along_x(points, x_min, x_max):
    """Bin the point cloud by world x and fit a local plane (via SVD) in each bin.

    Returns (bin_centers, normals) with normals as unit vectors pointing away from
    the solid surface (+z-ish); use build_orientation_from_normal() to turn one of
    these into a tool target orientation. This is the "surface normal estimation
    from a point cloud" step of a real scan-then-polish pipeline.
    """
    bin_centers = np.linspace(x_min, x_max, SCAN_NUM_X_BINS)
    normals = np.zeros((SCAN_NUM_X_BINS, 3))
    last_good = np.array([0.0, 0.0, 1.0])
    for i, xc in enumerate(bin_centers):
        mask = np.abs(points[:, 0] - xc) < SCAN_BIN_HALF_WIDTH
        local = points[mask]
        if local.shape[0] < 8:
            normals[i] = last_good
            continue
        centroid = local.mean(axis=0)
        _, _, vt = np.linalg.svd(local - centroid)
        normal = vt[-1]
        if normal[2] < 0:
            normal = -normal
        normals[i] = normal
        last_good = normal
    return bin_centers, normals


def estimate_normal_at_x(x_query, bin_centers, normals):
    """Interpolate the fitted-normal lookup table at an arbitrary x, renormalized."""
    n = np.array([np.interp(x_query, bin_centers, normals[:, k]) for k in range(3)])
    norm = np.linalg.norm(n)
    return n / norm if norm > 1e-9 else np.array([0.0, 0.0, 1.0])


def build_orientation_from_normal(normal):
    """Tool frame that presses along -normal, reducing to R_down when normal=+z.

    z_axis is the pressing direction (into the surface); x_axis/y_axis are picked
    by Gram-Schmidt against world +x so the frame varies smoothly and matches the
    original fixed-orientation controller exactly on the flat region.
    """
    z_axis = -normal
    x_ref = np.array([1.0, 0.0, 0.0])
    y_axis = np.cross(z_axis, x_ref)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    return np.column_stack([x_axis, y_axis, z_axis])


def main():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    fill_workpiece_hfield(model)

    arm_dof_idx = [model.joint(name).dofadr[0] for name in ARM_JOINT_NAMES]
    arm_act_idx = [model.actuator(f"actuator{i}").id for i in range(1, 7)]
    tcp_site_id = model.site("tcp_site").id
    pad_geom_id = model.geom("pad_geom").id
    pneumatic_act_id = model.actuator("pneumatic_force").id
    key_id = model.key("home").id

    # Scan the workpiece ONCE, before polishing starts, exactly like a real
    # scan-then-polish pipeline would: render a depth image, reconstruct a point
    # cloud, fit local surface normals along x. The controller only ever consults
    # this fitted lookup table afterward -- it never reads ARC_RADIUS/ARC_ELEVATION.
    mujoco.mj_forward(model, data)
    scan_points = capture_point_cloud(model, data)
    # Fixed eye-to-hand cameras see the fixture and table as well as the part.
    # Crop to the calibrated workpiece ROI before fitting; otherwise the 5 cm
    # table/workpiece height step is mixed into every x-bin and corrupts normals.
    workpiece_roi = (
        (np.abs(scan_points[:, 0] - WORKPIECE_CENTER_X) <= WORKPIECE_HALF_X) &
        (np.abs(scan_points[:, 1]) <= WORKPIECE_HALF_Y) &
        (scan_points[:, 2] >= Z_SURFACE_GUESS - 0.005) &
        (scan_points[:, 2] <= Z_SURFACE_GUESS + ARC_ELEVATION + 0.015)
    )
    scan_points = scan_points[workpiece_roi]
    hid = model.hfield("workpiece_hf").id
    x_half = model.hfield_size[hid][0]
    normal_x_bins, normal_lut = fit_normals_along_x(
        scan_points, WORKPIECE_CENTER_X - x_half, WORKPIECE_CENTER_X + x_half)

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))

    n_steps = int(SIM_TIME / model.opt.timestep)

    viewer_ctx = None
    if LIVE_VIEW:
        import mujoco.viewer as mj_viewer
        viewer_ctx = mj_viewer.launch_passive(model, data)
        if VIEW_CAMERA == "free":
            # A whole-cell starting pose that remains fully mouse-adjustable:
            # left drag rotates, right drag pans, and the wheel zooms.
            viewer_ctx.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer_ctx.cam.lookat[:] = np.array([0.50, 0.00, 0.40])
            viewer_ctx.cam.distance = 1.42
            viewer_ctx.cam.azimuth = 125.0
            viewer_ctx.cam.elevation = -24.0
        elif VIEW_CAMERA in ("cell_view", "tool_view", "camera_view"):
            viewer_ctx.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer_ctx.cam.fixedcamid = model.camera(VIEW_CAMERA).id
        else:
            raise ValueError(
                f"Unknown --camera={VIEW_CAMERA!r}; use free, cell_view, tool_view, "
                "or camera_view"
            )

    log = {"t": [], "x": [], "y": [], "z": [], "z_d": [], "fz": [],
           "fraw": [], "ffast": [], "fresultant": [], "fresultant_fast": [],
           "safety_tripped": []}
    stopped_by_user = False

    # In --view mode, keep replaying the same 20 s run until the user closes
    # the viewer window (checked via viewer_ctx.is_running()). Headless mode
    # runs exactly once, same as before.
    while True:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        data.ctrl[pneumatic_act_id] = 0.0
        mujoco.mj_forward(model, data)

        log = {"t": [], "x": [], "y": [], "z": [], "z_d": [], "fz": [],
               "fraw": [], "ffast": [], "fresultant": [], "fresultant_fast": [],
               "safety_tripped": []}
        f_integral = 0.0
        f_filt = 0.0
        f_fast = 0.0       # fast normal-force safety monitor
        f_resultant_fast = 0.0  # normal-estimate-independent force-magnitude monitor
        safety_tripped = False  # latched for the rest of this run after any over-force
        retreating = False
        fault_hold_pos = None
        overforce_time = 0.0
        reported_collision_pairs = set()

        for step in range(n_steps):
            t = step * model.opt.timestep
            dt = model.opt.timestep
            # Fail-safe default: the pneumatic cylinder is vented unless the
            # normal force-control branch below explicitly commands pressure.
            data.ctrl[pneumatic_act_id] = 0.0

            # Report unexpected structural collisions once per pair. They should
            # never be silently mistaken for polishing-pad contact.
            for con in data.contact:
                body1 = model.body(model.geom_bodyid[con.geom1]).name
                body2 = model.body(model.geom_bodyid[con.geom2]).name
                pair = tuple(sorted((body1, body2)))
                expected_pad_contact = pair == ("pneumatic_slide", "workpiece")
                if not expected_pad_contact and pair not in reported_collision_pairs:
                    reported_collision_pairs.add(pair)
                    print(f"UNEXPECTED COLLISION at t={t:.3f}s: {pair[0]} <-> {pair[1]}")

            mujoco.mj_jacSite(model, data, jacp, jacr, tcp_site_id)
            J_full = np.vstack([jacp, jacr])
            J = J_full[:, arm_dof_idx]  # 6x6 arm Jacobian for torque mapping

            x_cur = data.site_xpos[tcp_site_id].copy()
            R_cur = data.site_xmat[tcp_site_id].reshape(3, 3).copy()

            # translational + rotational velocity of the TCP (world frame)
            # Sensor/TCP velocity must include the pneumatic slide DOF. Omitting it
            # makes normal damping blind to the pad bouncing on its compliant axis.
            vel6 = J_full @ data.qvel
            x_dot, omega = vel6[:3], vel6[3:]

            # Local surface normal at the tool's current x, from the pre-scanned
            # point cloud (see chapter 12). Used for THREE things now: the tool
            # orientation target, the direction we regulate contact force along,
            # and the plane we do tangential impedance tracking in. On the flat
            # region this is (0,0,1) and everything below reduces exactly to the
            # old world-frame decomposition.
            local_normal = estimate_normal_at_x(x_cur[0], normal_x_bins, normal_lut)

            # low-pass filter the contact-force component ALONG THE SURFACE NORMAL
            # (not just world-z), so force control is genuinely normal-direction.
            # get_contact_force_on_pad returns the reaction force ON the pad; its
            # projection onto the outward normal is the pressing force we regulate.
            f_raw = get_contact_force_on_pad(model, data, pad_geom_id)
            f_n_raw = f_raw @ local_normal
            # Perception-independent final safety channel: a bad point-cloud normal
            # can underestimate f_raw@n, but it cannot hide the force-vector norm.
            f_resultant_raw = np.linalg.norm(f_raw)
            f_filt += F_FILT_ALPHA * (f_n_raw - f_filt)

            # Independent FAST safety monitor on the same raw signal -- not used for
            # f_err (that stays on the slow f_filt above), only for the retreat trip
            # below. Hysteresis (two different thresholds) avoids flapping in and out
            # of retreat right at the boundary.
            f_fast += F_FAST_ALPHA * (f_n_raw - f_fast)
            f_resultant_fast += F_FAST_ALPHA * (f_resultant_raw - f_resultant_fast)
            if not safety_tripped:
                if (f_fast > F_RETREAT_TRIGGER or
                        f_resultant_fast > F_RESULTANT_TRIGGER):
                    overforce_time += dt
                else:
                    overforce_time = 0.0

            if not safety_tripped and overforce_time >= F_RETREAT_PERSIST_TIME:
                safety_tripped = True
                retreating = True
                f_integral = 0.0
                print(f"SAFETY TRIP at t={t:.3f}s: fast normal/resultant force "
                      f"{f_fast:.2f}/{f_resultant_fast:.2f}N exceeded "
                      f"{F_RETREAT_TRIGGER:.2f}/{F_RESULTANT_TRIGGER:.2f}N for "
                      f"{overforce_time * 1000:.0f}ms")
            elif safety_tripped and retreating and f_fast < F_RETREAT_CLEAR:
                retreating = False
                fault_hold_pos = x_cur.copy()
                print(f"SAFETY LOCKOUT at t={t:.3f}s: contact released; "
                      "holding away from the surface until reset")

            # A circular pneumatic sander only needs its spindle/pressing axis to
            # follow the surface normal; yaw about that axis is physically
            # irrelevant. Controlling all three orientation DOFs over-constrains a
            # non-redundant 6-axis arm and can force an unnecessary wrist flip.
            tool_press_axis = R_cur[:, 2]
            desired_press_axis = -local_normal
            rot_err = np.cross(tool_press_axis, desired_press_axis)
            rot_wrench = KP_ROT * rot_err - KD_ROT * omega

            if safety_tripped:
                if retreating:
                    # First get away from the surface as quickly as possible.
                    F_xyz = F_RETREAT_FORCE * local_normal
                else:
                    # A trip is latched: do not automatically resume the raster/PI
                    # loop. Hold the released pose with Cartesian impedance instead.
                    hold_err = fault_hold_pos - x_cur
                    F_xyz = KP_FAULT_HOLD * hold_err - KD_FAULT_HOLD * x_dot
                z_d_log = x_cur[2]
            elif t < APPROACH_TIME:
                # --- phase 1: pure position control, descend onto the surface ---
                alpha = np.clip(t / APPROACH_TIME, 0.0, 1.0)
                alpha = alpha * alpha * (3.0 - 2.0 * alpha)
                x_d_xy = raster_xy(0.0)  # descend right onto the raster's starting point
                # The TCP is at the centre of the spherical pad, so first contact is
                # one pad radius above the workpiece. Driving the TCP itself to 0.30m
                # used to indent the 15mm pad into the surface and caused an avoidable
                # ~11N impact just before switching to force control.
                z_d_approach = APPROACH_Z * (1 - alpha) + APPROACH_CONTACT_Z * alpha
                pos_err = np.array([x_d_xy[0] - x_cur[0], x_d_xy[1] - x_cur[1],
                                     z_d_approach - x_cur[2]])
                F_xyz = np.array([
                    KP_POS_XY[0] * pos_err[0] - KD_POS_XY[0] * x_dot[0],
                    KP_POS_XY[1] * pos_err[1] - KD_POS_XY[1] * x_dot[1],
                    KP_POS_Z_APPROACH * pos_err[2] - KD_POS_Z_APPROACH * x_dot[2],
                ])
                z_d_log = z_d_approach
            else:
                # --- phase 2: hybrid control in the LOCAL SURFACE FRAME ---
                #   * normal direction  -> explicit force control (PI + damping)
                #   * tangent plane      -> impedance tracking of the raster path
                # This is the proper Raibert-Craig tangent/normal decomposition:
                # the force axis and the motion plane both rotate with the surface,
                # rather than being locked to world z / world xy. On the flat half
                # (normal = +z) it is algebraically identical to the old version.
                n = local_normal

                # -- normal-direction explicit force control --
                # Do not apply an 8N step at first contact. A direct step produced a
                # large contact impulse even with conservative PI gains, because the
                # feed-forward F_TARGET term itself jumps discontinuously. Smoothstep
                # gives zero slope at both ends of the force ramp.
                force_ramp = np.clip((t - APPROACH_TIME) / FORCE_RAMP_TIME, 0.0, 1.0)
                force_ramp = force_ramp * force_ramp * (3.0 - 2.0 * force_ramp)
                f_target_cmd = F_TARGET * force_ramp
                pneumatic_cmd = PNEUMATIC_PRELOAD * force_ramp
                data.ctrl[pneumatic_act_id] = pneumatic_cmd
                f_err = f_target_cmd - f_filt
                f_integral = np.clip(f_integral + f_err * dt, -I_CLAMP, I_CLAMP)
                vn = x_dot @ n  # tool velocity component along the normal
                # Pneumatic thrust is partly balanced by the slide spring and is
                # not equal to force delivered at the workpiece. Keep the full
                # robot feed-forward and let measured-force feedback account for
                # the cylinder/spring/contact transmission automatically.
                Fn_cmd = -(f_target_cmd + KP_F * f_err + KI_F * f_integral) - BZ * vn
                # Scale the command ceiling during ramp-up as well; otherwise the
                # static 9N limit would still permit a near-step at t=3s.
                f_max_now = min(F_MAX, f_target_cmd + 1.0)
                Fn_cmd = max(Fn_cmd, -f_max_now)  # soft ceiling, see F_MAX above
                F_normal = Fn_cmd * n  # a force vector along -n, into the surface

                # -- tangent-plane impedance tracking the raster target --
                xy = raster_xy(t - APPROACH_TIME)
                pos_err = np.array([xy[0] - x_cur[0], xy[1] - x_cur[1], 0.0])
                pos_err_tan = pos_err - (pos_err @ n) * n   # project onto tangent plane
                v_tan = x_dot - (x_dot @ n) * n             # tangential velocity only
                F_tangent = KP_POS_XY[0] * pos_err_tan - KD_POS_XY[0] * v_tan

                F_xyz = F_normal + F_tangent
                z_d_log = x_cur[2]  # no explicit z setpoint in force-control phase

            F_task = np.concatenate([F_xyz, rot_wrench])
            tau_task = J.T @ F_task
            tau = tau_task + data.qfrc_bias[arm_dof_idx]

            for k, aidx in enumerate(arm_act_idx):
                lo, hi = model.actuator_ctrlrange[aidx]
                data.ctrl[aidx] = np.clip(tau[k], lo, hi)

            mujoco.mj_step(model, data)
            if viewer_ctx is not None:
                viewer_ctx.sync()
                if not viewer_ctx.is_running():
                    stopped_by_user = True
                    break

            log["t"].append(t)
            log["x"].append(x_cur[0])
            log["y"].append(x_cur[1])
            log["z"].append(x_cur[2])
            log["z_d"].append(z_d_log)
            log["fz"].append(f_filt)
            log["fraw"].append(f_n_raw)
            log["ffast"].append(f_fast)
            log["fresultant"].append(f_resultant_raw)
            log["fresultant_fast"].append(f_resultant_fast)
            log["safety_tripped"].append(safety_tripped)

        if not LIVE_VIEW or stopped_by_user:
            break

    if viewer_ctx is not None:
        viewer_ctx.close()

    return log


if __name__ == "__main__":
    log = main()

    t_log = np.asarray(log["t"])
    fz_arr = np.asarray(log["fz"])
    ffast_arr = np.asarray(log["ffast"])
    fraw_arr = np.asarray(log["fraw"])
    fresultant_arr = np.asarray(log["fresultant"])
    fresultant_fast_arr = np.asarray(log["fresultant_fast"])

    # "Steady state" for statistics purposes: past the force ramp, and only while
    # actually in contact (skip the brief near-zero dips between raster rows,
    # which are real but would otherwise dominate a peak-to-peak/variance number
    # that is meant to describe tracking quality, not row-transition behavior).
    steady_mask = (t_log >= APPROACH_TIME + FORCE_RAMP_TIME) & (fz_arr > 4.0)
    fz_steady = fz_arr[steady_mask]
    steady_mean = fz_steady.mean()
    steady_std = fz_steady.std()
    steady_var = fz_steady.var()
    steady_p2p = fz_steady.max() - fz_steady.min()

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))

    # -- (0,0) Contact force tracking --
    # Plot order matters: matplotlib draws later calls on top of earlier ones. Layer
    # back-to-front from the noisiest/most "raw" signal to the cleanest: raw per-step
    # MEASURED force (fraw -- this is the actual physical reading, unfiltered -- as
    # real as it gets without cheating with analytic geometry) sits at the back, the
    # fast safety-monitor filter in the middle, and the slow filter that actually
    # drives force control drawn last/thickest so it stays legible on top.
    ax = axes[0, 0]
    ax.plot(t_log, fraw_arr, color="0.6", alpha=0.35, lw=0.4, zorder=1,
            label="raw measured $f_n$ (unfiltered, this is the real per-step reading)")
    ax.plot(t_log, ffast_arr, color="tab:orange", alpha=0.6, lw=0.7, zorder=2,
            label="fast safety monitor (light filter)")
    ax.plot(t_log, fz_arr, color="tab:blue", lw=1.6, zorder=3,
            label="measured $f_n$ (slow filter, drives force control)")
    ax.axhline(F_TARGET, color="r", ls="--", zorder=4, label="target $f_n$")
    ax.axhline(F_RETREAT_TRIGGER, color="darkorange", ls=":", zorder=4,
               label="safety trip threshold")
    ax.set_xlabel("time [s]"); ax.set_ylabel("normal force [N]")
    ax.set_title("Contact force tracking")
    stats_text = (f"steady-state (in contact, post-ramp):\n"
                  f"mean = {steady_mean:.2f} N\n"
                  f"std (标准差) = {steady_std:.3f} N\n"
                  f"variance (方差) = {steady_var:.3f} N²\n"
                  f"peak-to-peak (极差) = {steady_p2p:.2f} N")
    ax.text(0.98, 0.03, stats_text, transform=ax.transAxes, fontsize=8,
            ha="right", va="bottom",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="gray"))
    # The approach phase (t < APPROACH_TIME) is exactly 0 before contact, so the
    # upper-left corner is guaranteed clear of data regardless of how the force
    # trace behaves afterward -- unlike "upper right" (used previously), which sat
    # directly on top of the fast-monitor spikes.
    ax.legend(fontsize=7, loc="upper left")
    ax.grid(alpha=0.3)

    # -- (0,1) Height / z tracking --
    ax = axes[0, 1]
    ax.plot(t_log, log["z"], label="TCP z (actual)")
    ax.plot(t_log, log["z_d"], label="z setpoint (approach phase only)", ls="--")
    ax.axhline(Z_SURFACE_GUESS, color="gray", ls=":", label="surface")
    ax.set_xlabel("time [s]"); ax.set_ylabel("height [m]")
    ax.set_title("Height / force-loop setpoint"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # -- (0,2) TCP top-view path --
    ax = axes[0, 2]
    ax.plot(log["x"], log["y"])
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("TCP path on the workpiece (top view)")
    ax.set_aspect("equal"); ax.grid(alpha=0.3)

    # -- (1,0) Force distribution during steady polishing --
    # A histogram makes the peak-to-peak/variance numbers in the panel above
    # visually concrete instead of just text: this is what "0.3 N std" actually
    # looks like as a spread of samples around the 8 N target.
    ax = axes[1, 0]
    ax.hist(fz_steady, bins=40, color="tab:blue", alpha=0.75)
    ax.axvline(F_TARGET, color="r", ls="--", label="target $f_n$")
    ax.axvline(steady_mean, color="k", ls="-", lw=1, label="mean")
    ax.axvline(steady_mean - steady_std, color="k", ls=":", lw=1, label="mean ± std")
    ax.axvline(steady_mean + steady_std, color="k", ls=":", lw=1)
    ax.set_xlabel("normal force [N]"); ax.set_ylabel("sample count")
    ax.set_title("Steady-state force distribution"); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # -- (1,1) Resultant (perception-independent) safety channel --
    # This is |f_raw| -- the FULL 3D contact force vector's length, normal component
    # plus tangential friction, unlike panel (0,0) which is only the component
    # projected onto the (possibly wrong) estimated normal. It sits well above the
    # 8N normal-force target because the raster motion drags the pad sideways against
    # friction the whole time -- that is expected, not an error, and is exactly why
    # its own trip threshold (20N) is set higher than the normal-force one (10N): a
    # bad point-cloud normal can hide danger from panel (0,0), but it cannot make
    # this line lie, since it never depends on which direction we THINK is "normal".
    ax = axes[1, 1]
    ax.plot(t_log, fresultant_arr, color="tab:purple", alpha=0.35, lw=0.7,
            label="raw |contact force| (normal + friction, unfiltered)")
    ax.plot(t_log, fresultant_fast_arr, color="tab:purple", lw=1.4,
            label="fast-filtered |contact force|")
    ax.axhline(F_RESULTANT_TRIGGER, color="darkorange", ls=":",
               label="resultant trip threshold (20N)")
    ax.axhline(F_TARGET, color="r", ls="--", alpha=0.6,
               label="normal-force target (8N, for reference only)")
    ax.set_xlabel("time [s]"); ax.set_ylabel("force magnitude [N]")
    ax.set_title("Resultant-force safety channel\n(perception-independent backstop, includes friction)")
    ax.legend(fontsize=7, loc="upper left"); ax.grid(alpha=0.3)

    # -- (1,2) Per-row force tracking consistency --
    # Same row time windows used for the coverage check below, reused here to see
    # whether force tracking quality is uniform across the whole scan or gets worse
    # on particular rows (e.g. the steeper part of the arc).
    ax = axes[1, 2]
    row_errors = []
    for row in range(RASTER_ROWS):
        row_start = APPROACH_TIME + row * RASTER_ROW_PERIOD
        row_end = row_start + RASTER_ROW_PERIOD
        row_mask = (t_log >= row_start) & (t_log < row_end) & (fz_arr > 4.0)
        row_errors.append(np.mean(np.abs(fz_arr[row_mask] - F_TARGET)) if row_mask.any() else np.nan)
    ax.bar(np.arange(RASTER_ROWS), row_errors, color="tab:blue")
    ax.set_xlabel("raster row index"); ax.set_ylabel("mean |f_n - target| [N]")
    ax.set_title("Per-row force tracking error"); ax.grid(alpha=0.3, axis="y")

    fig.tight_layout()
    fig.savefig("polish_result.png", dpi=150)
    print("Saved plot to polish_result.png")
    print(f"Final 3 s mean |f_n - target|: "
          f"{np.mean(np.abs(fz_arr[-int(3/(t_log[1]-t_log[0])):] - F_TARGET)):.3f} N")
    print(f"Peak raw/fast/slow normal force: {np.max(log['fraw']):.2f} / "
          f"{np.max(log['ffast']):.2f} / {np.max(log['fz']):.2f} N")
    print(f"Steady-state: mean={steady_mean:.3f}N std={steady_std:.3f}N "
          f"variance={steady_var:.3f}N^2 peak-to-peak={steady_p2p:.3f}N")
    print(f"Safety tripped: {any(log['safety_tripped'])}")
    polish_mask = t_log >= APPROACH_TIME
    x_actual = np.asarray(log["x"])[polish_mask]
    y_actual = np.asarray(log["y"])[polish_mask]
    pad_radius = 0.025
    print(f"Actual TCP coverage x/y: {x_actual.min():.4f}..{x_actual.max():.4f} / "
          f"{y_actual.min():.4f}..{y_actual.max():.4f} m")
    print(f"Actual 50 mm pad coverage x/y: "
          f"{x_actual.min() - pad_radius:.4f}..{x_actual.max() + pad_radius:.4f} / "
          f"{y_actual.min() - pad_radius:.4f}..{y_actual.max() + pad_radius:.4f} m")
    row_ranges = []
    y_all = np.asarray(log["y"])
    for row in range(RASTER_ROWS):
        row_start = APPROACH_TIME + row * RASTER_ROW_PERIOD
        row_end = row_start + RASTER_ROW_PERIOD
        row_mask = (t_log >= row_start) & (t_log < row_end)
        row_ranges.append((y_all[row_mask].min(), y_all[row_mask].max()))
    worst_row_min = max(v[0] for v in row_ranges)
    worst_row_max = min(v[1] for v in row_ranges)
    print(f"Worst per-row TCP y coverage: {worst_row_min:.4f}..{worst_row_max:.4f} m")
    print(f"Worst per-row 50 mm pad y coverage: "
          f"{worst_row_min - pad_radius:.4f}..{worst_row_max + pad_radius:.4f} m")
