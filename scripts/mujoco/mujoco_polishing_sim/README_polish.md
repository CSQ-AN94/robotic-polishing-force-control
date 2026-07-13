# MuJoCo RM65-B + Gemini 336L + 气动打磨头力控仿真

## 环境要求
```bash
pip install mujoco numpy matplotlib
```

## 文件说明
- `rm65b_pneumatic.xml` — 睿尔曼 RM65-B 六轴模型。运动学、惯量、关节范围、
  力矩上限和 STL 外观来自睿尔曼官方 `RealManRobot/rm_robot`（Apache-2.0）；
  末端增加轴向滑动关节、弹簧阻尼、气缸推力和柔性背衬等效砂盘。
- `assets/rm65/` — RM65-B 官方外观网格及其 Apache-2.0 LICENSE。
- `polish_scene.xml` — 保留原来的 `36×36 cm` 工件和 4cm 圆弧高度；包含工作台、
  固定式相机架、整机视角 `cell_view` 和工具视角 `tool_view`。
- `polish_scene.xml` 中的 `scan_cam` — 固定式 eye-to-hand 奥比中光 Gemini 336L
  等效相机；工作模式按真实采集栈设为 640×480，仿真位姿就是已知外参。
- `polish_impedance_control.py` — 六轴任务空间阻抗控制 + 法向显式力控 + 气动预载。
- `polish_result.png` — 一次完整仿真（20s）的验证结果图。

## 运行方式

先进入目录：
```bash
cd "/Users/siqi.cai/Robot Manipulation/Force Control/scripts/mujoco/mujoco_polishing_sim"
```

**无头模式**（不弹窗口，只生成 `polish_result.png`，用项目主 `.venv`）：

```bash
"/Users/siqi.cai/Robot Manipulation/Force Control/.venv/bin/python" polish_impedance_control.py
```

**GUI 模式**（弹出交互式 3D 窗口，用专门的 `.viewer_venv`）：

```bash
".viewer_venv/bin/python3.13" ".viewer_venv/bin/mjpython" polish_impedance_control.py --view
```

```bash
pip install mujoco numpy matplotlib
cd mujoco_polishing_sim
python polish_impedance_control.py          # 无头模式
python polish_impedance_control.py --view    # 如果是 macOS，GUI 模式要用 mjpython

cd "/Users/siqi.cai/Robot Manipulation/Force Control/scripts/mujoco/mujoco_polishing_sim"
".viewer_venv/bin/python3.13" ".viewer_venv/bin/mjpython" polish_impedance_control.py --view
```

GUI 模式会一直循环播放（跑完一轮自动重开），直到你自己关掉窗口才停止。默认机位是自动对准打磨头的固定相机，窗口里按 `Tab` 键可以切回自由视角。

之所以 GUI 要换一套单独的 `.viewer_venv`，是因为项目主 `.venv` 底层的便携版 Python 和 MuJoCo 在 macOS 上弹窗口用的 `mjpython` 动态库加载机制不兼容；另外因为项目路径带空格，不能直接执行 `mjpython`，要显式用 `python3.13` 去解释它。

## 控制器结构（重点，写进你给陈老师的进展报告可以直接用）
1. 1**下降阶段 (t<3s)**：纯任务空间阻抗控制，x/y/z/姿态全部用 PD 阻抗律平滑下压到接触面附近。
2. **打磨阶段 (t>=3s)**：
   - x, y 依然用位置阻抗控制，走一个覆盖整个工件的光栅扫描轨迹（来回之字形，`raster_xy()`），
     而不是原来只在中间画一个小圆——这样才是真的"打磨整个平面"，扫描范围横跨平面和圆弧两个区域。
   - z 轴切换成**显式力控**（explicit force control）：不再用位置误差，而是直接由
     低通滤波后的接触力反馈，经 PI + 阻尼，直接算出要施加的法向力指令。
   - 关键教训（调参时踩的坑）：如果在 z 轴同时保留"位置阻抗刚度"和"力误差积分"，
     两者会叠加导致刚度爆表、打磨头在接触面上反复弹跳（我一开始就是这么写的，
     测出来力从 -70N 跳到 0N 来回震荡）。改成 z 轴纯用力指令 + 阻尼项之后，
     接触力才稳定收敛到目标值附近（8N 目标，稳态误差 <1N）。
   - 首次接触时目标力通过 1.5s smoothstep 从 0 平滑爬升到 8N，避免直接施加
     8N 阶跃造成冲击。当前力环参数为 `KP_F=0.1`、`KI_F=0.2`；固定随机种子的
     参数扫描中，这组参数比原来的 `0.3/0.5` 有更低的慢滤波峰值和波动。
   - 安全保护采用独立快滤波：实际法向力持续高于 10N 或接触合力持续高于 20N
     达 25ms 后立即沿法线撤离；合力通道不依赖点云法线，可兜底感知错误。
     接触力降到 2N 以下后进入锁定保持，本轮不会自动恢复打磨。这样短暂的 MuJoCo
     接触求解器数值尖峰不会误停，但持续过力不会造成反复“撤离—压回—再撤离”。
3. `qfrc_bias`（MuJoCo 自动算出的重力+科氏力项）作为前馈加到力矩指令上，
   这样阻抗/力控 PD 只需要处理"任务误差"，不用再单独写重力补偿。

## 可以直接迁移到你后续学习路径的地方
- 这套"任务空间阻抗 + 显式力控混合"的结构，和你已经读过的 robosuite OSC 控制器、
  deoxys_control 的 Cartesian impedance 模式是同一套逻辑，只是这里是从零手写、
  完全可控每一行，适合用来彻底搞懂原理。
- 想验证阻抗刚度对接触稳定性的影响，可以直接调 `KP_POS_XY`、`BZ`、`KP_F`、`KI_F`
  这几个增益，重新跑一遍看 `polish_result.png` 里力曲线的震荡程度。
- 踩过一个坑值得记录：把轨迹从小圆换成覆盖整个工件的光栅扫描后，范围变大了 3 倍多，
  原来给小圆调好的 `KP_POS_XY=300` 太硬，侧向修正力"漏"进了 z 轴力控制环，力曲线出现
  20N+ 的尖峰。换行/掉头的平滑处理都没解决根本问题，最后是把 `KP_POS_XY` 降到 100、
  RM65-B 六轴版本进一步把 `KP_POS_XY` 降到 60、`RASTER_ROW_PERIOD` 放慢到 12s 才稳定下来——阻抗增益是按具体任务的空间范围和
  速度调的，不是万能常数，任务尺度变了就要重新调。详见 `notes/mujoco_polishing_textbook.docx`
  第十章。

## 当前验证结果（RM65-B 版本）

完整 6 行中央光栅跨过平面和圆弧后：最后 3 秒平均法向力误差 `0.013 N`，原始/快滤波/
慢滤波法向力峰值分别为 `8.86/8.51/8.15 N`，安全保护未误触发。这里的 Gemini 336L
仍是仿真相机；接真实相机时必须用设备内参和 eye-to-hand 标定得到的外参替换仿真真值。

原工件几何不变，但 Franka 的 `x=0.34–0.66m, y=±0.16m` 全幅光栅不适合 RM65-B
六轴臂加长气动头：保持砂盘法向时远角会超出可达空间。RM65-B 使用中央
`x=0.46–0.56m, y=±0.10m` 区域，仍横跨平面和圆弧，并已逐端点做 IK 检查。
工具沿 RM65-B 法兰真实 `-Z` 出轴安装；砂盘直径缩为 50mm，与原工件比例协调。
