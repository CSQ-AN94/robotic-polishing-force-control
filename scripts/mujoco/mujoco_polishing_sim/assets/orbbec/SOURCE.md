# Orbbec Gemini 336L simulation asset

The camera enclosure used by `polish_scene.xml` is the official shared Gemini 335L/336L
`base_link.STL` from Orbbec's `OrbbecSDK_ROS2` repository, branch `v2-main`.

- Official repository: <https://github.com/orbbec/OrbbecSDK_ROS2>
- Mesh source: <https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_description/meshes/gemini335L_336L/base_link.STL>
- URDF source: <https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/orbbec_description/urdf/gemini_335_L_336_L.urdf.xacro>
- Official CAD download page: <https://doc.orbbec.com/documentation/Orbbec%20Gemini%20330%20Series%20Documentation/Download%20CAD%20Files%20%28Gemini%20330%20Series%29>
- Official enclosure dimensions: 124 mm × 29 mm × 27.7 mm
- License: Apache License 2.0; the repository license is copied to `LICENSE`.

Checksums:

```text
9de399ed805ddb004cacdf7656766d728d0e4fb9e9c83c70a3c12950c7db8303  gemini335L_336L_base_link.STL
e7ea24519f4a1025cfee099e78c7f76ae2a57569581b1c6460455193591100d8  gemini_335_L_336_L.urdf.xacro
1a325fd390b54e7b142d31dfcbfa4ed4621b83d2ec8e3b9fad054d491b3a4c49  LICENSE
```

The mesh remains unchanged. `polish_scene.xml` only rotates and recentres it so the
physical optical face points down toward the workpiece. The three dark lens accents are
MuJoCo primitives placed using the 95 mm stereo baseline and 23.75 mm RGB offset from
the official URDF. They make the sensor face readable because STL does not carry color.
