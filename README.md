# Robotic Polishing Force Control

MuJoCo 仿真：睿尔曼 RM65-B 六轴机械臂 + 固定式 Orbbec Gemini 336L 深度相机 + 气动柔顺打磨头，打磨一块半平面、半真圆弧的工件。核心是一套感知驱动的混合阻抗/显式力控制器——用深度相机点云实时拟合工件表面的局部法线，让力控制沿真实法线方向、运动跟踪沿真实切平面方向，而不是假设工件是水平的；外层再套一套三层的力安全机制（命令限幅、双通道快滤波监控、撤离并锁定状态机）。

配套一份非常详细的教材，从 Python/NumPy 语法基础一路讲到控制理论推导、代码实现、真实调参案例和安全机制设计依据。

## 目录结构

```text
notes/
  mujoco_polishing_textbook.md   完整教材（先读这个）
  组会汇报准备.md                 原理+实现的答辩准备材料，含控制框图
  session_handoff_perception_safety.md   开发过程的交接记录
  力控打磨输出图/                 若干代表性结果图（不同噪声水平下的表现）
  img/                           教材配图（含控制框图 SVG）

scripts/mujoco/mujoco_polishing_sim/
  polish_impedance_control.py    主控制脚本
  polish_scene.xml               场景（工件、相机、工作台）
  rm65b_pneumatic.xml            RM65-B + 气动打磨头模型
  panda_torque.xml               历史对照模型（早期 Franka Panda 原型，仅文本，网格未包含）
  assets/rm65/, assets/orbbec/   RM65-B 与 Gemini 336L 官方外观网格（各自附 LICENSE）
  README_polish.md               这个子项目自己的运行说明
```

## 怎么跑起来

无头模式（不弹窗口，跑完生成 `polish_result.png`）：

```bash
cd scripts/mujoco/mujoco_polishing_sim
python -m venv .venv && source .venv/bin/activate
pip install mujoco numpy matplotlib
python polish_impedance_control.py
```

GUI 模式（macOS 需要用 `mjpython`，见 [scripts/mujoco/mujoco_polishing_sim/README_polish.md](scripts/mujoco/mujoco_polishing_sim/README_polish.md) 里的详细说明）：

```bash
mjpython polish_impedance_control.py --view
```

## 从哪里开始读

先看 [notes/mujoco_polishing_textbook.md](notes/mujoco_polishing_textbook.md) 的"读者假设与范围"和"大地图"部分建立整体印象，再逐章往下读；如果只是想快速看懂控制架构，可以直接跳到教材里"控制系统的大闭环"那一节的框图。

## 关于历史 Franka Panda 资源

`panda_torque.xml` 保留作历史对照（教材附录里有说明它和现在 RM65-B 版本的区别），但它引用的网格文件（`.stl`/`.obj`）因为来源和授权不明确，没有包含在这个仓库里，所以这个历史文件目前只能读文本、不能直接可视化加载。如果你有自己的 Franka Panda 描述文件（比如来自官方或 MuJoCo Menagerie），把对应网格放进 `assets/` 目录同名位置即可恢复可视化。

---

## 本地工作区其它入口（仅供参考，未包含在本仓库范围内）

这个仓库是从一个更大的本地学习工作区里挑出来的一部分。工作区里还有下面这些跟本仓库无关的内容，仅记录在这里方便本地查阅，不随本仓库发布：

```text
papers/                 论文 PDF
scripts/fundamentals/   基础阻抗 / 质量-弹簧-阻尼小实验
scripts/robosuite/      robosuite / OSC_POSE 探针和 GUI 脚本
visualizers/            HTML/CSS/JS 可视化页面
data/                   仿真日志、JSON 数据等可复用数据
artifacts/              OCR 页面图、处理产物等中间文件
third_party/            外部或阶段性移植代码
```
