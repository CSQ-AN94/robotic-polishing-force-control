# Robosuite GUI 仿真启动说明

## 已配置好的环境

GUI 仿真使用 conda 环境：

```text
robo
```

环境路径：

```text
/Users/siqi.cai/miniconda3/envs/robo
```

这个环境里已经安装：

```text
robosuite 1.5.2
mujoco 3.10.0
```

## 为什么不用 `pip install robosuite`

你的系统终端里没有全局 `pip`，所以直接运行：

```bash
pip install robosuite
```

会报：

```text
zsh: command not found: pip
```

以后更稳的写法是：

```bash
python -m pip install robosuite
```

但现在不用重新安装，`robo` 环境已经装好了。

## 为什么 GUI 要用 `mjpython`

macOS 上 MuJoCo viewer 不能直接用普通 Python 打开，通常要通过 `mjpython` 启动。

本项目里可用的 GUI 启动命令是：

```bash
/Users/siqi.cai/miniconda3/envs/robo/bin/python \
  /Users/siqi.cai/miniconda3/envs/robo/bin/mjpython \
  scripts/robosuite/robosuite_osc_viewer.py
```

## 新的主 GUI：看完整控制链

现在更推荐先跑这个：

```bash
/Users/siqi.cai/miniconda3/envs/robo/bin/python \
  /Users/siqi.cai/miniconda3/envs/robo/bin/mjpython \
  scripts/robosuite/robosuite_osc_flow_viewer.py
```

这个窗口里会同时看到：

```text
机器人仿真画面
右侧实时流程面板
真实夹爪：当前末端位置
绿色小球：末端目标位置
橙色细线：末端位置误差
紫色细线：期望末端力
蓝/红柱：7 个关节力矩
```

这才对应你现在真正要看的主线：

```text
action -> 末端目标 -> 末端误差 -> 末端力 -> 关节力矩
```

关掉 MuJoCo 窗口就会停止程序。

如果想看 y 或 z 方向，把命令改成：

```bash
/Users/siqi.cai/miniconda3/envs/robo/bin/python \
  /Users/siqi.cai/miniconda3/envs/robo/bin/mjpython \
  scripts/robosuite/robosuite_osc_flow_viewer.py --axis y

/Users/siqi.cai/miniconda3/envs/robo/bin/python \
  /Users/siqi.cai/miniconda3/envs/robo/bin/mjpython \
  scripts/robosuite/robosuite_osc_flow_viewer.py --axis z
```

如果觉得变化太快，用慢速版：

```bash
/Users/siqi.cai/miniconda3/envs/robo/bin/python \
  /Users/siqi.cai/miniconda3/envs/robo/bin/mjpython \
  scripts/robosuite/robosuite_osc_flow_viewer.py --fps 8
```

## 旧的单轴 GUI：只看末端运动

运行后会打开 MuJoCo/robosuite 窗口。新版脚本默认只演示 x 轴：

```text
action[0] 给正值 -> 夹爪沿 +x 方向动
停一下
action[0] 给负值 -> 夹爪沿 -x 方向动回来
停一下
重复 3 次
```

这一步不要急着看阻抗公式。它只回答一个问题：

```text
action[0] 到底对应 Panda 末端的哪个运动方向？
```

默认启动命令：

```bash
/Users/siqi.cai/miniconda3/envs/robo/bin/python \
  /Users/siqi.cai/miniconda3/envs/robo/bin/mjpython \
  scripts/robosuite/robosuite_osc_viewer.py
```

看完 x 轴以后，再分别运行：

```bash
/Users/siqi.cai/miniconda3/envs/robo/bin/python \
  /Users/siqi.cai/miniconda3/envs/robo/bin/mjpython \
  scripts/robosuite/robosuite_osc_viewer.py --axis y

/Users/siqi.cai/miniconda3/envs/robo/bin/python \
  /Users/siqi.cai/miniconda3/envs/robo/bin/mjpython \
  scripts/robosuite/robosuite_osc_viewer.py --axis z
```

你的观察标准很简单：

```text
--axis x 时，主要看 dx 变大变小
--axis y 时，主要看 dy 变大变小
--axis z 时，主要看 dz 变大变小
```

如果你看不清，先盯住黑色夹爪尖端相对桌面方块的位置变化，不要看整条机械臂。

## 如果你想自己在终端里操作

可以先激活环境：

```bash
conda activate robo
```

然后运行：

```bash
python /Users/siqi.cai/miniconda3/envs/robo/bin/mjpython scripts/robosuite/robosuite_osc_viewer.py
```

如果终端找不到 `conda`，用完整路径：

```bash
/Users/siqi.cai/miniconda3/bin/conda activate robo
```

不过有些 shell 不能直接用完整路径 activate；最稳的方式还是使用上面的完整 `python mjpython` 命令。
