# 机械臂力控学习清单

你现在不要继续扩大范围。当前阶段只学一件事：

> robosuite 的 `OSC_POSE` 控制器如何把末端位姿误差变成关节力矩。

也就是这条链：

```text
action -> goal_pos -> position_error -> desired_force -> wrench -> joint torques
```

对应数学直觉是：

\[
F = K(x_d - x) + B(\dot{x}_d - \dot{x})
\]

在 robosuite 代码里，\(K\) 对应 `kp`，\(B\) 对应 `kd`。

## 你今天只做三件事

### 1. 看完整控制链

打开：

```text
http://127.0.0.1:8771/visualizers/robosuite_osc/osc_learning_dashboard.html
```

你要看懂的不是“机器人动了”，而是这条链每一步在做什么：

```text
action -> 末端目标 -> 末端误差 -> 末端力 -> 关节力矩
```

页面上先按这个顺序点五个流程节点：

```text
1 action
2 末端目标
3 末端误差
4 末端力
5 关节力矩
```

然后拖动 step，看 step 0、step 39、step 40、step 79。

你要能说出：

> `action[0]` 不是直接发给电机的力矩。它先被缩放成末端目标增量，然后产生末端位置误差，再通过阻抗公式得到末端力，最后通过 Jacobian 转成 7 个关节的力矩。

### 2. 看 probe 脚本

打开：

```text
scripts/robosuite/robosuite_osc_probe.py
```

只看这几段：

```python
controller_config = load_composite_controller_config(robot="Panda")
```

这句加载 Panda 默认控制器。默认控制器是 `OSC_POSE`。

```python
action[0] = 0.2
```

这句给 x 方向命令。

```python
obs, reward, done, info = env.step(action)
```

这句把命令送进仿真，robosuite 内部会调用控制器计算关节力矩。

你要回答：

```text
action[0]、action[1]、action[2] 分别控制什么方向？
```

先猜答案，然后改脚本验证。

### 3. 看 OSC 源码

打开：

```text
.venv/lib/python3.12/site-packages/robosuite/controllers/parts/arm/osc.py
```

只看这几个位置。

第一处，`kp` 和 `kd`：

```python
self.kp = self.nums2array(kp, 6)
self.kd = 2 * np.sqrt(self.kp) * damping_ratio
```

你要理解：

```text
kp 越大，末端越想快速贴近目标，表现得更硬。
damping_ratio 越大，kd 越大，速度误差被压得更厉害，表现得更稳。
```

第二处，末端位置误差：

```python
position_error = desired_world_pos - self.ref_pos
```

你要理解：

```text
desired_world_pos 是目标末端位置。
self.ref_pos 是当前末端位置。
position_error 就是目标和当前位置之间的差。
```

第三处，误差变成期望力：

```python
desired_force = position_error * kp + vel_pos_error * kd
```

这就是你之前学的弹簧-阻尼公式在 3D 末端空间里的版本。

第四处，末端力变成关节力矩：

```python
self.torques = J_full.T @ decoupled_wrench + torque_compensation
```

你要理解：

```text
末端空间的力不能直接发给电机。
电机要的是每个关节的力矩 tau。
所以 robosuite 用 Jacobian 转置 J^T 把末端 wrench 转成关节 torques。
```

## 你暂时不要学什么

现在不要看：

```text
FACTR
VLA
deoxys_control
真实 Franka 硬件接口
整仓库结构
```

这些都不是现在的主线。你现在的主线只有：

```text
action -> 末端目标 -> 末端误差 -> 末端力 -> 关节力矩
```

## 下一次实验

做三个小实验，每次只改一处。

### 实验 A：验证 x/y/z action

在 `scripts/robosuite/robosuite_osc_probe.py` 里改：

```python
action[0] = 0.2
```

分别改成：

```python
action[1] = 0.2
action[2] = 0.2
```

每改一次运行：

```bash
.venv/bin/python scripts/robosuite/robosuite_osc_probe.py
```

然后刷新轨迹页面。

你要写出：

```text
action[0] -> Δx 变化最大
action[1] -> Δy 变化最大
action[2] -> Δz 变化最大
```

### 实验 B：改刚度 `kp`

在 `scripts/robosuite/robosuite_osc_probe.py` 加一行：

```python
arm_cfg["kp"] = 50
```

放在：

```python
arm_cfg = controller_config["body_parts"]["right"]
```

后面。

然后再试：

```python
arm_cfg["kp"] = 300
```

你要观察：

```text
kp 小的时候，末端跟目标更软。
kp 大的时候，末端更强烈地跟目标。
```

### 实验 C：改阻尼 `damping_ratio`

同样加：

```python
arm_cfg["damping_ratio"] = 0.2
```

再试：

```python
arm_cfg["damping_ratio"] = 2.0
```

你要观察：

```text
damping_ratio 小，可能更容易有冲过头或抖动。
damping_ratio 大，响应更稳但可能更迟钝。
```

## 这一阶段通过标准

你能用自己的话说清楚下面这段，就算通过：

> robosuite 的 `OSC_POSE` 不是直接控制关节角，而是先根据 action 更新末端目标位置。控制器计算目标末端位置和当前末端位置的误差，用 `kp` 和 `kd` 得到末端期望力，再通过 Jacobian 转置把末端力转换成各个关节需要输出的力矩。

这句话说清楚以后，下一步才进入 `deoxys_control` 的 `osc_impedance.cpp`。
