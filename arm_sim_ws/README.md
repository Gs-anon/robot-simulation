# arm_description — 状态机式机械臂 ROS2 仿真工作空间

用一份 **YAML 状态机配置** 描述机械臂，自动生成 URDF，并在 **RViz（运动学）** 或
**Gazebo（物理）** 中仿真。改机械臂的 **轴序、偏置、连杆参数** 都只需改这一个 YAML 文件。

```
arm_sim_ws/
└── src/
    └── arm_description/
        ├── package.xml
        ├── CMakeLists.txt
        ├── setup.py / setup.cfg
        ├── resource/arm_description
        ├── arm_description/
        │   ├── __init__.py
        │   └── urdf_generator.py          # 核心：YAML -> URDF（含 DH 换算、惯量、Gazebo 接口）
        ├── config/
        │   ├── arm_config.yaml            # ★ 你要改的状态机参数就在这里
        │   └── controllers.yaml           # Gazebo 控制器（关节名需与 config 一致）
        ├── launch/
        │   ├── display.launch.py          # RViz 运动学仿真（开箱即用）
        │   └── gazebo.launch.py           # Gazebo 物理仿真（可选）
        └── rviz/
            └── arm.rviz                   # RViz 界面预设
```

---

## 一、状态机怎么理解

`config/arm_config.yaml` 里的 `states` 是一个**列表**，每一项 = 一个「状态」，
也就是 **一个关节 + 它带动的那根连杆**。列表从上到下的顺序，就是 **轴序**：

```
base_link ── joint_1 ── joint_2 ── joint_3 ── ... ── tool0
```

- **改轴序** → 直接调整 `states` 列表里各块的顺序。
- **加/减关节** → 增删一个状态块。
- **改偏置/连杆参数** → 改对应状态块里的 `dh`（或 `origin`）和 `link`。

---

## 二、改参数：三种约定任选其一（顶层 `convention`）

| 约定 | 说明 | 适合场景 |
|------|------|----------|
| `dh_std` | 标准 DH：`a / alpha / d / theta`，自动换算 URDF | 有现成 DH 表 |
| `dh_mod` | 改进 DH（Craig 约定） | Craig 教材风格 DH 表 |
| `urdf` | 直接写 `axis`（转轴）+ `origin.xyz/rpy`（偏置） | 想直接控制关节位姿 |

DH 参数含义（标准 DH）：

| 参数 | 含义 | 单位 |
|------|------|------|
| `a` | 连杆长度：沿 X_{i-1}，Z_{i-1} 到 Z_i 的距离 | 米 |
| `alpha` | 连杆扭转：绕 X_{i-1}，Z_{i-1} 转到 Z_i | 弧度 |
| `d` | 关节偏置：沿 Z_i，X_{i-1} 到 X_i 的距离 | 米 |
| `theta` | 关节角：绕 Z_{i-1}，X_{i-1} 转到 X_i（旋转关节的变量，填初值） | 弧度 |

> 注意：`link.geometry` 只影响 **显示/碰撞外形**；真正的运动学尺寸由 `a / d / alpha` 决定。
> 想精确校核，可对照各状态换算出的 URDF `origin`（标准 DH：`origin=(a,0,d)`，`rpy=(alpha,0,0)`）。

---

## 三、安装与运行（Ubuntu + ROS2）

以 Humble/Iron/Jazzy 任一发行版为例，把 `$ROS_DISTRO` 换成你的发行版名。

### 1. 安装基础依赖（RViz 运动学仿真必需）

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher-gui \
  ros-$ROS_DISTRO-rviz2 \
  python3-yaml
```

### 2. 编译

```bash
cd arm_sim_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. 运行 RViz 仿真

```bash
ros2 launch arm_description display.launch.py
```

弹出 RViz 后，`joint_state_publisher_gui` 窗口里拖动各关节滑杆，机械臂就会动。

### 4.（可选）导出 URDF 文件检查

```bash
ros2 run arm_description generate_urdf \
  $(ros2 pkg prefix arm_description)/share/arm_description/config/arm_config.yaml \
  -o /tmp/arm.urdf
cat /tmp/arm.urdf
```

### 5.（可选）改完参数后的本地自检

不启动 ROS 也能先确认改过的 YAML 能生成合法、连通的 URDF（需要 Python + PyYAML）：

```bash
python3 validate_generator.py
```

输出各项全为 `PASS` 即为正常；若报错会打印具体问题（如轴未归一化、连杆不连通）。

---

## 四、Gazebo 物理仿真（可选）

### 1. 安装依赖

```bash
sudo apt install -y \
  ros-$ROS_DISTRO-gazebo-ros \
  ros-$ROS_DISTRO-gazebo-ros2-control \
  ros-$ROS_DISTRO-ros2-control \
  ros-$ROS_DISTRO-ros2-controllers \
  ros-$ROS_DISTRO-joint-state-broadcaster \
  ros-$ROS_DISTRO-forward-command-controller
```

> 注意：Jazzy 及更新版本用 `gz-sim`，包名与启动方式略有不同；本工程按经典
> `gazebo_ros` 写法提供，若用 gz-sim 可把 launch 里的 `gazebo.launch.py` 换成
> `ros_gz_sim` 的启动。

### 2. 运行

先把 `arm_config.yaml` 里的 `enable_gazebo` 改为 `true`（launch 里也会强制打开），然后：

```bash
ros2 launch arm_description gazebo.launch.py
```

### 3. 给关节发位置目标

```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.5, -0.3, 0.2, 0.0, 0.0, 0.0]}"
```

> 关节顺序 = `controllers.yaml` 里 `arm_controller.joints` 的顺序，请与 `arm_config.yaml`
> 的关节名保持一致。

---

## 五、常见问题

- **RViz 里看不到机械臂**：确认 Fixed Frame 是 `base_link`，且左下角 Displays 里
  `RobotModel` 已勾选；`Description Topic` 应为 `/robot_description`。
- **改完 YAML 不生效**：用 `--symlink-install` 编译时 YAML 是软链接，直接改即可；
  否则重新 `colcon build` 一次。
- **import 报错找不到 `arm_description`**：确认已 `source install/setup.bash`。
- **滑杆不动**：确认 `joint_state_publisher_gui` 窗口处于最前，且关节在限位范围内。

---

## 六、快速改造示例

**例 1：把 6 轴改成 4 轴** — 删掉 `joint_5`、`joint_6` 两个状态块，同时删
`controllers.yaml` 里对应的关节名。

**例 2：把关节 2 从「绕 Z 转」改成「绕 Y 转」**（URDF 约定下）— 该状态块的
`axis` 改成 `[0, 1, 0]`。

**例 3：给关节加偏置**（URDF 约定下）— 改该状态块的 `origin.xyz`；DH 约定下则改 `d`。

**例 4：换连杆长度**（DH 约定下）— 改该状态块的 `a`，并把 `link.geometry.length`
改成接近的显示长度。
