# 机械臂视觉引导自动喷涂系统

基于 ROS + Gazebo + MoveIt + RGB-D 点云感知 + PCL + tf2 + ros_control 的机械臂自动喷涂项目。

## 项目简介

本项目实现了一套"视觉引导自动喷涂"系统：

1. 在 Gazebo 中仿真一台 6 轴机械臂，末端安装喷枪与 RGB-D 相机
2. 相机实时采集场景点云，通过 PCL 平面检测算法识别车门位置
3. 系统自动生成"从上往下"的蛇形/S 型喷涂轨迹
4. MoveIt 通过 `ros_control`（`JointTrajectoryController`）将规划轨迹发送到 Gazebo，**机械臂在 Gazebo 中真实运动**，而不只是 RViz 动画

---

## 系统组成

| 组件 | 说明 |
|------|------|
| 6 轴机械臂 | 含 URDF / Xacro 描述，joint1~joint6 |
| 喷枪 | `spray_gun_link`，固连于末端 link6 |
| 喷涂 TCP | `spray_tcp_link`，MoveIt 末端执行器坐标系 |
| RGB-D 相机 | `rgbd_camera_link`，Gazebo OpenNI Kinect 插件，发布 `/camera/depth/points` |
| 车门模型 | `door_link`，固定在场景中作为喷涂目标 |
| Gazebo | 物理仿真，`gazebo_ros_control` 插件驱动真实关节运动 |
| MoveIt | 运动规划，使用 `JointTrajectoryController` 真实执行 |
| 感知节点 | `door_cloud_listener`：点云 → 平面检测 → 目标位姿 |
| 喷涂执行节点 | `door_spray_executor`：蛇形轨迹规划 + Cartesian 执行 |

---

## 项目结构

```
src/
├── arm_description/                  # 机械臂描述包
│   ├── urdf/
│   │   ├── arm_description.xacro    # 主 URDF（含 ros_control 插件）
│   │   └── door.xacro               # 车门模型
│   ├── config/
│   │   └── arm_controllers.yaml     # ros_control 控制器参数（/arm 命名空间）
│   └── launch/
│       ├── gazebo.launch            # 仅 Gazebo，独立使用
│       └── display.launch           # RViz 可视化
│
├── arm_description_moveit_config/    # MoveIt 配置包
│   ├── config/
│   │   ├── arm_description.srdf         # 机械臂规划组定义
│   │   ├── kinematics.yaml              # KDL 运动学求解器
│   │   ├── ompl_planning.yaml           # OMPL 规划器配置
│   │   ├── joint_limits.yaml            # 关节限位
│   │   ├── simple_moveit_controllers.yaml  # ★ 真实控制器映射（指向 /arm/arm_controller）
│   │   └── fake_controllers.yaml        # fake 模式控制器（仅 RViz）
│   └── launch/
│       ├── gazebo_spray.launch      # ★ Gazebo + MoveIt 真实执行一体化
│       ├── spray_demo.launch        # ★ 完整 Demo（含感知 + 喷涂节点）
│       ├── demo.launch              # RViz fake execution（原有）
│       ├── demo_gazebo.launch       # MoveIt 官方 Gazebo 模板
│       └── move_group.launch        # move_group 核心节点
│
└── arm_perception/                   # 感知与执行包
    └── src/
        ├── door_cloud_listener.cpp   # 点云感知节点
        ├── door_moveit_executor.cpp  # 简单单点规划执行节点
        └── door_spray_executor.cpp   # ★ 自动蛇形喷涂执行节点
```

---

## 环境依赖

| 依赖 | 版本 |
|------|------|
| Ubuntu | 20.04 (Focal) |
| ROS | Noetic |
| Gazebo | 11.x（随 ROS Noetic 安装） |
| MoveIt | 1.x for Noetic |
| PCL | 1.10+ |
| tf2 / tf2_ros | Noetic 默认版本 |
| ros_control | Noetic 默认版本 |
| robot_state_publisher | Noetic 默认版本 |
| joint_state_controller | ros-noetic-joint-state-controller |
| position_controllers | ros-noetic-position-controllers |

### 安装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-moveit \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-joint-state-controller \
  ros-noetic-position-controllers \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  ros-noetic-pcl-ros \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs
```

---

## 编译方法

```bash
# 1. 创建工作空间（如果尚未创建）
mkdir -p ~/arm_ws/src
cd ~/arm_ws

# 2. 将项目代码放入 src/（或 clone 到 src/）
# cp -r /path/to/Arm/src/* ~/arm_ws/src/

# 3. 安装依赖
cd ~/arm_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. 编译
catkin_make

# 5. source 工作空间
source ~/arm_ws/devel/setup.bash
```

---

## 运行方法

### 方式一：一键启动完整 Demo（推荐）

```bash
source ~/arm_ws/devel/setup.bash
roslaunch arm_description_moveit_config spray_demo.launch
```

这条命令会同时启动：
- Gazebo（含机械臂和车门）
- MoveIt move_group（真实控制器模式）
- RViz
- 点云感知节点 `door_cloud_listener`
- 自动喷涂节点 `door_spray_executor`

启动后，`door_cloud_listener` 开始处理相机点云并检测车门。检测到车门后，
`door_spray_executor` 自动规划并执行从上往下的蛇形喷涂轨迹，**Gazebo 中机械臂会真实运动**。

---

### 方式二：分步启动（方便调试）

#### 终端 1：Gazebo + MoveIt（真实执行）

```bash
source ~/arm_ws/devel/setup.bash
roslaunch arm_description_moveit_config gazebo_spray.launch
```

#### 终端 2：点云感知节点

```bash
source ~/arm_ws/devel/setup.bash
rosrun arm_perception door_cloud_listener
```

#### 终端 3：自动喷涂执行节点

```bash
source ~/arm_ws/devel/setup.bash
rosrun arm_perception door_spray_executor
```

**等 `door_cloud_listener` 开始输出 `approach_pose_base` 信息后，再启动 `door_spray_executor`。**

---

### 方式三：仅 RViz 仿真（fake execution，不需要 Gazebo）

```bash
source ~/arm_ws/devel/setup.bash
roslaunch arm_description_moveit_config demo.launch
```

此模式下机械臂只在 RViz 中动，Gazebo 不参与。用于快速验证 MoveIt 规划是否正常。

---

## 关键话题说明

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | Gazebo RGB-D 相机发布的点云 |
| `/door_target_pose` | `geometry_msgs/PoseStamped` | 车门中心位置（相机坐标系） |
| `/door_target_pose_base` | `geometry_msgs/PoseStamped` | 车门中心位置（base_link 坐标系） |
| `/door_approach_pose_base` | `geometry_msgs/PoseStamped` | 喷涂接近位姿（base_link 坐标系，含法向量方向） |
| `/arm/joint_states` | `sensor_msgs/JointState` | ros_control 发布的真实关节状态 |
| `/arm/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectoryAction` | MoveIt 发送轨迹给 Gazebo 的 Action |
| `/move_group/status` | `actionlib_msgs/GoalStatusArray` | MoveIt 规划执行状态 |

---

## 自动喷涂流程说明

```
RGB-D 相机 ──点云──▶ door_cloud_listener
                          │
                    PCL 平面检测 (RANSAC)
                          │
                    计算车门中心 + 法向量
                          │
                    生成接近位姿（沿法向量退后 0.15m）
                          │
                    发布 /door_approach_pose_base
                          │
                          ▼
                   door_spray_executor
                          │
              (1) MoveIt 规划到喷涂中心
                          │
              (2) MoveIt 规划到蛇形起始点（左上）
                          │
              (3) computeCartesianPath()
                  生成从上往下的蛇形轨迹点
                    z 轴逐步下降（主推进方向）
                    y 轴左右短距往返（覆盖宽度）
                          │
              (4) MoveIt 执行 Cartesian 轨迹
                          │
                          ▼
              /arm/arm_controller/follow_joint_trajectory
                          │
                          ▼
                    Gazebo 机械臂真实运动 ✓
```

### 为什么用 Cartesian Path 做喷涂段？

喷涂要求末端按照精确的空间轨迹运动（蛇形路径），每一段都要到达特定位置，
且工具姿态保持固定（垂直车门面）。`computeCartesianPath()` 可以：
- 保证末端沿直线段运动，适合连续喷涂
- 以极小步长（如 1cm）逐步插值，保证覆盖均匀
- 保持固定的工具姿态（喷枪朝向由法向量决定）

### 为什么从上往下蛇形？

1. 喷涂重力方向：漆液向下流，从上往下喷可以避免上方已喷区域被下方轨迹产生的飞溅污染
2. 每行做短距离横向往返（而不是大范围横移），减少单次行程中位姿变化，提高路径可执行性
3. 从上往下推进，与真实喷涂工艺一致

### 如何让 Gazebo 真实执行？

关键在于 MoveIt 控制器管理器的配置：

| 模式 | 控制器 | Gazebo 是否动 |
|------|--------|-------------|
| `fake` (demo.launch 默认) | `moveit_fake_controller_manager` | ❌ 不动 |
| `simple` (gazebo_spray.launch) | `moveit_simple_controller_manager` → `/arm/arm_controller` | ✅ 真实运动 |

在 `simple` 模式下，MoveIt 将规划好的轨迹通过 `FollowJointTrajectory` Action 发送给
Gazebo 中运行的 `JointTrajectoryController`，后者驱动关节按轨迹运动。

`simple_moveit_controllers.yaml` 中配置了控制器地址：
```yaml
controller_list:
  - name: /arm/arm_controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
```

---

## 常见问题与排查

### Q1：RViz 里机械臂会动，但 Gazebo 里不动

**原因**：使用了 fake controller 模式（`demo.launch` 默认）。

**解决**：改用 `gazebo_spray.launch` 或 `spray_demo.launch`，它们使用 `simple` 控制器管理器，
直接向 Gazebo 中的 `arm_controller` 发送真实轨迹。

---

### Q2：`door_spray_executor` 一直显示 "Waiting for /door_approach_pose_base"

**原因**：`door_cloud_listener` 没有运行，或者相机话题没有数据。

**排查步骤**：
```bash
# 检查感知节点是否在运行
rosnode list | grep door_cloud_listener

# 检查相机话题是否有数据
rostopic hz /camera/depth/points

# 检查感知节点是否在发布目标位姿
rostopic echo -n 1 /door_approach_pose_base
```

---

### Q3：MoveIt 规划一直失败

**可能原因及对策**：

1. **目标不可达**：车门位置可能超出机械臂工作空间。尝试调整 Gazebo 中车门位置。
2. **起始状态碰撞**：机械臂当前位置与场景发生碰撞。尝试在 RViz 中手动将机械臂移到安全位置。
3. **规划时间不足**：增大 `planning_time` 参数（默认 8 秒）。

---

### Q4：Cartesian path fraction 太低（< 0.85）

**原因**：蛇形路径中存在不可达点，或关节限位阻止连续运动。

**对策**：
- 减小 `spray_width` 和 `spray_height`（缩小喷涂范围）
- 减小 `min_cartesian_fraction` 阈值（如改为 0.70）
- 增大 `eef_step`（如改为 0.02，牺牲精度换可行性）

```bash
rosrun arm_perception door_spray_executor \
  _spray_width:=0.05 \
  _spray_height:=0.10 \
  _min_cartesian_fraction:=0.70
```

---

### Q5：姿态约束过严导致失败

`door_spray_executor` 在 `moveToPose` 阶段使用完整的 6 DOF 位姿目标（含方向）。
如果方向约束过严，可以：

1. 增大 `orientation_tolerance`（默认 0.10 rad）
2. 或者在 `door_spray_executor.cpp` 中的 `moveToPose` 将 `position_only_` 改为 `true`

---

### Q6：控制器未连接 / Action 服务器超时

**排查**：
```bash
# 检查控制器是否在运行
rostopic list | grep arm_controller

# 检查 Action 服务器
rostopic echo /arm/arm_controller/follow_joint_trajectory/status

# 检查控制器状态
rosservice call /arm/controller_manager/list_controllers
```

如果控制器没有启动，手动重新 spawn：
```bash
rosrun controller_manager spawner joint_state_controller arm_controller __ns:=/arm
```

---

### Q7：喷涂轨迹过大导致失败

将 `spray_width` 和 `spray_height` 改小，例如：
```bash
rosrun arm_perception door_spray_executor \
  _spray_width:=0.04 \
  _spray_height:=0.08 \
  _line_spacing:=0.02
```

---

### Q8：如何关闭 RViz 的 Loop Animation

RViz 中 MotionPlanning 面板的 `Loop Animation` 选项控制轨迹是否循环播放。

1. 在 RViz 左侧 Displays 面板找到 **MotionPlanning**
2. 展开 → 找到 **Trajectory Visualization** → **Loop Animation**
3. 取消勾选
4. 保存 RViz 配置：File → Save Config As → 覆盖 `moveit.rviz`

---

## 喷涂参数参考

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `spray_width` | 0.06 m | 每行横向摆动距离（左右各一半） |
| `spray_height` | 0.12 m | 喷涂区域总高度（从上往下推进） |
| `line_spacing` | 0.03 m | 行间距（垂直推进步长） |
| `eef_step` | 0.01 m | Cartesian 路径插值步长 |
| `jump_threshold` | 0.0 | 关节跳变阈值（0=禁用检测） |
| `min_cartesian_fraction` | 0.85 | 最低 Cartesian 覆盖率 |
| `planning_time` | 8.0 s | MoveIt 最大规划时间 |
| `max_attempts` | 10 | MoveIt 最大规划次数 |
| `position_tolerance` | 0.01 m | 目标位置容差 |
| `orientation_tolerance` | 0.10 rad | 目标姿态容差 |
