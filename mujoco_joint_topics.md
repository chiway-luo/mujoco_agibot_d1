# MuJoCo 关节话题排查表

## 关节相关话题链路

| 话题 | 类型 | 当前角色 | 运行逻辑 | 排查重点 |
| --- | --- | --- | --- | --- |
| `/legs_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 发布者通常是 `/quadruped_controller_node`，订阅者是 `/legs_controller` | CHAMP 根据机身姿态、步态和逆解结果发布 12 个关节的目标角度；`legs_controller` 接收后写入 ros2_control 的 position command interface | 这是 ROS 到 MuJoCo 的关节命令入口。不要按数组位置猜关节，必须看 `joint_names` |
| `/legs_controller/state` | `control_msgs/msg/JointTrajectoryControllerState` | 发布者是 `/legs_controller` | 关节轨迹控制器输出当前控制状态，包含 `desired`、`actual`、`error` | 用它比较控制器想要的角度和 MuJoCo 实际反馈角度 |
| `/legs_controller/controller_state` | `control_msgs/msg/JointTrajectoryControllerState` | 发布者是 `/legs_controller` | 与控制器状态相关的调试输出，通常可作为 `/legs_controller/state` 的补充 | 如果两个状态话题都存在，优先看内容更稳定的那个 |
| `/get_joint_states` | `sensor_msgs/msg/JointState` | 发布者是 `/joint_state_broadcaster`，订阅者包含 `/robot_state_publisher` 和 `/state_estimation_node` | MuJoCo 通过 ros2_control 反馈实际关节位置、速度；launch 中把 `/joint_states` remap 到 `/get_joint_states` | 反馈顺序不保证等于控制器顺序，消费者必须按 `name` 字段映射 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 当前预期没有独立发布者 | 本工程 launch 中将 `/joint_states` remap 到 `/get_joint_states` | 如果这里为空，不一定是错误；重点检查 `/get_joint_states` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 通常由键盘、导航或速度平滑节点发布；CHAMP 订阅 | 速度指令会改变 CHAMP 的步态输出，最终影响 `/legs_controller/joint_trajectory` | 静态站立排查时应确认没有持续速度命令 |
| `/body_pose` | `geometry_msgs/msg/Pose` | CHAMP 订阅 | 外部机身姿态命令会改变逆解结果，最终影响 `/legs_controller/joint_trajectory` | 静态站立排查时应确认没有异常机身姿态输入 |

## 关节命令方向

当前 launch 中 CHAMP 的输出目标话题是 `/legs_controller/joint_trajectory`，控制器通过 remap 接收这个话题。关节控制链路为：

```text
/cmd_vel 或 /body_pose
        -> /quadruped_controller_node
        -> /legs_controller/joint_trajectory
        -> /legs_controller
        -> ros2_control position command interface
        -> mujoco_ros2_control
        -> MuJoCo joint qpos
        -> /joint_state_broadcaster
        -> /get_joint_states
```

当前站立角度基准：

| 关节类型 | 角度 rad | 角度 deg |
| --- | ---: | ---: |
| ABAD | `0.0` | `0.0` |
| HIP | `1.0475260019302368` | `60.02` |
| KNEE | `-1.9930626153945923` | `-114.19` |

## 站立角度测试命令

如果 CHAMP 正在持续发布 `/legs_controller/joint_trajectory`，这条手动命令可能会被 CHAMP 后续消息覆盖。排查单次角度时，建议先停掉 CHAMP 控制节点，或者用 `--rate` 持续发布。

```bash
ros2 topic pub --once /legs_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['FL_ABAD_JOINT', 'FL_HIP_JOINT', 'FL_KNEE_JOINT', 'FR_ABAD_JOINT', 'FR_HIP_JOINT', 'FR_KNEE_JOINT', 'RL_ABAD_JOINT', 'RL_HIP_JOINT', 'RL_KNEE_JOINT', 'RR_ABAD_JOINT', 'RR_HIP_JOINT', 'RR_KNEE_JOINT'], points: [{positions: [0.0, 1.0475260019302368, -1.9930626153945923, 0.0, 1.0475260019302368, -1.9930626153945923, 0.0, 1.0475260019302368, -1.9930626153945923, 0.0, 1.0475260019302368, -1.9930626153945923], time_from_start: {sec: 1, nanosec: 0}}]}"
```

持续发布版本：

```bash
ros2 topic pub --rate 20 /legs_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['FL_ABAD_JOINT', 'FL_HIP_JOINT', 'FL_KNEE_JOINT', 'FR_ABAD_JOINT', 'FR_HIP_JOINT', 'FR_KNEE_JOINT', 'RL_ABAD_JOINT', 'RL_HIP_JOINT', 'RL_KNEE_JOINT', 'RR_ABAD_JOINT', 'RR_HIP_JOINT', 'RR_KNEE_JOINT'], points: [{positions: [0.0, 1.0475260019302368, -1.9930626153945923, 0.0, 1.0475260019302368, -1.9930626153945923, 0.0, 1.0475260019302368, -1.9930626153945923, 0.0, 1.0475260019302368, -1.9930626153945923], time_from_start: {sec: 1, nanosec: 0}}]}"
```

## 单关节方向测试命令

下面这条命令只把 `FL_HIP_JOINT` 从站立基准的 `1.047526` 改到 `1.247526`，其余关节保持站立角度。用它可以观察正方向是否符合预期：

```bash
ros2 topic pub --once /legs_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['FL_ABAD_JOINT', 'FL_HIP_JOINT', 'FL_KNEE_JOINT', 'FR_ABAD_JOINT', 'FR_HIP_JOINT', 'FR_KNEE_JOINT', 'RL_ABAD_JOINT', 'RL_HIP_JOINT', 'RL_KNEE_JOINT', 'RR_ABAD_JOINT', 'RR_HIP_JOINT', 'RR_KNEE_JOINT'], points: [{positions: [0.0, 1.2475260019302368, -1.9930626153945923, 0.0, 1.0475260019302368, -1.9930626153945923, 0.0, 1.0475260019302368, -1.9930626153945923, 0.0, 1.0475260019302368, -1.9930626153945923], time_from_start: {sec: 1, nanosec: 0}}]}"
```
