使用champ开源算法,实现智元机器狗d1的mujoco仿真和导航
> 测试阶段,目前只实现了加载并使用cmd_vel控制机器人运动,文档不正确,请勿参考除README.md外的其他文档
===

## 使用方式
<!-- mujoco文档见 [mujoco_use.md](mujoco_use.md) -->
- 克隆本仓库
```bash
git clone https://github.com/chiway-luo/mujoco_agibot_d1.git
```
- 安装依赖
```
sudo apt update
sudo apt install ros-humble-mujoco-ros2-control
ros2 run mujoco_ros2_control robot_description_to_mjcf.sh --install-only  # 安装机器人描述转换脚本
```
- mujoco仿真环境启动
```
ros2 launch sim_mujoco_dog d1_mujoco_sim_dog.launch.py
```


- 控制节点(手动控制机器狗)
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 参考仓库
- [chvmp/champ](https://github.com/chvmp/champ.git)

## 开发参考
- 基坐标系 base_link
- 雷达坐标系 laser_up

robot_description_to_mjcf.sh -> ros2_control_node
ros2_control_node start -> imu_broadcaster, rviz_node
controllers_delay -> jsb_spawner
jsb_spawner exit -> legs_spawner
legs_spawner exit -> CHAMP

## 话题说明
- /get_joint_states 关节状态 mujoco -> ros2
- /legs_controller/joint_trajectory 关节角度命令 CHAMP -> ros2_control -> mujoco
- /odom MuJoCo floating_base_joint 里程计 mujoco -> ros2

当前控制链路仍然是位置控制:
CHAMP 发布 JointTrajectory 关节角度 -> legs_controller 使用 position command interface -> MuJoCo position actuator 跟踪角度。


### 解决方案
参考 [ssh端口转发](https://github.com/chiway-luo/ssh-x11-forwarding-guide.git) , 将仿真环境部署在远程服务器上,通过ssh连接进行仿真环境的使用


[quadruped_controller_node-8] [INFO] [1781946458.651894168] [rclcpp]: Successfully parsed urdf file
[state_estimation_node-9] [INFO] [1781946458.653284829] [rclcpp]: Successfully parsed urdf file

