使用champ开源算法,实现智元机器狗d1的mujoco仿真和导航
> 测试阶段,请勿使用
===
## 配置流程
1. 需要从github上克隆mujoco_ros2_control仓库,并放置在src目录下
```bash
cd src/
git clone https://github.com/dfki-ric/mujoco_ros2_control
```
2. 为了降低编译时间,删除examples目录
```bash
cd mujoco_ros2_control
rm -rf examples
```

## 使用方式
mujoco文档见 [mujoco](src/mujoco_ros2_control/mujoco_ros2_control/README.md)
- 克隆本仓库
```bash

```
- 安装依赖
```

```
- mujoco仿真环境启动
```
ros2 launch sim_ign_dog d1_mujoco_sim_dog.launch.py
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

xacro2mjcf -> mujoco_node
mujoco_node start -> imu_broadcaster, base_pose_broadcaster, rviz_node
controllers_delay -> jsb_spawner
jsb_spawner exit -> legs_spawner
legs_spawner exit -> CHAMP

## 话题说明
- /get_joint_states 关节状态 mujoco -> ros2
- /joint_command 关节命令 ros2 -> mujoco



### 解决方案
参考 [ssh端口转发](https://github.com/chiway-luo/ssh-x11-forwarding-guide.git) , 将仿真环境部署在远程服务器上,通过ssh连接进行仿真环境的使用