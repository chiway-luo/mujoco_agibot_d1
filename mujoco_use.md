# URDF 配置（用于 xacro2mjcf 脚本）

若要将此包用于现有机器人描述（URDF 或 Xacro），请创建一个 **Xacro 包装文件**，将以下内容合并在一起：

- 你现有的机器人描述；
- MuJoCo 配置；
- ROS 2 Control 配置。

可参考示例中的 `urdf` 目录（[franka](https://github.com/dfki-ric/mujoco_ros2_control/blob/main/examples/franka_mujoco/urdf/franka.urdf.xacro)、[unitree](https://github.com/dfki-ric/mujoco_ros2_control/blob/main/examples/unitree_h1_mujoco/urdf/unitree_h1.urdf.xacro)）。

---

## MuJoCo 专用元素

下面的片段展示了如何将 MuJoCo 配置元素集成到机器人描述中：

```xml
<mujoco>
    <!-- 编译器选项：
         https://mujoco.readthedocs.io/en/stable/XMLreference.html#compiler -->
    <compiler
        meshdir="/tmp/mujoco/meshes"
        discardvisual="true"
        autolimits="false"
        balanceinertia="true"/>

    <!-- 全局仿真选项：
         https://mujoco.readthedocs.io/en/stable/XMLreference.html#option -->
    <option
        integrator="implicitfast"
        gravity="0 0 -9.81"
        impratio="10"
        cone="elliptic"
        solver="Newton">
        <flag multiccd="enable"/>
    </option>

    <!-- 向某个 MJCF body 或其任意子元素添加元素/标签 -->
    <reference name="${prefix}left_inner_finger">
        <!-- 添加按 body 与按 joint 的配置 -->
        <body gravcomp="1"/>            <!-- 启用重力补偿 -->
        <joint damping="10"/>           <!-- 为所有子关节添加阻尼 -->

        <!-- 修改给定名称的子 geom -->
        <geom
            name="geom1"
            friction="0.7"
            mass="0"
            priority="1"
            solimp="0.95 0.99 0.001"
            solref="0.004 1"/>
    </reference>

    <!-- 定义 RGB-D 相机：
         https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-camera -->
    <reference name="camera_link">
        <camera
            name="camera"
            mode="fixed"
            fovy="45"
            quat="0.5 0.5 -0.5 -0.5"/>
    </reference>

    <!-- 相对于世界坐标系的相机位姿传感器 -->
    <sensor>
        <!-- 位置传感器：
             https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-framepos -->
        <framepos
            name="camera_link_pose"
            objtype="xbody"
            objname="camera_link"
            reftype="body"
            refname="world"/>

        <!-- 姿态传感器：
             https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-framequat -->
        <framequat
            name="camera_link_quat"
            objtype="xbody"
            objname="camera_link"
            reftype="body"
            refname="world"/>
    </sensor>

    <!-- 执行器定义：
         https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator -->
    <actuator>
        <position
            name="pos_finger_joint1"
            joint="${arm_id}_finger_joint1"
            kp="1000"
            forcelimited="true"
            forcerange="-120 120"
            ctrllimited="true"
            ctrlrange="0 0.04"
            user="1"/>
    </actuator>
</mujoco>
```

## ROS 2 Control 硬件示例
下面示例展示了如何声明一个同时使用 PID 与力矩控制的 ROS 2 Control 系统：
```xml
<ros2_control name="${prefix}${name}" type="system">
    <hardware>
        <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    </hardware>

    <!-- 使用 位置 + 速度 + 加速度 PID 控制的关节 -->
    <joint name="joint1">
        <command_interface name="position"/>
        <command_interface name="velocity"/>
        <command_interface name="acceleration"/>

        <param name="kp">1000.0</param>
        <param name="ki">0.0</param>
        <param name="kd">0.01</param>

        <!-- 仅在使用 位置 + 速度 控制时需要 -->
        <param name="kvff">0.01</param>

        <!-- 在使用 位置 + 速度 + 加速度 控制时需要 -->
        <param name="kaff">0.01</param>

        <state_interface name="position"/>
        <state_interface name="velocity"/>
    </joint>

    <!-- 使用力矩（effort）控制的关节 -->
    <joint name="joint2">
        <command_interface name="effort"/>

        <state_interface name="position">
            <param name="initial_value">1.0</param>
        </state_interface>

        <state_interface name="velocity">
            <param name="initial_value">0.0</param>
        </state_interface>
    </joint>
</ros2_control>
```

## ROS 2 Control 传感器接口

传感器在 `<ros2_control>` 块中声明，并会通过 `<param>` 自动与 MuJoCo 传感器匹配。该 `<param>` 用于指定传感器所附着的 MuJoCo 对象（site、xbody、geom 等）。当前支持三类传感器：**IMU**、**力/力矩（Force/Torque）**、**位姿（Pose）**。

传感器类型会根据你声明的 state interface 名称自动判定。

### IMU 传感器

从 MuJoCo site 读取姿态（framequat）、角速度（gyro）和线加速度（accelerometer）。可配合标准 [imu_sensor_broadcaster](https://control.ros.org/rolling/doc/ros2_controllers/imu_sensor_broadcaster/doc/userdoc.html) 使用。

```xml
<ros2_control name="MySystem" type="system">
    <hardware>
        <plugin>mujoco_ros2_control/MujocoSystem</plugin>
    </hardware>

    <!-- ... joints ... -->

    <sensor name="imu_in_pelvis">
        <param name="site">imu_in_pelvis</param>
        <state_interface name="orientation.x"/>
        <state_interface name="orientation.y"/>
        <state_interface name="orientation.z"/>
        <state_interface name="orientation.w"/>
        <state_interface name="angular_velocity.x"/>
        <state_interface name="angular_velocity.y"/>
        <state_interface name="angular_velocity.z"/>
        <state_interface name="linear_acceleration.x"/>
        <state_interface name="linear_acceleration.y"/>
        <state_interface name="linear_acceleration.z"/>
    </sensor>
</ros2_control>
```

对应的 MuJoCo 传感器必须在 `<mujoco>` 段中定义：
```xml
<mujoco>
    <!-- 在 IMU 所在的 body 上创建 site -->
    <reference name="pelvis">
        <site name="imu_in_pelvis" size="0.01" pos="0 0 0"/>
    </reference>

    <sensor>
        <gyro name="imu_in_pelvis-angular-velocity" site="imu_in_pelvis" noise="5e-4" cutoff="34.9"/>
        <accelerometer name="imu_in_pelvis-linear-acceleration" site="imu_in_pelvis" noise="1e-2" cutoff="157"/>
        <framequat name="imu_in_pelvis-orientation" objtype="site" objname="imu_in_pelvis"/>
    </sensor>
</mujoco>
```

控制器配置（`controllers.yaml`）：
```yaml
controller_manager:
  ros__parameters:
    imu_broadcaster:
      type: imu_sensor_broadcaster/IMUSensorBroadcaster

imu_broadcaster:
  ros__parameters:
    sensor_name: "imu_in_pelvis"
    frame_id: "imu_in_pelvis"
```

### 力/力矩传感器

从 MuJoCo site 读取力与力矩。可配合标准 [force_torque_sensor_broadcaster](https://control.ros.org/rolling/doc/ros2_controllers/force_torque_sensor_broadcaster/doc/userdoc.html) 使用。

```xml
<sensor name="ft_sensor">
    <param name="site">ft_site</param>
    <state_interface name="force.x"/>
    <state_interface name="force.y"/>
    <state_interface name="force.z"/>
    <state_interface name="torque.x"/>
    <state_interface name="torque.y"/>
    <state_interface name="torque.z"/>
</sensor>
```

对应的 MuJoCo 传感器：
```xml
<mujoco>
    <reference name="link7">
        <site name="ft_site" pos="0 0 0.107" quat="0.92388 0 0 -0.382683"/>
    </reference>
    <sensor>
        <force name="ft_site_force" site="ft_site"/>
        <torque name="ft_site_torque" site="ft_site"/>
    </sensor>
</mujoco>
```

控制器配置：
```yaml
controller_manager:
  ros__parameters:
    ft_sensor_broadcaster:
      type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster

ft_sensor_broadcaster:
  ros__parameters:
    sensor_name: "ft_sensor"
    frame_id: "link7"
```

### 位姿传感器

读取 MuJoCo body 的位置（framepos）和姿态（framequat）。可配合标准 [pose_broadcaster](https://control.ros.org/rolling/doc/ros2_controllers/pose_broadcaster/doc/userdoc.html) 使用。对于浮动基座机器人，这有助于获取 base link 位姿。

```xml
<sensor name="pelvis_pose">
    <param name="body">pelvis</param>
    <state_interface name="position.x"/>
    <state_interface name="position.y"/>
    <state_interface name="position.z"/>
    <state_interface name="orientation.x"/>
    <state_interface name="orientation.y"/>
    <state_interface name="orientation.z"/>
    <state_interface name="orientation.w"/>
</sensor>
```

对应的 MuJoCo 传感器：
```xml
<mujoco>
    <sensor>
        <framepos name="pelvis_pose" objtype="body" objname="pelvis" reftype="body" refname="world"/>
        <framequat name="pelvis-orientation" objtype="body" objname="pelvis" reftype="body" refname="world"/>
    </sensor>
</mujoco>
```

控制器配置：
```yaml
controller_manager:
  ros__parameters:
    pelvis_pose_broadcaster:
      type: pose_broadcaster/PoseBroadcaster

pelvis_pose_broadcaster:
  ros__parameters:
    pose_name: "pelvis_pose"
    frame_id: "world"
    tf:
      enable: true
      child_frame_id: "pelvis"
```

### 传感器匹配

`<sensor>` 块中的 `<param>` 会告诉插件应查找哪个 MuJoCo 对象。支持的键有：`site`、`body`、`geom`、`camera`、`light`、`frame`。如果未提供 param，则使用传感器 `name` 作为匹配键。

例如，`<param name="site">imu_in_pelvis</param>` 会匹配所有对象名为 `imu_in_pelvis` 的 MuJoCo 传感器。

## 旁路传感器（相机与激光雷达）

这些传感器**不是** ros2_control 接口。每个实例都会以独立 ROS 节点运行，并拥有自己的发布器和工作线程，因此不会出现在 `<ros2_control>` 块中，也不会通过控制器广播。

### RGB-D 相机

在 `<reference>` body 下声明的每个 MuJoCo `<camera>` 都会自动生成一个以该相机命名的深度相机节点。相机所在的 site/body 决定位姿；具体发布哪些数据由每个相机对应的 ROS 参数决定。

```xml
<reference name="camera_link">
    <camera name="camera" mode="fixed" fovy="45" quat="0.5 0.5 -0.5 -0.5"/>
</reference>
```

在参数 YAML 中，每个相机的参数以相机名作为键：
```yaml
camera:
  ros__parameters:
    width: 640
    height: 480
    frequency: 30.0
    color_image: true
    depth_image: true
    point_cloud: true
```

话题（仅为你启用的输出创建）：

| 输出 | 话题 | 类型 |
|---|---|---|
| 彩色图 | `/<camera_name>/color/image_raw`（+ `camera_info`） | `sensor_msgs/Image` |
| 深度图 | `/<camera_name>/depth/image_rect_raw`（+ `camera_info`） | `sensor_msgs/Image` |
| 点云 | `/<camera_name>/depth/points` | `sensor_msgs/PointCloud2` |

完整参数说明：[`mujoco_rgbd_parameters.yaml`](src/mujoco_rgbd_parameters.yaml)。

### GL 深度缓冲激光雷达

使用 GPU 渲染的激光雷达会附着到名称以配置前缀开头（默认：`lidar_`）的任意 MuJoCo site 上。发布消息中的 `frame_id` 为其父 body 名称；site 的局部位姿定义激光雷达坐标系。

仅当某个 lidar site 在 MJCF 中存在，且其按 site 配置的 ROS 参数设置了 `enabled: true` 时，才会实例化该 lidar。`enabled: false`（或缺少参数）的 site 会被跳过，因此你可以将 lidar site 保留在 MJCF 中并在运行时切换启用状态。

```xml
<reference name="head_link">
    <site name="lidar_head" pos="0 0 0.05" quat="1 0 0 0"/>
</reference>
```

在参数 YAML 中，每个 lidar 的参数以 site 名称作为键：
```yaml
lidar_head:
  ros__parameters:
    enabled: true
    output: cloud           # 'scan' or 'cloud'
    frequency: 10.0
    horizontal_min_angle: -1.5708
    horizontal_max_angle:  1.5708
    horizontal_samples: 1800
    vertical_min_angle: -0.2618
    vertical_max_angle:  0.2618
    vertical_samples: 16
    range_min: 0.05
    range_max: 30.0
    render_height: 256
```

话题：

| 输出模式 | 话题 | 类型 |
|---|---|---|
| `scan` | `/<site_name>/scan` | `sensor_msgs/LaserScan` |
| `cloud` | `/<site_name>/points` | `sensor_msgs/PointCloud2` |

完整参数说明：[`mujoco_gl_lidar_parameters.yaml`](src/mujoco_gl_lidar_parameters.yaml)。
