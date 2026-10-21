# Go2-W Gazebo

Simulation of the Unitree Go2-W using ROS 2 Jazzy Jalisco and Gazebo Harmonic. This package uses the model from [go2w_description](../go2w_description/README.md) and provides a standing posture, four-wheel driving, odometry, joint feedback, IMUs, LiDAR, a front RGB camera, and Unitree LowState telemetry.

## 🚀 Usage

### 🦿 Launch the Robot

Start the simulation with Gazebo and RViz:

```bash
ros2 launch go2w_gazebo sim.launch.py
```

To use a custom world and initial pose:

```bash
ros2 launch go2w_gazebo sim.launch.py world:=/absolute/path/world.sdf x:=1.0 y:=2.0 z:=0.43 Y:=1.57
```

To run without windows, add `gui:=false rviz:=false`. Custom worlds should include a ground surface and the Physics, UserCommands, SceneBroadcaster, Imu, and Sensors systems, as shown in [empty.sdf](worlds/empty.sdf). World and configuration paths must be accessible inside the container when using Docker.

<a id="spawn-into-an-existing-world"></a>

### 🌍 Spawn into an Existing World

Insert the robot into a running Gazebo world:

```bash
ros2 launch go2w_gazebo spawn.launch.py world_name:=my_world bridge_clock:=true
```

Set `world_name` to the name declared inside the SDF file. Leave `bridge_clock:=false` if the world already has a ROS clock bridge.

### 🎮 Drive the Robot

Publish forward velocity in m/s using `linear.x` and yaw rate in rad/s using `angular.z`:

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

Use a negative `linear.x` to reverse, or set `linear.x` to zero and `angular.z` to `0.4` to turn in place. Stop by sending a zero command or stopping publication with Ctrl+C:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

### 👁️ View Sensor Data

In RViz, use `odom` as the fixed frame. Add an Image display for `/camera/image_raw` and a PointCloud2 display for `/utlidar/cloud` or `/pointcloud`.

## ⚙️ Parameters

Configure the simulation using these files:

| File | Settings |
| --- | --- |
| [parameters.yaml](config/parameters.yaml) | Standing posture, wheel geometry, friction, controller rate, velocity and acceleration limits, and command timeout. |
| [sensors.yaml](config/sensors.yaml) | Sensor enables, rates, ranges, noise, frames, camera resolution, intrinsics, topics, and mounting pose. |
| [lowstate.yaml](config/lowstate.yaml) | LowState publication rates and simulated battery, temperature, and power readings. |
| [gz_ros2_control.yaml](config/gz_ros2_control.yaml) | Controller types, joint interfaces, and odometry settings. |
| [ros_gz_bridge.yaml](config/ros_gz_bridge.yaml) | Gazebo-to-ROS topic mappings. |

Default driving limits are 0.5 m/s and 0.75 rad/s, with a command timeout of 0.5 seconds of simulation time. Restart the simulation after changing configuration files; rebuild the Docker image when changing its bundled files.

### 🛠️ Launch Arguments

Both launch files accept the following arguments unless indicated:

| Argument | Default | Description |
| --- | --- | --- |
| `world` | Bundled `worlds/empty.sdf` | World file; `sim.launch.py` only. |
| `world_name` | `default` | Existing world's SDF name; `spawn.launch.py` only. |
| `robot_name` | `go2w` | Gazebo entity name. |
| `x`, `y`, `z` | `0.0`, `0.0`, `0.43` | Initial position in meters. |
| `R`, `P`, `Y` | `0.0`, `0.0`, `0.0` | Initial roll, pitch, and yaw in radians. |
| `gui` | `true` | Open Gazebo; `sim.launch.py` only. |
| `rviz` | `true` for simulation, `false` for spawn | Open RViz. |
| `bridge_clock` | `false` | Bridge the world clock; `spawn.launch.py` only. |
| `enable_sensors` | `true` | Enable simulated sensors. |
| `sensors_config` | Installed `config/sensors.yaml` | Path to a custom sensor configuration. |
| `enable_lowstate` | `true` | Publish Unitree LowState telemetry. |
| `lowstate_config` | Installed `config/lowstate.yaml` | Path to a custom LowState configuration. |
| `startup_timeout` | `60.0` | Timeout per startup phase in wall-clock seconds. |

For example, load a custom sensor configuration:

```bash
ros2 launch go2w_gazebo sim.launch.py sensors_config:=/absolute/path/sensors.yaml
```

### 🔋 LowState Telemetry

Motor and IMU feedback are included in `/lowstate` and `/lf/lowstate`. Simulated battery, temperature, and power readings are configured in [lowstate.yaml](config/lowstate.yaml) and can be updated while the simulation runs:

```bash
ros2 param set /lowstate_publisher bms_state.soc 75
ros2 param set /lowstate_publisher power_v 31.0
ros2 param set /sensor_adapter temperature 42
```

Defaults are 100% charge, 33.6 V supply, zero current, and 25°C temperatures. Save persistent values in the configuration files.

## 📡 Interfaces

The following table shows the default topics and publication rates:

| Topic | Message type | Description |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Forward velocity and yaw-rate commands. |
| `/leg_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Leg position commands. |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry at 50 Hz. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Feedback for all 16 movable joints. |
| `/lowstate`, `/lf/lowstate` | `unitree_go/msg/LowState` | Motor and IMU feedback with simulated telemetry at 200 Hz and 50 Hz. |
| `/imu` | `unitree_go/msg/IMUState` | Unitree body IMU at 500 Hz. |
| `/imu/data` | `sensor_msgs/msg/Imu` | Standard body IMU at 500 Hz. |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | LiDAR IMU at 250 Hz. |
| `/utlidar/cloud`, `/pointcloud` | `sensor_msgs/msg/PointCloud2` | LiDAR point cloud at 10 Hz. |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | RGB image, 1280 × 720 at 15 Hz. |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration. |
| `/ground_truth/poses` | `geometry_msgs/msg/PoseArray` | Robot pose measured by Gazebo in the world frame. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Robot and sensor transforms. |

Use `use_sim_time:=true` in nodes consuming simulation data. The TF tree connects `odom` to the robot's `base` frame and its sensor frames.
