# Go2-W Simulation

Simulation of the Unitree Go2-W using ROS 2 Jazzy Jalisco and Gazebo Harmonic. The robot holds a standing posture and drives its four wheels using velocity commands, with wheel odometry, joint feedback, IMUs, LiDAR, and a front RGB camera.

| Package | Description |
| --- | --- |
| [go2w_description](go2w_description/README.md) | Robot model, meshes, and standalone RViz visualization. Included as a Git submodule. |
| [go2w_gazebo](go2w_gazebo/README.md) | Gazebo simulation, controllers, and sensors. |

## 💻 Installation

Clone the repository into your ROS 2 workspace, including its submodule:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/lucasmazzetto/go2w_simulation.git
```

If you already cloned without the submodule, run `git submodule update --init --recursive` from the repository directory. Choose the Linux or Docker setup below.

### 🐧 Linux Setup

Use Ubuntu 24.04 with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/), `colcon`, and an initialized `rosdep` installation.

Install the dependencies and build the packages. The following commands also install the pinned Unitree messages if `unitree_go` is not already available:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
sudo apt update
sudo apt install -y python3-vcstool ros-jazzy-rosidl-generator-dds-idl

if ! ros2 pkg prefix unitree_go >/dev/null 2>&1; then
  vcs import src < src/go2w_simulation/dependencies.repos
  rosdep install --from-paths src/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go --ignore-src --rosdistro jazzy -y
  colcon build --symlink-install --packages-select unitree_go
  source install/setup.bash
fi

rosdep install --from-paths src/go2w_simulation --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --packages-up-to go2w_gazebo
source install/setup.bash
```

### 🐳 Docker Setup

Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/), then build and start the simulation:

```bash
cd ~/ros2_ws/src/go2w_simulation
./build_docker.sh
./run_docker.sh
```

The runner starts Gazebo and RViz and automatically uses an available GPU. GUI mode requires X11/XWayland and a readable `XAUTHORITY` file. NVIDIA GPUs require the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

Use `./run_docker.sh --headless` to run without windows, `./run_docker.sh --cpu-rendering` to use software rendering, or `./run_docker.sh --headless bash` to open a shell.

## 🚀 Usage

### Launch the Robot

For a native installation, source ROS 2 and the workspace in each terminal before running commands:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch go2w_gazebo sim.launch.py
```

For Docker, run `./run_docker.sh` from the repository directory. To use a custom world and initial pose:

```bash
ros2 launch go2w_gazebo sim.launch.py world:=/absolute/path/world.sdf x:=1.0 y:=2.0 z:=0.43 Y:=1.57
```

To spawn into an existing running world, use the following command. Add `bridge_clock:=true` if the world has no ROS clock bridge. See the [Gazebo package guide](go2w_gazebo/README.md#spawn-into-an-existing-world) for world setup.

```bash
ros2 launch go2w_gazebo spawn.launch.py world_name:=my_world
```

### Drive the Robot

Publish forward velocity in m/s using `linear.x` and yaw rate in rad/s using `angular.z`:

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

Use a negative `linear.x` to reverse, or set `linear.x` to zero and `angular.z` to `0.4` to turn in place. Stop by sending a zero command or stopping publication with Ctrl+C; commands expire after 0.5 seconds of simulation time.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
```

With Docker, run commands in another terminal by prefixing them with `./run_docker.sh --headless`:

```bash
./run_docker.sh --headless ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

### Visualize the Model

To view the robot in RViz with joint sliders, run:

```bash
ros2 launch go2w_description display.launch.py
```

With Docker, use `./run_docker.sh ros2 launch go2w_description display.launch.py`.

## ⚙️ Parameters

Configure the simulation using these files:

| File | Settings |
| --- | --- |
| [parameters.yaml](go2w_gazebo/config/parameters.yaml) | Standing posture, wheel geometry, friction, controller rate, velocity and acceleration limits, and command timeout. |
| [sensors.yaml](go2w_gazebo/config/sensors.yaml) | Sensor enables, rates, ranges, noise, camera resolution, intrinsics, topics, and mounting pose. |
| [lowstate.yaml](go2w_gazebo/config/lowstate.yaml) | LowState publication rates and simulated battery, temperature, and power readings. |
| [gz_ros2_control.yaml](go2w_gazebo/config/gz_ros2_control.yaml) | Controller types, joint interfaces, and odometry settings. |

Default driving limits are 0.5 m/s and 0.75 rad/s. Restart the simulation after changing configuration files; rebuild the Docker image when changing its bundled files.

The main arguments for `sim.launch.py` are:

| Argument | Default | Description |
| --- | --- | --- |
| `world` | Bundled `worlds/empty.sdf` | Path to the Gazebo world. |
| `x`, `y`, `z` | `0.0`, `0.0`, `0.43` | Initial position in meters. |
| `R`, `P`, `Y` | `0.0`, `0.0`, `0.0` | Initial roll, pitch, and yaw in radians. |
| `gui`, `rviz` | `true`, `true` | Open Gazebo and RViz windows. |
| `enable_sensors` | `true` | Enable simulated sensors. |
| `sensors_config` | Installed `config/sensors.yaml` | Path to a custom sensor configuration. |
| `enable_lowstate` | `true` | Publish Unitree LowState telemetry. |
| `lowstate_config` | Installed `config/lowstate.yaml` | Path to a custom LowState configuration. |

## 📡 Interfaces

| Topic | Message type | Description |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Forward velocity and yaw-rate commands. |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry at 50 Hz. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Feedback for all 16 movable joints. |
| `/lowstate`, `/lf/lowstate` | `unitree_go/msg/LowState` | Motor and IMU feedback with simulated telemetry at 200 Hz and 50 Hz. |
| `/imu` | `unitree_go/msg/IMUState` | Unitree body IMU at 500 Hz. |
| `/imu/data` | `sensor_msgs/msg/Imu` | Standard body IMU at 500 Hz. |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | LiDAR IMU at 250 Hz. |
| `/utlidar/cloud`, `/pointcloud` | `sensor_msgs/msg/PointCloud2` | LiDAR point cloud at 10 Hz. |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | RGB image, 1280 × 720 at 15 Hz. |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Robot and sensor transforms. |

Use `use_sim_time:=true` in nodes consuming simulation data. In RViz, add an Image display for `/camera/image_raw` or a PointCloud2 display for `/utlidar/cloud`.
