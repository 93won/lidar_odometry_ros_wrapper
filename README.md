# LiDAR Odometry ROS2 Wrapper

This package provides a ROS2 wrapper for the LiDAR Odometry system with Probabilistic Kernel Optimization (PKO). It enables real-time LiDAR-based odometry estimation in ROS2 environments.

## Demo Video

[![LiDAR Odometry Demo](https://img.youtube.com/vi/swrJY2EStrs/0.jpg)](https://www.youtube.com/watch?v=swrJY2EStrs)

## Features

- ⚡ Real-time LiDAR odometry processing
- 🎯 Feature-based point cloud registration  
- 🔧 Ceres Solver-based optimization with PKO
- 📈 ROS2 native implementation
- 🌐 TF2 transform broadcasting
- 📊 Trajectory visualization
- 🎮 Optional Pangolin viewer integration

## Dependencies

### ROS2 Dependencies
- `rclcpp`
- `sensor_msgs`
- `nav_msgs` 
- `geometry_msgs`
- `visualization_msgs`
- `tf2` and `tf2_ros`
- `pcl_ros` and `pcl_conversions`

### System Dependencies  
- Eigen3
- PCL (Point Cloud Library)
- Ceres Solver
- OpenGL and GLEW
- Pangolin (included as submodule)

## Installation

### 1. Setup Workspace and Clone Repository
```bash
# Create a new ROS2 workspace
mkdir -p lidar_odom_ws/src
cd lidar_odom_ws/src

# Clone the repository
git clone https://github.com/93won/lidar_odometry_ros_wrapper.git
cd lidar_odometry_ros_wrapper

# Initialize and download submodules
git submodule update --init --recursive
```

### 2. Install System Dependencies
```bash
# Ubuntu 22.04
sudo apt update
sudo apt install -y \
    libeigen3-dev \
    libpcl-dev \
    libceres-dev \
    libgl1-mesa-dev \
    libglew-dev \
    pkg-config
```

### 3. Build the Package
```bash
cd ../../  # Go back to lidar_odom_ws root
colcon build --packages-select lidar_odometry_ros
source install/setup.bash
```

## Usage

### Basic Usage
```bash
# Launch with default Velodyne topic
ros2 launch lidar_odometry_ros lidar_odometry.launch.py \
    config_file:=/path/to/your/workspace/lidar_odometry_ros_wrapper/lidar_odometry/config/kitti.yaml

# Launch with custom topic (e.g., Livox)
ros2 launch lidar_odometry_ros lidar_odometry.launch.py \
    config_file:=/path/to/your/workspace/lidar_odometry_ros_wrapper/lidar_odometry/config/kitti.yaml \
    pointcloud_topic:=/livox/pointcloud \
    use_sim_time:=true
```

### Quick Start

#### Option 1: KITTI Sample Data
Download and play the KITTI sample ROS bag file:
```bash
# Download KITTI sample bag
# https://drive.google.com/file/d/1U0tRSsc1PbEj_QThOHcD8l3qFkma3zjc/view?usp=sharing

# Terminal 1: Launch odometry system
ros2 launch lidar_odometry_ros lidar_odometry.launch.py \
    config_file:=/path/to/your/workspace/lidar_odometry_ros_wrapper/lidar_odometry/config/kitti.yaml \
    use_sim_time:=true

# Terminal 2: Play KITTI bag file
ros2 bag play /path/to/kitti_sample.bag --clock
```

#### Option 2: Livox MID360 Sample Data
Download and play the Livox MID360 sample ROS bag file:
```bash
# Download Livox MID360 sample bag
# https://drive.google.com/file/d/1UI6Qc5cdY8R61Odc7A6IU-jRWZgnCx2g/view?usp=sharing
# Source: https://www.youtube.com/watch?v=u8siB0KLFLc

# Terminal 1: Launch odometry system for Livox
ros2 launch lidar_odometry_ros lidar_odometry.launch.py \
    config_file:=/path/to/your/workspace/lidar_odometry_ros_wrapper/lidar_odometry/config/kitti.yaml \
    use_sim_time:=true \
    pointcloud_topic:=/livox/pointcloud

# Terminal 2: Play Livox bag file
ros2 bag play /path/to/livox_mid360_sample.bag --clock
```

**Note**: The Livox sample data uses standard `sensor_msgs/PointCloud2` messages, not Livox custom message format.







## KITTI Dataset Usage

### 1. Download KITTI Dataset
```bash
# Create data directory
mkdir -p ~/kitti_data
cd ~/kitti_data

# Download KITTI Odometry Dataset (example: sequence 00)
# Visit: https://www.cvlibs.net/datasets/kitti/eval_odometry.php
# Download velodyne laser data and poses

# Expected structure:
# ~/kitti_data/
# ├── sequences/
# │   └── 00/
# │       ├── velodyne/
# │       │   ├── 000000.bin
# │       │   ├── 000001.bin
# │       │   └── ...
# │       └── poses.txt
```

### 2. Convert KITTI to ROS2 Bag
```bash
# Use the provided conversion script
cd ~/ros2_ws/src/lidar_odometry_ros_wrapper/scripts

python3 kitti_to_rosbag.py \
    --kitti_dir ~/kitti_data/sequences/07 \
    --output_bag ~/kitti_data/kitti_seq07.db3 \
    --topic_name /velodyne_points \
    --frame_id velodyne
```

### 3. Run Examples

#### Option A: KITTI Dataset
```bash
# Terminal 1: Launch odometry system for KITTI
ros2 launch lidar_odometry_ros lidar_odometry.launch.py \
    config_file:=$(pwd)/lidar_odometry/config/kitti.yaml \
    use_sim_time:=true \
    pointcloud_topic:=/velodyne_points

# Terminal 2: Play KITTI bag file
ros2 bag play ~/kitti_data/kitti_seq07.db3 --clock
```

#### Option B: Livox MID360 Dataset
```bash
# Terminal 1: Launch odometry system for Livox MID360
ros2 launch lidar_odometry_ros lidar_odometry.launch.py \
    config_file:=$(pwd)/lidar_odometry/config/kitti.yaml \
    use_sim_time:=true \
    pointcloud_topic:=/livox/pointcloud

# Terminal 2: Play Livox bag file
ros2 bag play /path/to/your/livox_bag --clock
```

#### Launch Parameters
- `config_file`: Path to YAML configuration file (required)
- `use_sim_time`: Enable simulation time for bag playback (default: true)
- `pointcloud_topic`: Input point cloud topic name (default: /velodyne_points)
- `enable_rviz`: Launch RViz for visualization (default: true)


## License

This project is released under the MIT License. See the original [LiDAR Odometry repository](https://github.com/93won/lidar_odometry) for more details.

## Citation

If you use this work, please cite:

```bibtex
@article{choi2025probabilistic,
  title={Probabilistic Kernel Optimization for Robust State Estimation},
  author={Choi, Seungwon and Kim, Tae-Wan},
  journal={IEEE Robotics and Automation Letters},
  volume={10},
  number={3},
  pages={2998--3005},
  year={2025},
  publisher={IEEE},
  doi={10.1109/LRA.2025.3536294}
}
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
