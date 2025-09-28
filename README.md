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
# Launch the LiDAR odometry node (config_file is REQUIRED)
ros2 launch lidar_odometry_ros lidar_odometry.launch.py config_file:=/path/to/your/workspace/lidar_odometry_ros_wrapper/lidar_odometry/config/kitti.yaml
```


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

### 3. Run with KITTI Data
```bash
# Terminal 1: Launch odometry system
ros2 launch lidar_odometry_ros lidar_odometry.launch.py \
    config_file:=$(pwd)/lidar_odometry/config/kitti.yaml

# Terminal 2: Play bag file
ros2 bag play ~/kitti_data/kitti_seq07.db3
```

## License

This project is released under the MIT License. See the original [LiDAR Odometry repository](https://github.com/93won/lidar_odometry) for more details.

## Citation

If you use this work, please cite:

```bibtex
@article{choi2025probabilistic,
  title={Probabilistic Kernel Optimization for Robust State Estimation},
  author={Choi, Seungwon and Kim, Tae-Wook},
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
