#!/usr/bin/env python3
"""
@file      kitti_to_rosbag.py
@brief     KITTI Velodyne data to ROS2 bag converter
@author    Seungwon Choi
@date      2025-09-28
@copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.

@par License
This project is released under the MIT License.

@par Usage
python3 kitti_to_rosbag.py /path/to/KITTI/Velodyne/07

@par Description
Converts KITTI .bin files to ROS2 bag format for LiDAR odometry testing.
Reads Velodyne binary point cloud files from KITTI dataset and converts them
to ROS2 bag format with configurable frequency for odometry algorithm testing.
"""

import os
import sys
import struct
import argparse
import shutil
from pathlib import Path
import numpy as np
from typing import List, Tuple

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import rosbag2_py
from builtin_interfaces.msg import Time


class KittiToBag:
    """Convert KITTI Velodyne data to ROS2 bag"""
    
    def __init__(self, kitti_path: str, output_bag_path: str = None, frequency: float = 10.0):
        """
        Initialize converter
        
        Args:
            kitti_path: Path to KITTI sequence directory (e.g., /data/KITTI/Velodyne/07)
            output_bag_path: Output bag path (default: sequence_name.bag in same directory)
            frequency: Recording frequency in Hz (default: 10.0)
        """
        self.kitti_path = Path(kitti_path)
        self.sequence_name = self.kitti_path.name
        
        # Set output bag path
        if output_bag_path is None:
            # Create rosbag directory and put data.bag inside
            rosbag_dir = self.kitti_path / "rosbag"
            rosbag_dir.mkdir(exist_ok=True)
            self.output_bag_path = rosbag_dir
        else:
            self.output_bag_path = Path(output_bag_path)
        
        self.frequency = frequency
        self.dt = 1.0 / frequency  # Time interval between frames
        
        # Velodyne data directory
        self.velodyne_dir = self.kitti_path / "velodyne"
        
        if not self.velodyne_dir.exists():
            raise FileNotFoundError(f"Velodyne directory not found: {self.velodyne_dir}")
        
        print(f"🚗 KITTI to ROS2 Bag Converter")
        print(f"📁 Input:  {self.kitti_path}")
        print(f"📦 Output: {self.output_bag_path}")
        print(f"📊 Frequency: {self.frequency} Hz")
        
    def load_kitti_point_cloud(self, bin_file: Path) -> np.ndarray:
        """
        Load KITTI binary point cloud file
        
        Args:
            bin_file: Path to .bin file
            
        Returns:
            numpy array of shape (N, 4) with [x, y, z, intensity]
        """
        if not bin_file.exists():
            raise FileNotFoundError(f"Binary file not found: {bin_file}")
            
        # KITTI Velodyne format: float32 x 4 (x, y, z, intensity)
        points = np.fromfile(str(bin_file), dtype=np.float32).reshape(-1, 4)
        return points
    
    def create_pointcloud2_msg(self, points: np.ndarray, timestamp: float, frame_id: str = "velodyne") -> PointCloud2:
        """
        Convert numpy point array to ROS2 PointCloud2 message
        
        Args:
            points: numpy array of shape (N, 4) with [x, y, z, intensity]
            timestamp: timestamp in seconds
            frame_id: frame ID for the point cloud
            
        Returns:
            PointCloud2 message
        """
        # Create header
        header = Header()
        header.frame_id = frame_id
        
        # Convert timestamp to ROS2 Time
        sec = int(timestamp)
        nanosec = int((timestamp - sec) * 1e9)
        header.stamp = Time(sec=sec, nanosec=nanosec)
        
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.is_dense = False
        
        # Define point fields (x, y, z, intensity)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.point_step = 16  # 4 fields * 4 bytes each
        msg.row_step = msg.point_step * msg.width
        
        # Pack point data
        msg.data = points.astype(np.float32).tobytes()
        
        return msg
    
    def get_bin_files(self) -> List[Path]:
        """Get sorted list of .bin files"""
        bin_files = sorted(self.velodyne_dir.glob("*.bin"))
        if not bin_files:
            raise FileNotFoundError(f"No .bin files found in {self.velodyne_dir}")
        return bin_files
    
    def convert_to_bag(self) -> bool:
        """
        Convert KITTI data to ROS2 bag
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Get list of binary files
            bin_files = self.get_bin_files()
            print(f"📋 Found {len(bin_files)} point cloud files")
            
            # Remove existing bag directory if it exists
            if self.output_bag_path.exists():
                print(f"🗑️  Removing existing bag: {self.output_bag_path}")
                if self.output_bag_path.is_dir():
                    shutil.rmtree(self.output_bag_path)
                else:
                    self.output_bag_path.unlink()
            
            # Initialize rosbag2
            writer = rosbag2_py.SequentialWriter()
            
            # Storage options (use sqlite3 instead of mcap)
            storage_options = rosbag2_py.StorageOptions(uri=str(self.output_bag_path), storage_id='sqlite3')
            
            # Converter options  
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            
            writer.open(storage_options, converter_options)
            
            # Create topic info
            topic_metadata = rosbag2_py.TopicMetadata(
                name='/velodyne_points',
                type='sensor_msgs/msg/PointCloud2',
                serialization_format='cdr'
            )
            writer.create_topic(topic_metadata)
            
            print(f"🎬 Starting conversion...")
            
            # Process each binary file
            for i, bin_file in enumerate(bin_files):
                try:
                    # Load point cloud
                    points = self.load_kitti_point_cloud(bin_file)
                    
                    # Calculate timestamp (starting from 0)
                    timestamp = i * self.dt
                    
                    # Create ROS2 message
                    msg = self.create_pointcloud2_msg(points, timestamp)
                    
                    # Write to bag
                    timestamp_ns = int(timestamp * 1e9)
                    writer.write('/velodyne_points', serialize_message(msg), timestamp_ns)
                    
                    # Progress update
                    if (i + 1) % 50 == 0 or i == len(bin_files) - 1:
                        progress = (i + 1) / len(bin_files) * 100
                        print(f"⏳ Progress: {progress:.1f}% ({i+1}/{len(bin_files)}) - {bin_file.name}")
                        
                except Exception as e:
                    print(f"❌ Error processing {bin_file}: {e}")
                    continue
            
            # Close writer
            writer.close()
            
            print(f"✅ Conversion completed!")
            print(f"📦 Output bag: {self.output_bag_path}")
            print(f"📊 Total frames: {len(bin_files)}")
            print(f"⏱️  Duration: {len(bin_files) * self.dt:.1f} seconds")
            
            return True
            
        except Exception as e:
            print(f"❌ Conversion failed: {e}")
            return False


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="Convert KITTI Velodyne data to ROS2 bag format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 kitti_to_rosbag.py /home/eugene/data/KITTI/Velodyne/07
  python3 kitti_to_rosbag.py /data/KITTI/00 --output /tmp/kitti_00.bag --freq 20
        """
    )
    
    parser.add_argument('kitti_path', type=str,
                        help='Path to KITTI sequence directory (e.g., /data/KITTI/Velodyne/07)')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Output bag file path (default: sequence_name.bag in input directory)')
    parser.add_argument('--freq', '-f', type=float, default=10.0,
                        help='Recording frequency in Hz (default: 10.0)')
    
    args = parser.parse_args()
    
    # Check if input path exists
    if not Path(args.kitti_path).exists():
        print(f"❌ Error: Path does not exist: {args.kitti_path}")
        sys.exit(1)
    
    try:
        # Initialize converter
        converter = KittiToBag(
            kitti_path=args.kitti_path,
            output_bag_path=args.output,
            frequency=args.freq
        )
        
        # Convert to bag
        success = converter.convert_to_bag()
        
        if success:
            print(f"\n🎉 Successfully converted KITTI data to ROS2 bag!")
            print(f"🚀 You can now play it with:")
            print(f"   ros2 bag play {converter.output_bag_path}")
        else:
            print(f"\n❌ Conversion failed!")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print(f"\n⛔ Conversion interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
