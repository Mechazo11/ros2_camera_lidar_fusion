#!/usr/bin/env python3

"""
ROS2 node for synchronized capture and storage of camera images and LiDAR point clouds.

Author: Clemente Donoso, comments and minor improvements by Azmyin Md. Kamal
Date: 11/02/2025
AI Tool: Claude Sonnet 4.5

Key notes
* message_filters package is used with ApproximateTimeSynchronizer used to synchronize images and lidar cloud
* images are saved as .png, lidar scans are saved as Open3D->PCD
"""

# Import
import rclpy, os, cv2, datetime
import numpy as np
from cv_bridge import CvBridge
import open3d as o3d
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
import threading

# Imports from this library
from ros2_camera_lidar_fusion.read_yaml import extract_configuration

def debug_lock():
    """Locks system in an infinite loop for debugging."""
    print("LOCK")
    while (1):
        pass

class SaveData(Node):
    def __init__(self):
        super().__init__('save_data_node')
        self.get_logger().info('Save data node has been started')

        # Declare ROS2 parameter
        self.declare_parameter('config_file', '')
        config_file_str = self.get_parameter('config_file').get_parameter_value().string_value
        
        # Config file name string cannot be empty or a blank string
        if not config_file_str or config_file_str.strip() == "":
            raise ValueError(f"SaveData: Config file name must be passed")

        config_file = extract_configuration(config_file_str) # Uses get_package_share_directory() 
        if config_file is None:
            self.get_logger().error("Failed to extract configuration file.")
            return
        
        print(f"config file: {config_file}")

        debug_lock()
        
        self.max_file_saved = config_file['general']['max_file_saved']
        self.storage_path = config_file['general']['data_folder']
        self.image_topic = config_file['camera']['image_topic']
        self.lidar_topic = config_file['lidar']['lidar_topic']
        self.keyboard_listener_enabled = config_file['general']['keyboard_listener']
        self.slop = config_file['general']['slop']

        if not os.path.exists(self.storage_path):
            os.makedirs(self.storage_path)
        self.get_logger().warn(f'Data will be saved at {self.storage_path}')

        self.image_sub = Subscriber(
            self,
            Image,
            self.image_topic
        )
        self.pointcloud_sub = Subscriber(
            self,
            PointCloud2,
            self.lidar_topic
        )

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.pointcloud_sub],
            queue_size=10,
            slop=self.slop
        )
        self.ts.registerCallback(self.synchronize_data)

        self.save_data_flag = not self.keyboard_listener_enabled
        if self.keyboard_listener_enabled:
            self.start_keyboard_listener()

    def start_keyboard_listener(self):
        """Starts a thread to listen for keyboard events."""
        def listen_for_space():
            while True:
                key = input("Press 'Enter' to save data (keyboard listener enabled): ")
                if key.strip() == '':
                    self.save_data_flag = True
                    self.get_logger().info('Space key pressed, ready to save data')
        thread = threading.Thread(target=listen_for_space, daemon=True)
        thread.start()

    def synchronize_data(self, image_msg, pointcloud_msg):
        """Handles synchronized messages and saves data if the flag is set."""
        if self.save_data_flag:
            file_name = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            self.get_logger().info(f'Synchronizing data at {file_name}')
            total_files = len(os.listdir(self.storage_path))
            if total_files < self.max_file_saved:
                self.save_data(image_msg, pointcloud_msg, file_name)
                if self.keyboard_listener_enabled:
                    self.save_data_flag = False

    def pointcloud2_to_open3d(self, pointcloud_msg):
        """Converts a PointCloud2 message to an Open3D point cloud."""
        points = []
        for p in point_cloud2.read_points(pointcloud_msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float32))
        return pointcloud

    def save_data(self, image_msg, pointcloud_msg, file_name):
        """Saves image and point cloud data to the storage path."""
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        pointcloud = self.pointcloud2_to_open3d(pointcloud_msg)
        o3d.io.write_point_cloud(f'{self.storage_path}/{file_name}.pcd', pointcloud)
        cv2.imwrite(f'{self.storage_path}/{file_name}.png', image)
        self.get_logger().info(f'Data has been saved at {self.storage_path}/{file_name}.png')


def main(args=None):
    rclpy.init(args=args)
    node = SaveData()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
