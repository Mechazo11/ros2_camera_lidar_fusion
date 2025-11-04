#!/usr/bin/env python3

"""
Test Lidar's projection on camera images using the obtained T_lidar_to_cam extrinsic

Author: Clemente Donoso, comments and minor improvements by Azmyin Md. Kamal
Date: 11/02/2025
AI Tool: Claude Sonnet 4.5

Usage:
    * Start this node
    * Use ros2 bag pay <name> -r 1.0 to start the rosbag
    * Output will be automatically seen??

"""

# Impport
# import os
from pathlib import Path
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
import struct

from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from ros2_camera_lidar_fusion.read_yaml import extract_configuration
from ros2_camera_lidar_fusion.utils import debug_lock


# TODO move these three methods to utils
def load_extrinsic_matrix(yaml_path: Path) -> np.ndarray:
    # if not os.path.isfile(yaml_path):
    #     raise FileNotFoundError(f"No extrinsic file found: {yaml_path}")

    if not yaml_path.exists():
        raise FileNotFoundError(f"T_lidar_to_camera extrinsic file no found in {yaml_path}")

    with open(yaml_path.as_posix(), 'r') as f:
        data = yaml.safe_load(f)

    if 'T_lidar_to_cam' not in data:
        raise KeyError(f"YAML {yaml_path} has no 'T_lidar_to_cam' key.")

    matrix_list = data['T_lidar_to_cam']
    T = np.array(matrix_list, dtype=np.float64).reshape((4,4))
    if T.shape != (4, 4):
        raise ValueError("Extrinsic matrix is not 4x4.")
    return T

def load_camera_calibration(yaml_path: Path) -> (np.ndarray, np.ndarray):
    if not yaml_path.exists():
        raise FileNotFoundError(f"No camera intrinsic calibration file found in {yaml_path}")

    with open(yaml_path, 'r') as f:
        calib_data = yaml.safe_load(f)

    cam_mat_data = calib_data['camera_matrix']['data']
    camera_matrix = np.array(cam_mat_data, dtype=np.float64).reshape((3, 3))  # Reshapes to 3x3 matrix

    dist_data = calib_data['distortion_coefficients']['data']
    dist_coeffs = np.array(dist_data, dtype=np.float64).reshape((1, -1))

    return camera_matrix, dist_coeffs


def pointcloud2_to_xyz_array_fast(cloud_msg: PointCloud2, skip_rate: int = 1) -> np.ndarray:
    if cloud_msg.height == 0 or cloud_msg.width == 0:
        return np.zeros((0, 3), dtype=np.float32)

    field_names = [f.name for f in cloud_msg.fields]
    if not all(k in field_names for k in ('x','y','z')):
        return np.zeros((0,3), dtype=np.float32)

    x_field = next(f for f in cloud_msg.fields if f.name=='x')
    y_field = next(f for f in cloud_msg.fields if f.name=='y')
    z_field = next(f for f in cloud_msg.fields if f.name=='z')

    dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('_', 'V{}'.format(cloud_msg.point_step - 12))
    ])

    raw_data = np.frombuffer(cloud_msg.data, dtype=dtype)
    points = np.zeros((raw_data.shape[0], 3), dtype=np.float32)
    points[:,0] = raw_data['x']
    points[:,1] = raw_data['y']
    points[:,2] = raw_data['z']

    if skip_rate > 1:
        points = points[::skip_rate]

    return points

class LidarCameraProjectionNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_projection_node')
        
        # Setup some global variables and objects
        self.bridge = CvBridge()
        self.skip_rate = 1

        # Declare ROS2 parameter
        self.declare_parameter('config_file', '')
        config_file_str = self.get_parameter('config_file').get_parameter_value().string_value
        config_file, _ = extract_configuration(config_file_str) # Uses get_package_share_directory() 
        
        # Build paths
        self.this_pkg_path = Path().home() / config_file['general']['ros_ws_name'] / 'src'
        self.data_dir = self.this_pkg_path / config_file['general']['data_folder']
        self.config_folder = self.this_pkg_path / config_file['general']['config_folder']
        
        self.extrinsic_yaml = self.config_folder / config_file['general']['camera_extrinsic_calibration']
        self.camera_yaml = self.config_folder / config_file['general']['camera_intrinsic_calibration']
        self.is_compressed = config_file['camera']['is_compressed']
        self.image_topic = config_file['camera']['image_topic']
        self.lidar_topic = config_file['lidar']['lidar_topic']
        self.projected__image_topic = config_file['camera']['projected_topic']

        self.T_lidar_to_cam = load_extrinsic_matrix(self.extrinsic_yaml)
        self.camera_matrix, self.dist_coeffs = load_camera_calibration(self.camera_yaml)

        print(f"DEBUG")
        print(f"camera yaml: {self.data_dir}")
        print(f"extrinsic yaml: {self.extrinsic_yaml}")
        print(f"Is image compressed: {self.is_compressed}")
        self.get_logger().info("Loaded extrinsic:\n{}".format(self.T_lidar_to_cam))
        self.get_logger().info("Camera matrix:\n{}".format(self.camera_matrix))
        self.get_logger().info("Distortion coeffs:\n{}".format(self.dist_coeffs))
        self.get_logger().info(f"Subscribing to lidar topic: {self.lidar_topic}")
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        self.get_logger().info(f"Publishing lidar`s projection to image on topic: {self.projected__image_topic}")
        print(f"DEBUG\n")

        if not self.is_compressed:
            self.image_sub = Subscriber(
                self,
                Image,
                self.image_topic
            )
        else:
            self.image_sub = Subscriber(
                self,
                CompressedImage,
                self.image_topic
            )
        
        self.lidar_sub = Subscriber(self, PointCloud2, self.lidar_topic)
        # Topic to show projected images
        self.pub_image = self.create_publisher(Image, self.projected__image_topic, 1)

        self.ts = ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub],
            queue_size=5,
            slop=0.07
        )
        self.ts.registerCallback(self.sync_callback)

        
    def sync_callback(self, image_msg: Image, lidar_msg: PointCloud2):
        """Return a pair of image-lidar scan synchronized by time"""

        if self.is_compressed:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, 'passthrough')
        else:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'passthrough')

        if len(cv_image.shape) == 2:  # grayscale
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

        xyz_lidar = pointcloud2_to_xyz_array_fast(lidar_msg, skip_rate=self.skip_rate)
        n_points = xyz_lidar.shape[0]
        if n_points == 0:
            self.get_logger().warn("Empty cloud. Nothing to project.")
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            out_msg.header = image_msg.header
            self.pub_image.publish(out_msg)
            return

        xyz_lidar_f64 = xyz_lidar.astype(np.float64)
        ones = np.ones((n_points, 1), dtype=np.float64)
        xyz_lidar_h = np.hstack((xyz_lidar_f64, ones))

        xyz_cam_h = xyz_lidar_h @ self.T_lidar_to_cam.T
        xyz_cam = xyz_cam_h[:, :3]

        mask_in_front = (xyz_cam[:, 2] > 0.0)
        xyz_cam_front = xyz_cam[mask_in_front]
        n_front = xyz_cam_front.shape[0]
        if n_front == 0:
            self.get_logger().info("No points in front of camera (z>0).")
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            out_msg.header = image_msg.header
            self.pub_image.publish(out_msg)
            return
        
        rvec = np.zeros((3,1), dtype=np.float64)
        tvec = np.zeros((3,1), dtype=np.float64)
        image_points, _ = cv2.projectPoints(
            xyz_cam_front,
            rvec, tvec,
            self.camera_matrix,
            self.dist_coeffs
        )
        image_points = image_points.reshape(-1, 2)

        h, w = cv_image.shape[:2]
        for (u, v) in image_points:
            u_int = int(u + 0.5)
            v_int = int(v + 0.5)
            if 0 <= u_int < w and 0 <= v_int < h:
                cv2.circle(cv_image, (u_int, v_int), 2, (0, 255, 0), -1)

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = image_msg.header
        self.pub_image.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
