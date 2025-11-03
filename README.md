# ROS2 Camera-LiDAR Fusion

[![License](https://img.shields.io/badge/License-MIT--Clause-blue.svg)](https://opensource.org/licenses/MIT)
[![ROS2 Version](https://img.shields.io/badge/ROS-Humble-green)](https://docs.ros.org/en/humble/index.html)

A ROS2 package for calculating **intrinsic** and **extrinsic calibration** between camera and LiDAR sensors. This repository provides an intuitive workflow to fuse data from these sensors, enabling precise projection of LiDAR points into the camera frame and offering an efficient approach to sensor fusion.

## Visual Overview

* Credit: CDonosoK

| **Static Sensors** | **Moving Sensors** |
|---------------------|--------------------|
| <img src="https://github.com/CDonosoK/ros2_camera_lidar_fusion/blob/main/assets/static_sensors.gif" alt="Static Sensors" width="400"> | <img src="https://github.com/CDonosoK/ros2_camera_lidar_fusion/blob/dev/assets/moving_sensors.gif" alt="Moving Sensors" width="400"> |

## Node Overview
This package includes the following ROS2 nodes for camera and LiDAR calibration:

| **Node Name**           | **Description**                                                                                       | **Output**                                     |
|--------------------------|-------------------------------------------------------------------------------------------------------|-----------------------------------------------|
| `get_intrinsic_camera_calibration.py`  | Computes the intrinsic calibration of the camera.                                                    | Camera intrinsic calibration file.            |
| `save_sensor_data.py`    | Records synchronized data from camera and LiDAR sensors.                                             | Sensor data file.                             |
| `extract_points.py`      | Allows manual selection of corresponding points between camera and LiDAR.                            | Corresponding points file.                    |
| `get_extrinsic_camera_calibration.py` | Computes the extrinsic calibration between camera and LiDAR sensors.                                | Extrinsic calibration file.                   |
| `lidar_camera_projection.py` | Projects LiDAR points into the camera frame using intrinsic and extrinsic calibration parameters. | Visualization of projected points.            |


## Get Started

### Prerequisites

To run this package, ensure the following dependencies are installed:
- **ROS2**: Humble or later
- **Open3D**: An open-source library that supports rapid development of software that deals with 3D data. Warning better to install `open3d<=0.18` as there are libraries that relies on the older version of `numpy<=1.26.4`
- **Docker**: To streamline the environment setup and execution.
- **NVIDIA Container Toolkit** (if using an NVIDIA GPU): For hardware acceleration.

### Installation

### Build from source

* Clone this repository to a pre-existing ROS 2 workspace
```bash
cd ~/ros2_test_ws/src
git clone -b az_dev --single-branch https://github.com/Mechazo11/ros2_camera_lidar_fusion.git
cd ..
rosdep install -r --from-paths src --rosdistro ${ROS_DISTRO} -i -y
colcon build --symlink-install
source ./install/setup.bash
```


## Usage

### Initial setup

* Ensure a configuration `yaml` file corresponding to the dataset you are working with, is available. See the `nyuapril_ptah.yaml` as an example

* Then perform the following steps in order. We use `ptah` dataset from COPED NYUAPRIL as a test case

### Obtain data

* Run the `save__data` node. Example for NYUAPRIL PTAH sequence

```bash
ros2 run ros2_camera_lidar_fusion save_data --ros-args -p config_file:=nyuapril_ptah.yaml
```

### Obtain camera`s intrinsics

```bash
ros2 run ros2_camera_lidar_fusion get_intrinsic_camera_calibration.py  --ros-args -p config_file:=nyuapril_ptah.yaml
```

### Select 2D keypoints to establish correspondences

```bash
ros2 run ros2_camera_lidar_fusion extract_points.py  --ros-args -p config_file:=nyuapril_ptah.yaml
```

### Obtain camera to lidar synchronized data

```bash
ros2 run ros2_camera_lidar_fusion get_extrinsic_camera_calibration --ros-args -p config_file:=nyuapril_ptah.yaml
```

### Test visualization

```bash
ros2 run ros2_camera_lidar_fusion lidar_camera_projection --ros-args -p config_file:=nyuapril_ptah.yaml
```

### Workflow
Follow these steps to perform calibration and data fusion:

1. **Data Recording**  
   Use `save_sensor_data.py` to capture and save synchronized data from the camera and LiDAR.

2. **Intrinsic Calibration**  
   Run `get_intrinsic_camera_calibration.py` to generate the intrinsic calibration file for the camera.

3. **Point Correspondence**  
   Execute `extract_points.py` to manually select corresponding points between camera and LiDAR. This generate the `ros2_camera_lidar_fusion_correspondences.txt` file

4. **Extrinsic Calibration**  
   Run `get_extrinsic_camera_calibration.py` to compute the transformation matrix between camera and LiDAR.

5. **LiDAR Projection**  
   Use `lidar_camera_projection.py` to project LiDAR points into the camera frame for visualization and analysis.

### Running Nodes
To execute a specific node, use the following command:
```bash
ros2 run ros2_camera_lidar_fusion <node_name>
```
For example:
```bash
ros2 run ros2_camera_lidar_fusion lidar_camera_projection.py
```

---

## Maintainer
This package is maintained by:

**Clemente Donoso**  
Email: [clemente.donosok@gmail.com](mailto:clemente.donosok@gmail.com)
GitHub: [CDonosoK](https://github.com/CDonosoK)  

---

## License
This project is licensed under the **MIT**. See the [LICENSE](LICENSE) file for details.

---
Contributions and feedback are welcome! If you encounter any issues or have suggestions for improvements, feel free to open an issue or submit a pull request.



## Misc

## Support 💖

If you find this project helpful and want to support its ongoing development, please consider supporting CDonosoK. The original repo can be found here https://github.com/CDonosoK/ros2_camera_lidar_fusion
---

#### Build Using Docker
This repository includes a pre-configured Docker setup for easy deployment. To build the Docker image:
1. Navigate to the `docker` directory:
   ```bash
   cd ros2_camera_lidar_fusion/docker
   ```
2. Run the build script:
   ```bash
   sh build.sh
   ```
   This will create a Docker image named `ros2_camera_lidar_fusion`.

#### Run the Docker Container
Once built, launch the container using:
```bash
sh run.sh
```