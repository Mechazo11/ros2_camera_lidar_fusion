"""
Helper methods for reading YAML file

Author: Azmyin Md. Kamal
Date: 11/02/2025
AI Tool: Claude Sonent 4.5
"""

import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def extract_configuration(config_file_name: str):
    """Return configurations parameter from the chosen yaml file."""
    
    # Config file name string cannot be empty or a blank string
    if not config_file_name or config_file_name.strip() == "":
        raise ValueError(f"SaveData: Config file name must be passed and it cannot be empty")
    
    path_to_package = Path(get_package_share_directory('ros2_camera_lidar_fusion'))
    config_file = path_to_package / 'config' / config_file_name
    
    with open(config_file, 'r') as file:
        config_data = yaml.safe_load(file)
    return config_data, path_to_package