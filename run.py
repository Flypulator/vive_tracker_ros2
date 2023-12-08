import os
import sys
import yaml

import src.vive_tracker

dir_name = os.path.dirname(os.path.abspath(__file__))
config_path = f"{dir_name}/vive_config.yaml"
with open(config_path, 'r') as file:
    vive_config = yaml.safe_load(file)

sys.path.append(vive_config["ros2_packages_path"])

src.vive_tracker.main()