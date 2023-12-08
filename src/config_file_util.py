import os

import yaml


def get_config():
    dir_name = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = f"{dir_name}/vive_config.yaml"
    with open(config_path, 'r') as file:
        vive_config = yaml.safe_load(file)
    return vive_config