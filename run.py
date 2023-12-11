import sys

import src.vive_tracker
from src.config_file_util import get_config

vive_config = get_config()
sys.path.append(vive_config["ros2_packages_path"])

src.vive_tracker.main()
