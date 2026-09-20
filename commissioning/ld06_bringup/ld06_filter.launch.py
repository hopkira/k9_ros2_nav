"""Start only the filter when the LD06 driver is already running."""
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([ExecuteProcess(
        cmd=[sys.executable, str(Path(__file__).with_name('filter_scan.py'))],
        output='screen')])
