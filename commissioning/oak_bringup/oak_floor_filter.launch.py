"""Launch only the commissioning floor filter alongside the existing camera."""
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([ExecuteProcess(
        cmd=[sys.executable, str(Path(__file__).with_name('floor_filter.py'))],
        additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
        output='screen')])
