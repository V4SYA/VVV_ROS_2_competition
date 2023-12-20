import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   pkg_path = get_package_share_directory('autorace_camera')

   config = os.path.join(
      get_package_share_directory('autorace_core_exhausted_by_ros'),
      'calibration',
      'calibration.yaml'
      )
   
   pid_config = os.path.join(
      get_package_share_directory('autorace_core_exhausted_by_ros'),
      'calibration',
      'pid_config.yaml'
      )
   camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, "launch", "extrinsic_camera_calibration.launch.py"),
        )
   )

   pid = Node(
         package='autorace_core_exhausted_by_ros',
         executable='pid',
         namespace='drive',
         name='pid',
         parameters=[pid_config]
      )
   
   yolo = Node(
         package='autorace_core_exhausted_by_ros',
         executable='detect_signs',
         namespace='drive',
         name='yolo'
      )
   line = Node(
         package='autorace_core_exhausted_by_ros',
         executable='detect_line',
         namespace='detect',
         name='lane',
         parameters=[config]
      )

   return LaunchDescription([camera, line, pid, yolo])
