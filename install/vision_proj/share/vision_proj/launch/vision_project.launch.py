# vision_proj/launch/vision_proj.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('vision_proj')

    # 설정 파일 경로
    camera_calib_file = os.path.join(package_dir, 'config', 'camera_calib.yaml')
    robot_config_file = os.path.join(package_dir, 'config', 'robot_config.yaml')
    yolo_model_path = os.path.join(package_dir, 'models', 'yolov8n.pt') # 모델 파일은 직접 다운로드하여 이곳에 저장해야 함

    return LaunchDescription([
        Node(
            package='vision_proj',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{'use_sim_time': False}] # 시뮬레이션 환경이 아니라면 False
        ),
        Node(
            package='vision_proj',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen',
            parameters=[
                {'model_path': yolo_model_path}
            ]
        ),
        Node(
            package='vision_proj',
            executable='coordinate_publisher_node',
            name='coordinate_publisher_node',
            output='screen',
            parameters=[
                {'camera_calibration_file': camera_calib_file}
            ]
        ),
        Node(
            package='vision_proj',
            executable='robot_control_node',
            name='robot_control_node',
            output='screen',
            parameters=[
                {'robot_config_file': robot_config_file},
                {'serial_port': '/dev/ttyUSB0'} # OpenCR 시리얼 포트, 실제 환경에 맞게 변경
            ]
        ),
        Node(
            package='vision_proj',
            executable='main_pipeline_node',
            name='main_pipeline_node',
            output='screen'
        ),
    ])