# vision_proj/setup.py

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_proj'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.py'))), # launch 파일 추가
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))), # config 파일 추가
        ('share/' + package_name + '/models', glob(os.path.join('models', '*.pt'))), # 모델 파일 추가
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name', # 본인의 이름으로 변경
    maintainer_email='your.email@example.com', # 본인의 이메일로 변경
    description='ROS2 package for D435i vision and OpenManipulator-X control',
    license='Apache-2.0', # 적절한 라이선스로 변경
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = vision_proj.camera_node:main',
            'object_detection_node = vision_proj.object_detection_node:main',
            'coordinate_publisher_node = vision_proj.coordinate_publisher_node:main',
            'robot_control_node = vision_proj.robot_control_node:main',
            'main_pipeline_node = vision_proj.main_pipeline_node:main',
        ],
    },
)