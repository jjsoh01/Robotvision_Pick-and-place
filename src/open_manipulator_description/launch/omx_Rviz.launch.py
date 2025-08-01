from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 경로 설정
    description_pkg = FindPackageShare("open_manipulator_description")
    # 경로는 본인 노트북 경로에 맞게 재설정 필요.
    xacro_file = "/home/jjsoh/Robotvision_ws/src/open_manipulator_description/urdf/omx/omx.urdf.xacro"


    # xacro → urdf 변환
    robot_description_content = Command(["xacro ", xacro_file])
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen"
        ),
        # Node(
        #     package="joint_state_publisher_gui",
        #     executable="joint_state_publisher_gui",
        #     name="joint_state_publisher_gui",
        #     output="screen"
        # ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([description_pkg, "rviz", "view_omx.rviz"])]
        )
    ])
