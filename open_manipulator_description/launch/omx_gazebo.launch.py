from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    prefix = LaunchConfiguration("prefix")

    urdf_file = Command([
        PathJoinSubstitution(["/opt/ros/humble/bin/xacro"]), " ",
        PathJoinSubstitution([
            FindPackageShare("open_manipulator_description"),
            "urdf", "omx", "omx.urdf.xacro"
        ]),
        " prefix:=", prefix,
        " use_sim:=true"
    ])

    gz_world = PathJoinSubstitution([
        FindPackageShare("open_manipulator_description"),
        "gazebo", "empty.sdf"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("prefix", default_value='""'),

        # ✅ 1. Ignition Gazebo (ign gazebo)
        ExecuteProcess(
            cmd=["ign", "gazebo", "-r", gz_world],
            output="screen"
        ),

        # ✅ 2. robot_state_publisher 노드 실행
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": urdf_file,
                "use_sim_time": True
            }],
            output="screen"
        ),

        Node(
            package="ros_ign_gazebo",
            executable="create",
            arguments=["-name", "omx", "-topic", "robot_description"],
            output="screen"
        ),


        # ✅ 3. ros2_control 컨트롤러 자동 로드
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--controller-manager", "/controller_manager"],
            output="screen"
        )
    ])
