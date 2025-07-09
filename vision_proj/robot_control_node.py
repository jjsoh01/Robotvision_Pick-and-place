import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import numpy as np
import time

# Dummy 컨트롤러 (실제로는 DynamixelAX12A, OpenManipulatorXKinematics import)
class DummyDynamixelAX12A:
    def __init__(self, serial_port, baudrate=1000000):
        print(f"[Dummy] Pretend connect Dynamixel at {serial_port} (baud {baudrate})")
    def set_position(self, id, position):
        print(f"[Dummy] Set motor {id} to position {position}")
    def close(self):
        print("[Dummy] Closing dummy dynamixel connection")

from vision_proj.open_manipulator_x_kinematics import OpenManipulatorXKinematics

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('robot_config_file', '')
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        robot_config_file = self.get_parameter('robot_config_file').get_parameter_value().string_value
        try:
            self.dxl_controller = DummyDynamixelAX12A(serial_port, baudrate=1000000)
        except Exception as e:
            self.dxl_controller = DummyDynamixelAX12A(serial_port, baudrate=1000000)
        self.kinematics = OpenManipulatorXKinematics(robot_config_file)
        self.object_3d_subscription = self.create_subscription(
            PointStamped,
            'object/position3d_robot_frame',
            self.object_3d_callback,
            10
        )

    def object_3d_callback(self, msg):
        target_x = msg.point.x
        target_y = msg.point.y
        target_z = msg.point.z
        print(f"[DEBUG] Received 3D pos: {target_x:.3f}, {target_y:.3f}, {target_z:.3f}")
        current_joint_angles = [0.0, 0.0, 0.0, 0.0]
        try:
            target_joint_angles = self.kinematics.inverse_kinematics(
                target_x, target_y, target_z, current_joint_angles
            )
            if target_joint_angles is None:
                print("No IK solution!")
                return
            for i, angle_rad in enumerate(target_joint_angles):
                motor_id = i + 1
                dxl_pos = self.kinematics.radian_to_dxl(angle_rad)
                if dxl_pos is not None:
                    self.dxl_controller.set_position(motor_id, dxl_pos)
        except Exception as e:
            print(f"Robot control error: {e}")

    def destroy_node(self):
        if self.dxl_controller:
            self.dxl_controller.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
