# vision_proj/vision_proj/robot_control_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from vision_proj.open_manipulator_x_kinematics import OpenManipulatorXKinematics
import serial
import time

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # --- parameters ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('robot_config_file', '')
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        robot_config_file = self.get_parameter('robot_config_file').get_parameter_value().string_value

        # --- OpenCM (Arduino) 연결 ---
        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=0.1)
            self.get_logger().info(f"[Robot] Opened serial port {serial_port}")
        except Exception as e:
            self.get_logger().error(f"[Robot] Failed to open serial port {serial_port}: {e}")
            self.ser = None

        # --- kinematics 세팅 ---
        self.kin = OpenManipulatorXKinematics(robot_config_file)

        # --- pick&place 임시 목표지점 ---
        self.place_xyz = [0.20, 0.00, 0.10]

        # --- 상태 플래그 & 구독/퍼블리시 ---
        self.busy = False
        self.create_subscription(
            PointStamped,
            'object/position3d_robot_frame',
            self.cb_3d,
            1
        )
        self.place_done_pub = self.create_publisher(
            Bool,
            'robot/place_done',
            10
        )

        self.get_logger().info('[Robot] RobotControlNode ready.')

    def cb_3d(self, msg: PointStamped):
        # busy 중이면 무시
        if self.busy:
            return
        self.busy = True

        # 1) pick 위치 계산
        px, py, pz = msg.point.x, msg.point.y, msg.point.z
        self.get_logger().info(f"[Robot] Picking at ({px:.3f}, {py:.3f}, {pz:.3f})")
        current_j = [0.0]*4
        pick_j = self.kin.inverse_kinematics(px, py, pz, current_j)
        if pick_j is None:
            self.get_logger().error("[Robot] IK 실패 (pick)")
            self._publish_done()
            return

        # 2) 이동 + 그리퍼 open
        self._send_cmd(pick_j, gripper=0)
        time.sleep(2.0)

        # 3) 그리퍼 close
        self._send_cmd(pick_j, gripper=900)
        time.sleep(1.0)

        # 4) place 위치 계산
        tx, ty, tz = self.place_xyz
        place_j = self.kin.inverse_kinematics(tx, ty, tz, pick_j)
        if place_j is None:
            self.get_logger().error("[Robot] IK 실패 (place)")
            self._publish_done()
            return

        # 5) 이동(holding)
        self._send_cmd(place_j, gripper=900)
        time.sleep(2.0)

        # 6) 그리퍼 open → 물체 내려놓기
        self._send_cmd(place_j, gripper=0)
        time.sleep(1.0)

        self.get_logger().info("[Robot] Pick & Place 완료")
        self._publish_done()

    def _send_cmd(self, joint_rads, gripper: int):
        """OpenCM ← 'j1,j2,j3,j4,gripper\\n' 포맷 전송"""
        dxl = [ self.kin.radian_to_dxl(r) for r in joint_rads ]
        packet = dxl + [gripper]
        line = ','.join(str(int(x)) for x in packet) + '\n'
        if self.ser and self.ser.is_open:
            self.ser.write(line.encode())
            self.get_logger().debug(f"[Robot] SERIAL → {line.strip()}")

    def _publish_done(self):
        """place_done 토픽에 완료 신호 발행 & busy 해제"""
        done = Bool()
        done.data = True
        self.place_done_pub.publish(done)
        self.busy = False

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
