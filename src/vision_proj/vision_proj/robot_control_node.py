# vision_proj/vision_proj/robot_control_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np
import serial
import time

from vision_proj.open_manipulator_x_kinematics import OpenManipulatorXKinematics
from vision_proj.dynamixel_utils import send_joint_positions

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        self.simulation_mode = True
        # serial_port = 

        robot_config_file = "/home/jjsoh/Robotvision_ws/src/vision_proj/config/robot_config.yaml"

        # --- 시리얼 연결 (실제 로봇인 경우만)
        self.ser = None
        if not self.simulation_mode:
            try:
                self.ser = serial.Serial(serial_port, 115200, timeout=0.1)
                self.get_logger().info(f"[Robot] Opened serial port {serial_port}")
            except Exception as e:
                self.get_logger().error(f"[Robot] Failed to open serial port {serial_port}: {e}")

        # --- kinematics 세팅 ---
        self.kin = OpenManipulatorXKinematics(robot_config_file)

        # --- 시뮬레이션용 publisher 생성 ---
        if self.simulation_mode:
            self.traj_pub = self.create_publisher(
                JointTrajectory,
                '/arm_controller/joint_trajectory',
                10
            )

        # --- pick&place 목표지점 ---
        self.place_xyz = [0.15, 0.30, 0.10]

        # --- 구독/퍼블리시 ---
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

        # --- joint_state 퍼블리셔 (RViz 시각화용) ---
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        mode = "SIMULATION" if self.simulation_mode else "REAL"
        self.get_logger().info(f'[Robot] RobotControlNode ready in {mode} mode.')

    def cb_3d(self, msg: PointStamped):
        if self.busy:
            return
        self.busy = True

        px, py, pz = msg.point.x, msg.point.y, msg.point.z
        self.get_logger().info(f"[Robot] Picking at ({px:.3f}, {py:.3f}, {pz:.3f})")
        # current_j = [0.0, 0.0, np.pi/2]
        pick_j = self.kin.inverse_kinematics(px, py, pz)
        # 디버깅을 위한 로거
        self.get_logger().info(f"[Robot] IK 성공 (pick) → Joint Angles: [{pick_j}]")

        if pick_j is None:
            self.get_logger().error("[Robot] IK 실패 (pick)")
            self._publish_done()
            return

        # 2) 이동 + 그리퍼 open
        self._send_cmd(pick_j, gripper=0)
        time.sleep(5.0)

        # 3) 그리퍼 close
        self._send_cmd(pick_j, gripper=900)
        time.sleep(2.0)

        # 4) place 위치로 이동
        tx, ty, tz = self.place_xyz
        place_j = self.kin.inverse_kinematics(tx, ty, tz)
        if place_j is None:
            self.get_logger().error("[Robot] IK 실패 (place)")
            self._publish_done()
            return

        # 5) 이동(holding)
        self._send_cmd(place_j, gripper=900)
        time.sleep(5.0)

        # 6) 그리퍼 open → 물체 내려놓기
        self._send_cmd(place_j, gripper=0)
        time.sleep(2.0)

        # 7) 홈(초기) 위치로 이동 (그리퍼는 open 상태)
        home_j = [0.0, 0.0, np.pi/2, 0.0]  # 초기 자세 (필요시 각 관절 rad 단위로 수정)
        self._send_cmd(home_j, gripper=0)
        time.sleep(5.0)

        self.get_logger().info("Pick & Place 완료")
        self._publish_done()

    def _send_cmd(self, joint_rads, gripper: int):
        if self.simulation_mode:
            msg = JointTrajectory()
            msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
            point = JointTrajectoryPoint()

            joint_positions = [float(x) for x in joint_rads]
            point.positions = joint_positions
            point.time_from_start.sec = 2
            msg.points.append(point)
            self.traj_pub.publish(msg)
            self.get_logger().info(f"Gazebo로 보내는 joint 값 : {joint_rads}")

            # 3. 일정 시간 동안 RViz에 joint_states 반복 퍼블리시
            for _ in range(20):  # 20회 × 0.1초 = 2초
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = [
                    'joint1',
                    'joint2',
                    'joint3',
                    'joint4',
                    'gripper_left_joint',
                    'gripper_right_joint'
                ]
                joint_state_msg.position = joint_positions + [0.0, 0.0]  # 그리퍼는 0으로 고정
                self.joint_state_pub.publish(joint_state_msg)
                time.sleep(0.1)  # 10Hz 퍼블리시

        else:
            send_joint_positions(self.ser, joint_rads, gripper)
            

    def _publish_done(self):
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
