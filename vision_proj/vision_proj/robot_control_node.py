# vision_proj/vision_proj/robot_control_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np
import time

from vision_proj.open_manipulator_x_kinematics import OpenManipulatorXKinematics


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        self.simulation_mode = False
        robot_config_file = "/home/jjsoh/Robotvision_ws/src/vision_proj/config/robot_config.yaml"

        # --- kinematics 세팅 ---
        self.kin = OpenManipulatorXKinematics(robot_config_file)

        # --- dynamixel 제어용 publisher ---
        self.dxl_pub = self.create_publisher(
            Float64MultiArray,
            'dynamixel/joint_goals',
            10
        )

        # --- 시뮬레이션용 publisher ---
        if self.simulation_mode:
            self.traj_pub = self.create_publisher(
                JointTrajectory,
                '/arm_controller/joint_trajectory',
                10
            )

        # --- pick&place 목표지점 ---
        self.place_xyz = [0.2, 0.2, 0.1]

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
        pick_j = self.kin.inverse_kinematics(px, py, pz)

        if pick_j is None:
            self.get_logger().error("[Robot] IK 실패 (pick)")
            self._publish_done()
            return

        self.get_logger().info(f"[Robot] IK 성공 (pick) → Joint Angles: {pick_j}")

        vec = np.array([px, py, pz])
        dist = np.linalg.norm(vec[:2])  # xy 평면 거리 (z는 그대로 둠)

        if dist < 0.11:  # 너무 가까우면 에러 방지
            self.get_logger().error("[Robot] Target 너무 가까움.")
            self._publish_done()
            return

        # xy 평면 방향 벡터 정규화 → 10cm 뒤쪽 offset
        dir_xy = vec[:2] / dist
        approach_xy = vec[:2] - dir_xy * 0.1  # 10cm 뒤쪽
        pz = pz + 0.05
        ax, ay, az = approach_xy[0], approach_xy[1], pz

        # Approach IK ---
        approach_j = self.kin.inverse_kinematics(ax, ay, az)
        if approach_j is None:
            self.get_logger().error("[Robot] IK 실패 (approach)")
            self._publish_done()
            return

        # 2) 이동 + 그리퍼 open
        self._send_cmd(approach_j, gripper=400)
        time.sleep(5.0)

        # Pick pose (실제 목표) ---
        pick_j = self.kin.inverse_kinematics(px, py, pz)
        if pick_j is None:
            self.get_logger().error("[Robot] IK 실패 (pick)")
            self._publish_done()
            return

        self._send_cmd(pick_j, gripper=400)  # 그대로 직진
        time.sleep(2.0)

        # 3) 그리퍼 close
        self._send_cmd(pick_j, gripper=600)
        time.sleep(4.0)

        # 4) place 위치로 이동
        tx, ty, tz = self.place_xyz
        place_j = self.kin.inverse_kinematics(tx, ty, tz)
        if place_j is None:
            self.get_logger().error("[Robot] IK 실패 (place)")
            self._publish_done()
            return

        # 5) 이동(holding)
        self._send_cmd(place_j, gripper=600)
        time.sleep(5.0)

        # 6) 그리퍼 open → 물체 내려놓기
        self._send_cmd(place_j, gripper=400)
        time.sleep(4.0)

        vec_place = np.array([tx, ty, tz])
        dist_place = np.linalg.norm(vec_place[:2])
        if dist_place > 0.11:
            dir_xy = vec_place[:2] / dist_place
            retreat_xy = vec_place[:2] - dir_xy * 0.10  # 10cm 뒤로
            rx, ry, rz = retreat_xy[0], retreat_xy[1], tz
            retreat_j = self.kin.inverse_kinematics(rx, ry, rz)
            if retreat_j is not None:
                self._send_cmd(retreat_j, gripper=400)
                time.sleep(3.0)
            else:
                self.get_logger().warn("[Robot] IK 실패 (retreat from place), 홈으로 바로 이동")

        # 7) 홈(초기) 위치로 이동
        home_j = [0.0, 0.0, np.pi/2, 0.0]
        self._send_cmd(home_j, gripper=400)
        time.sleep(5.0)

        self.get_logger().info("Pick & Place 완료")
        self._publish_done()

    def _send_cmd(self, joint_rads, gripper: int):
        # 공통: RViz/robot_state_publisher용 JointState 메시지 준비
        joint_positions = [float(x) for x in joint_rads]

        # 그리퍼 각도(있으면 사용, 없으면 0.0 고정)
        # gripper 값이 0/1/각도 등 프로젝트 규약에 맞게 변환하세요.
        gripper_left = 0.0
        gripper_right = 0.0

        if self.simulation_mode:
            # --- Gazebo / RViz 제어 ---
            msg = JointTrajectory()
            msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = 2
            msg.points.append(point)
            self.traj_pub.publish(msg)
            self.get_logger().info(f"Gazebo로 보내는 joint 값 : {joint_rads}")
        else:
            # --- 실제 로봇 제어 (Dynamixel) ---
            msg = Float64MultiArray()
            ticks = [self._rad_to_tick(r, i) for i, r in enumerate(joint_rads)]
            ticks.append(gripper)  # 마지막은 gripper 제어
            msg.data = [float(t) for t in ticks]
            self.dxl_pub.publish(msg)
            self.get_logger().info(f"Dynamixel goal ticks: {ticks}")

        # --- 공통: RViz 시각화를 위한 /joint_states 퍼블리시 (약 2초간 10Hz) ---
        for _ in range(20):
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [
                'joint1', 'joint2', 'joint3', 'joint4',
                'gripper_left_joint', 'gripper_right_joint'
            ]
            js.position = joint_positions + [gripper_left, gripper_right]
            self.joint_state_pub.publish(js)
            time.sleep(0.1)



    def _rad_to_tick(self, rad: float, joint_index: int) -> int:
        deg = np.degrees(rad)

        # === Joint별 보정 ===
        if joint_index == 2:  # joint3 특수 보정 (IK 0 rad → tick 210)
            tick = int(deg / 300.0 * 1023 + 210)

        else:
            if joint_index == 1:  # joint2만 보정 (무게 때문에 살짝 더 올림)
                deg = deg - 27  # <-- 여기서 5~10도 조정 가능

            # joint1,2,4 공통 변환
            tick = int((deg + 150) / 300.0 * 1023)

        # 안전 범위 제한
        if tick < 0: tick = 0
        if tick > 1023: tick = 1023

        return tick





    def _publish_done(self):
        done = Bool()
        done.data = True
        self.place_done_pub.publish(done)
        self.busy = False

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
