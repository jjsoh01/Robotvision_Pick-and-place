import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import transforms3d.euler, transforms3d.quaternions

def transform_coord(cam_point, roll_deg, pitch_deg, yaw_deg, tx, ty, tz):
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)
    yaw = np.deg2rad(yaw_deg)
    # 축변환
    P = np.array([[0,0,1],[-1,0,0],[0,-1,0]], float)
    p_aligned = P @ cam_point
    # 쿼터니언 회전 (intrinsic x→y→z)
    q = transforms3d.euler.euler2quat(roll, pitch, yaw, axes='rxyz')
    vq = np.array([0.0, *p_aligned])
    v_rot = transforms3d.quaternions.qmult(
        transforms3d.quaternions.qmult(q, vq),
        transforms3d.quaternions.qconjugate(q)
    )[1:]
    return v_rot + np.array([tx, ty, tz], float)

class MarkerTestNode(Node):
    def __init__(self):
        super().__init__('marker_test_node')
        self.pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # 테스트용 카메라 좌표
        p_cam = np.array([0.0, 0.0, 1.0])
        p_robot = transform_coord(p_cam, 0, -45, 45, -0.3, 0.3, 0.56)

        # 카메라 좌표계 점 (파랑)
        cam_marker = Marker()
        cam_marker.header.frame_id = "camera_link"
        cam_marker.header.stamp = self.get_clock().now().to_msg()
        cam_marker.ns = "points"
        cam_marker.id = 0
        cam_marker.type = Marker.SPHERE
        cam_marker.action = Marker.ADD
        cam_marker.pose.position.x = p_cam[0]
        cam_marker.pose.position.y = p_cam[1]
        cam_marker.pose.position.z = p_cam[2]
        cam_marker.scale.x = 0.02
        cam_marker.scale.y = 0.02
        cam_marker.scale.z = 0.02
        cam_marker.color.r = 0.0
        cam_marker.color.g = 0.0
        cam_marker.color.b = 1.0
        cam_marker.color.a = 1.0

        # 로봇 좌표계 점 (빨강)
        robot_marker = Marker()
        robot_marker.header.frame_id = "base_link"
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = "points"
        robot_marker.id = 1
        robot_marker.type = Marker.SPHERE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position.x = p_robot[0]
        robot_marker.pose.position.y = p_robot[1]
        robot_marker.pose.position.z = p_robot[2]
        robot_marker.scale.x = 0.02
        robot_marker.scale.y = 0.02
        robot_marker.scale.z = 0.02
        robot_marker.color.r = 1.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 0.0
        robot_marker.color.a = 1.0

        # 발행
        self.pub.publish(cam_marker)
        self.pub.publish(robot_marker)
        self.get_logger().info(f"Cam: {p_cam}  ->  Robot: {p_robot}")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
