import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge

import numpy as np
from transforms3d.quaternions import mat2quat
import transforms3d.euler
import transforms3d.quaternions

import message_filters
from vision_proj.transform_utils import get_3d_point_from_depth
import os

def transform_coord(cam_point, roll, pitch, yaw, tx, ty, tz):
    # 각도 라디안으로
    roll = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)
    yaw = np.deg2rad(yaw)

    # 축 변환
    # 카메라(x(오른쪽), y(아래), z(전방)) 
    # → 로봇(+x(전방), y(왼), z(위)) 변환
    # 로봇 x = 카메라 z
    # 로봇 y = -카메라 x
    # 로봇 z = -카메라 y
    P = np.array([
        [0, 0, 1],
        [-1, 0, 0],
        [0, -1, 0],
    ])

    p_robot_aligned = P @ cam_point

    print(f"축 변환 : x={p_robot_aligned[0]:.3f}, y={p_robot_aligned[1]:.3f}, z={p_robot_aligned[2]:.3f}")

    # 쿼터니언 회전 생성 (sxyz: x→y→z intrinsic, roll, pitch, yaw 순서.)
    q = transforms3d.euler.euler2quat(roll, pitch, yaw, axes='rxyz')
    # q = transforms3d.euler.euler2quat(yaw, pitch, roll, axes='rzyx')

    # 벡터를 쿼터니언으로 (0, x, y, z)
    v_q = np.concatenate([[0], p_robot_aligned])
    q_conj = transforms3d.quaternions.qconjugate(q)

    # 회전 적용: q * v * q_conj
    v_rot = transforms3d.quaternions.qmult(transforms3d.quaternions.qmult(q, v_q), q_conj)[1:]

    print(f"회전 : x={v_rot[0]:.3f}, y={v_rot[1]:.3f}, z={v_rot[2]:.3f}")

    # 4. 평행이동 적용
    translation = (tx, ty, tz)
    v_final = v_rot + np.array(translation)

    return v_final

class CoordinatePublisherNode(Node):
    def __init__(self):
        super().__init__('coordinate_publisher_node')
        self.bridge = CvBridge()
        self.camera_info = None

        # MessageFilter로 depth, camera_info, object 픽셀 동기화
        self.depth_sub = message_filters.Subscriber(self, Image, 'camera/depth/image_raw')
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, 'camera/color/camera_info')
        self.object_pixel_sub = message_filters.Subscriber(self, Point, 'object/detected_center')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.camera_info_sub, self.object_pixel_sub], 10, 0.1, allow_headerless=True
        )
        self.ts.registerCallback(self.synchronized_callback)

        self.object_3d_publisher = self.create_publisher(PointStamped, 'object/position3d_robot_frame_raw', 10)

    def synchronized_callback(self, depth_msg, camera_info_msg, object_pixel_msg):
        if self.camera_info is None or self.camera_info.header.stamp != camera_info_msg.header.stamp:
            self.camera_info = camera_info_msg
        if self.camera_info is None:
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
            return

        pixel_u = int(object_pixel_msg.x)
        pixel_v = int(object_pixel_msg.y)
        if not (0 <= pixel_v < depth_image.shape[0] and 0 <= pixel_u < depth_image.shape[1]):
            return

        camera_point_3d = get_3d_point_from_depth(depth_image, pixel_u, pixel_v, self.camera_info)
        if camera_point_3d is None:
            return

        self.get_logger().info(f"카메라 좌표 : x={camera_point_3d[0]:.3f}, y={camera_point_3d[1]:.3f}, z={camera_point_3d[2]:.3f}"
)

        point_homogeneous = np.array([camera_point_3d[0], camera_point_3d[1], camera_point_3d[2]])
        robot_point_homogeneous = transform_coord(point_homogeneous, 0, 0, 0, -0.25, 0.0, 0.27)
        robot_x = robot_point_homogeneous[0]
        robot_y = robot_point_homogeneous[1]
        robot_z = robot_point_homogeneous[2]

        object_3d_msg = PointStamped()
        object_3d_msg.header.stamp = depth_msg.header.stamp
        object_3d_msg.header.frame_id = 'robot_base_link'
        object_3d_msg.point.x = robot_x
        object_3d_msg.point.y = robot_y
        object_3d_msg.point.z = robot_z

        self.get_logger().info(f"로봇 좌표 : x={robot_x:.3f}, y={robot_y:.3f}, z={robot_z:.3f}"
)

        self.object_3d_publisher.publish(object_3d_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
