import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge

import numpy as np
from scipy.spatial.transform import Rotation as R

import message_filters
from vision_proj.transform_utils import get_3d_point_from_depth, load_camera_robot_transform
import os

def make_transform_matrix(roll, pitch, yaw, tx, ty, tz, degrees=True):
    P = np.array([
        [0, 0, 1],
        [-1, 0, 0],
        [0, -1, 0],
    ])

    # 카메라(x, y, z) → 로봇(+x, -y, -z) 매핑
    # 로봇 x = 카메라 z
    # 로봇 y = -카메라 x
    # 로봇 z = -카메라 y

    rot = R.from_euler('zyx', [yaw, pitch, roll], degrees=degrees)
    R_mat = rot.as_matrix()
    # 최종 회전행렬 = R * P
    # roll, pitch, yaw 는 축 변환 후 들어갔으므로 로봇 축 기준으로 값 넣기.
    # tx, ty, tz 는 카메라 축 기준으로 로봇 축이 어디에 있는지.
    RP = R_mat @ P
    T = np.eye(4)
    T[:3, :3] = RP
    T[:3, 3] = [tx, ty, tz]
    return T

class CoordinatePublisherNode(Node):
    def __init__(self):
        super().__init__('coordinate_publisher_node')
        self.bridge = CvBridge()
        self.camera_info = None
        self.transform_camera_to_robot = make_transform_matrix(0.0, 50, -40, 0.3, -0.58, 0.1 )

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

        point_homogeneous = np.array([camera_point_3d[0], camera_point_3d[1], camera_point_3d[2], 1.0])
        robot_point_homogeneous = np.dot(self.transform_camera_to_robot, point_homogeneous)
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
