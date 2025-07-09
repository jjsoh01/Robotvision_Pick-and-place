import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
import numpy as np
import message_filters
from vision_proj.transform_utils import get_3d_point_from_depth, load_camera_robot_transform
import os

class CoordinatePublisherNode(Node):
    def __init__(self):
        super().__init__('coordinate_publisher_node')
        self.bridge = CvBridge()
        self.camera_info = None
        self.declare_parameter('camera_calibration_file', '')
        calib_file = self.get_parameter('camera_calibration_file').get_parameter_value().string_value
        if calib_file and os.path.exists(calib_file):
            self.transform_camera_to_robot = load_camera_robot_transform(calib_file)
        else:
            self.transform_camera_to_robot = np.identity(4)

        # MessageFilter로 depth, camera_info, object 픽셀 동기화
        self.depth_sub = message_filters.Subscriber(self, Image, 'camera/depth/image_raw')
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, 'camera/color/camera_info')
        self.object_pixel_sub = message_filters.Subscriber(self, Point, 'object/detected_center')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.camera_info_sub, self.object_pixel_sub], 10, 0.1, allow_headerless=True
        )
        self.ts.registerCallback(self.synchronized_callback)

        self.object_3d_publisher = self.create_publisher(PointStamped, 'object/position3d_robot_frame', 10)

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
        self.object_3d_publisher.publish(object_3d_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
