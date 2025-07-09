# vision_proj/vision_proj/camera_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()

        # ROS2 Publisher 설정
        self.color_publisher = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, 'camera/color/camera_info', 10)

        self.get_logger().info('Initializing RealSense camera...')

        # RealSense 파이프라인 설정
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # 스트림 설정: 컬러 (640x480, RGB8, 30fps), 깊이 (640x480, Z16, 30fps)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # 파이프라인 시작
        try:
            profile = self.pipeline.start(self.config)
            self.get_logger().info('RealSense camera started.')

            # 카메라 내부 파라미터 가져오기 (컬러 스트림 기준)
            intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.camera_info_msg = self.create_camera_info_message(intrinsics)

            # Timer 설정 (초당 30프레임 발행)
            self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            rclpy.shutdown()

    def create_camera_info_message(self, intrinsics):
        """카메라 내부 파라미터 메시지 생성"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = 'camera_link' # 카메라 프레임 ID (TF에서 사용)
        camera_info.width = intrinsics.width
        camera_info.height = intrinsics.height

        # K (내부 파라미터 매트릭스)
        camera_info.k = [intrinsics.fx, 0.0, intrinsics.ppx,
                         0.0, intrinsics.fy, intrinsics.ppy,
                         0.0, 0.0, 1.0]

        # P (투영 매트릭스, 스테레오 카메라의 경우)
        camera_info.p = [intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
                         0.0, intrinsics.fy, intrinsics.ppy, 0.0,
                         0.0, 0.0, 1.0, 0.0]

        # D (왜곡 계수)
        # RealSense는 주로 왜곡이 보정된 이미지를 제공하지만, SDK에서 제공하는 값 사용
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0] # 보통 리얼센스는 왜곡 보정되어 0.0
        camera_info.distortion_model = 'plumb_bob' # 일반적으로 사용되는 왜곡 모델

        return camera_info

    def timer_callback(self):
        try:
            # 프레임 대기
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return

            # NumPy 배열로 변환
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # 이미지 메시지로 변환 및 퍼블리시
            color_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1") # 16비트 깊이 이미지

            timestamp = self.get_clock().now().to_msg()
            color_msg.header.stamp = timestamp
            depth_msg.header.stamp = timestamp

            # 카메라 정보 메시지도 함께 발행 (타임스탬프 업데이트)
            self.camera_info_msg.header.stamp = timestamp

            self.color_publisher.publish(color_msg)
            self.depth_publisher.publish(depth_msg)
            self.camera_info_publisher.publish(self.camera_info_msg)

        except Exception as e:
            self.get_logger().error(f"Error in camera callback: {e}")

    def destroy_node(self):
        self.get_logger().info('Stopping RealSense pipeline...')
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()