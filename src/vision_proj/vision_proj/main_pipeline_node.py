import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

class MainPipelineNode(Node):
    def __init__(self):
        super().__init__('main_pipeline_node')

        # 상태 플래그
        self.busy = False

        # 3D 물체 위치 Subscribe(Raw)
        self.create_subscription(
            PointStamped,
            'object/position3d_robot_frame_raw',
            self.on_object_detected,
            10
        )

        # robot_control_node에 좌표 Publish
        self.target_pub = self.create_publisher(
            PointStamped,
            'object/position3d_robot_frame',
            10
        )

        # 플레이스 완료 신호 구독 (robot_control_node 에서 Publish)
        self.create_subscription(
            Bool,
            'robot/place_done',
            self.on_place_done,
            10
        )

    def on_object_detected(self, msg: PointStamped):
        """
        감지된 3D 위치를 받으면,
        - busy=False 일 때만 컨트롤러로 전달 → 픽&플레이스 트리거
        - busy=True 면 무시
        """
        if not self.busy:
            self.busy = True
            self.get_logger().info(f'쓰레기 감지. Controller로 좌표 보냄.'
                                   f'x={msg.point.x:.3f}, y={msg.point.y:.3f}, z={msg.point.z:.3f}')
            # 새로운 topic으로 보내기
            self.target_pub.publish(msg)
        else:
            self.get_logger().debug('물체 옮기는 중!')

    def on_place_done(self, msg: Bool):
        """
        픽&플레이스 완료 신호(Bool)를 받으면 busy=False 로 풀어줘서
        다음 감지를 받을 수 있도록 허용
        """
        if msg.data:
            self.busy = False
            self.get_logger().info('이동 완료! 다음 물체 대기.')

def main(args=None):
    rclpy.init(args=args)
    node = MainPipelineNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
