# vision_proj/vision_proj/main_pipeline_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

class MainPipelineNode(Node):
    def __init__(self):
        super().__init__('main_pipeline_node')

        # 상태 플래그
        self.busy = False

        # 3D 물체 위치 감지 토픽 구독
        self.create_subscription(
            PointStamped,
            'object/position3d_robot_frame',
            self.on_object_detected,
            10
        )

        # 그대로 robot_control_node 가 구독하는 동일 토픽에 퍼블리시
        self.target_pub = self.create_publisher(
            PointStamped,
            'object/position3d_robot_frame',
            10
        )

        # 플레이스 완료 신호 구독 (robot_control_node 에서 퍼블리시한다고 가정)
        self.create_subscription(
            Bool,
            'robot/place_done',
            self.on_place_done,
            10
        )

        self.get_logger().info('MainPipelineNode ready.')

    def on_object_detected(self, msg: PointStamped):
        """
        감지된 3D 위치를 받으면,
        - busy=False 일 때만 컨트롤러로 전달 → 픽&플레이스 트리거
        - busy=True 면 무시
        """
        if not self.busy:
            self.busy = True
            self.get_logger().info(f'[PIPE] Trigger pick & place: '
                                   f'x={msg.point.x:.3f}, y={msg.point.y:.3f}, z={msg.point.z:.3f}')
            # 바로 같은 토픽으로 재발행 → robot_control_node 가 동작
            self.target_pub.publish(msg)
        else:
            self.get_logger().debug('[PIPE] Busy, ignoring new detection.')

    def on_place_done(self, msg: Bool):
        """
        픽&플레이스 완료 신호(Bool)를 받으면 busy=False 로 풀어줘서
        다음 감지를 받을 수 있도록 허용
        """
        if msg.data:
            self.busy = False
            self.get_logger().info('[PIPE] Place done, ready for next object.')

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
