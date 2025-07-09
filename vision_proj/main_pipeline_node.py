# vision_proj/vision_proj/main_pipeline_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String # 예시: 명령 메시지
import time

class MainPipelineNode(Node):
    def __init__(self):
        super().__init__('main_pipeline_node')
        self.get_logger().info('MainPipelineNode started. Waiting for commands or initiating sequence.')

        # (선택 사항) 특정 시퀀스 시작 또는 명령 구독
        # self.command_subscriber = self.create_subscription(
        #     String,
        #     'pipeline/command',
        #     self.command_callback,
        #     10
        # )

        # 예시: 일정 시간 후 특정 로직 실행
        self.timer = self.create_timer(5.0, self.initial_sequence_timer_callback)
        self.sequence_started = False

    def initial_sequence_timer_callback(self):
        if not self.sequence_started:
            self.get_logger().info("Initiating initial robot movement or vision task after 5 seconds...")
            # 여기에 초기 로봇 동작, 스캔 시작 등의 로직을 추가할 수 있습니다.
            # 예를 들어, robot_control_node로 초기 자세 명령을 보내거나
            # vision_pipeline의 특정 동작을 트리거할 수 있습니다.

            # (예시) robot_control_node에 'home' 명령 발행 (robot_control_node가 해당 명령을 구독해야 함)
            # cmd_msg = String()
            # cmd_msg.data = "move_to_home_pose"
            # self.command_publisher.publish(cmd_msg)

            self.sequence_started = True
            # 한 번만 실행할 경우 타이머 비활성화
            self.timer.cancel()

    # def command_callback(self, msg):
    #     self.get_logger().info(f"Received command: {msg.data}")
    #     if msg.data == "start_grasp":
    #         self.start_grasping_sequence()
    #     elif msg.data == "reset":
    #         self.reset_system()

    # def start_grasping_sequence(self):
    #     self.get_logger().info("Starting grasping sequence...")
    #     # 여기서는 robot_control_node에게 파지 동작을 지시하는 메시지를 발행할 수 있습니다.
    #     # 또는 vision_nodes가 특정 방식으로 동작하도록 지시할 수 있습니다.
    #     pass

    # def reset_system(self):
    #     self.get_logger().info("Resetting the system...")
    #     pass

def main(args=None):
    rclpy.init(args=args)
    main_pipeline_node = MainPipelineNode()
    rclpy.spin(main_pipeline_node)
    main_pipeline_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()