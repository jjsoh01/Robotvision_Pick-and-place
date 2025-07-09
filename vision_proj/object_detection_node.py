import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from geometry_msgs.msg import Point

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.target_classes = ['can', 'bottle', 'cup']

        self.image_subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.image_callback,
            10
        )
        self.detection_publisher = self.create_publisher(Point, 'object/detected_center', 10)

        cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image, verbose=False)
        h, w, _ = cv_image.shape
        image_center = np.array([w/2, h/2])
        detected_objs = []

        annotated_frame = cv_image.copy()
        for r in results:
            boxes = r.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                if class_name not in self.target_classes:
                    continue
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                dist = np.linalg.norm(np.array([center_x, center_y]) - image_center)
                detected_objs.append({'center_x': center_x, 'center_y': center_y, 'distance': dist})

                # === 관심 객체만 초록색 박스와 라벨 표시 ===
                label = f"{class_name}"
                color = (0, 255, 0)  # Green
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # 화면에 실시간 표시
        try:
            cv2.imshow("YOLO Detection", annotated_frame)
            key = cv2.waitKey(1)
            # (ESC로 끄고 싶으면 아래 주석 해제)
            # if key == 27:
            #     self.get_logger().info("ESC pressed. Just returning.")
            #     return
        except Exception as e:
            self.get_logger().warn(f"[DEBUG] cv2.imshow failed: {e}")

        # Pick 대상 한 개만 publish (없으면 아무것도 안 보냄)
        if detected_objs:
            detected_objs.sort(key=lambda x: x['distance'])
            best = detected_objs[0]
            point_msg = Point()
            point_msg.x = float(best['center_x'])
            point_msg.y = float(best['center_y'])
            point_msg.z = 0.0
            self.detection_publisher.publish(point_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
