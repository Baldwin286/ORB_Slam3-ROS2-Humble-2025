# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2

# class CameraPublisher(Node):

#     def __init__(self):
#         super().__init__('camera_publisher')
#         self.publisher_ = self.create_publisher(Image, 'camera', 10)
#         self.timer = self.create_timer(0.1, self.timer_callback)
#         self.bridge = CvBridge()
#         self.cap = cv2.VideoCapture(2)

#     def timer_callback(self):
#         ret, frame = self.cap.read()
#         if ret:
#             msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
#             msg.header.stamp = self.get_clock().now().to_msg()  # Set the current time as the timestamp
#             self.publisher_.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     camera_publisher = CameraPublisher()
#     rclpy.spin(camera_publisher)
#     camera_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

    
        self.publisher_ = self.create_publisher(Image, 'camera', 10)
        self.bridge = CvBridge()

        
        self.cap = cv2.VideoCapture(2)  

    
        if not self.cap.isOpened():
            self.get_logger().error('Camera not found!')
            return

        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        
        self.last_time = time.time()

        
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info('Camera publisher started!')

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Frame error.')
            return

        
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)

        
        now = time.time()
        fps = 1.0 / (now - self.last_time)
        self.last_time = now
        self.get_logger().info(f"FPS: {fps:.1f}")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

