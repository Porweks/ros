from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rclpy.node import Node
import rclpy
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class DepthCamera(Node):

    def __init__(self):
        super().__init__('depth_camera')

        # Publisher which will publish to the topic '/robot/cmd_vel'.
        self.velocity_publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.depth_subscriber = self.create_subscription(Image, '/robot/depth_image', self.update_depth, 10)
        self.depth_image = None
        self.timer = self.create_timer(0.5, self.move)
        self.bridge = CvBridge()

    def update_depth(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def move(self):
        vel_msg = Twist()
        if self.depth_image is not None:
            # Вычисляем среднюю глубину в центральной области изображения
            height, width = self.depth_image.shape
            center_x = width // 2
            center_y = height // 2
            roi_size = 50  # Размер области интереса (ROI)
            roi = self.depth_image[center_y - roi_size:center_y + roi_size, center_x - roi_size:center_x + roi_size]
            avg_depth = np.mean(roi)

            if avg_depth < 0.41:  # Если средняя глубина меньше 0.41 метра
                vel_msg.linear.x = 0.0
            else:
                vel_msg.linear.x = 0.3

        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    time.sleep(2)
    x = DepthCamera()
    rclpy.spin(x)
    x.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()