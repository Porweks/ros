import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
import math

class TurtleTF2Listener(Node):
    def __init__(self):
        super().__init__('turtle_tf2_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        try:
            trans = self.tf_buffer.lookup_transform('turtle2', 'carrot', rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().info(f'transform not found: {e}')
            return

        msg = Twist()
        msg.linear.x = 1.0 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
        msg.angular.z = 4.0 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTF2Listener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()