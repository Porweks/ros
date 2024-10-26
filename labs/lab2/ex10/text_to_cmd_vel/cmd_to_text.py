import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys

class TextToCmdVelNode(Node):
    def __init__(self):
        super().__init__('text_to_cmd_vel')
        
        self.cmd_text_pub = self.create_publisher(String, 'cmd_text', 10)

        timer_period = .1  # seconds

        self.timer = self.create_timer(timer_period, self.publish_command)
        self.flag = False


        self.commands = ["turn_right", "turn_left", "move_forward", "move_backward"]

    def publish_command(self):
        """
        Публикация команды в топик cmd_text.
        """
        command = sys.argv[1]
        if command in self.commands:
            msg = String()
            msg.data = command
            self.cmd_text_pub.publish(msg)
            self.get_logger().info(f"Отправлена команда: {msg.data}")
        else:
            self.get_logger().warn(f"Неизвестная команда: {command}")
        self.flag = True

def main(args=None):
    rclpy.init(args=args)
    node = TextToCmdVelNode()
    while rclpy.ok():
        rclpy.spin_once()
        if node.flag == True:
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()