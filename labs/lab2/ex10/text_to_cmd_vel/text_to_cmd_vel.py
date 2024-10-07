# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Константы для скорости
LINEAR_SPEED = 1.0  # 1 метр в секунду
ANGULAR_SPEED = 1.5  # 1.5 радиан в секунду

class TextToCmdVelNode(Node):
    def __init__(self):
        super().__init__('text_to_cmd_vel')

        self.cmd_vel_sub = self.create_subscription(String, 'cmd_text', self.text_command_callback, 5)  

        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)


        self.commands = ["turn_right", "turn_left", "move_forward", "move_backward"]

    def text_command_callback(self, msg):
        """
        Обработчик сообщений из топика cmd_text.
        """
        self.get_logger().info(f"Получена команда: {msg.data}")

        twist_msg = Twist()

        if msg.data == "turn_right":
            twist_msg.angular.z = -ANGULAR_SPEED
            self.get_logger().info("Поворот направо")
        elif msg.data == "turn_left":
            twist_msg.angular.z = ANGULAR_SPEED
            self.get_logger().info("Поворот налево")
        elif msg.data == "move_forward":
            twist_msg.linear.x = LINEAR_SPEED
            self.get_logger().info("Движение вперед")
        elif msg.data == "move_backward":
            twist_msg.linear.x = -LINEAR_SPEED
            self.get_logger().info("Движение назад")
        else:
            self.get_logger().warn(f"Неизвестная команда: {msg.data}")
            return

        # Публикация команды скорости
        self.cmd_vel_pub.publish(twist_msg)
        self.get_logger().info("Команда опубликована")

def main(args=None):
    rclpy.init(args=args)
    node = TextToCmdVelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()