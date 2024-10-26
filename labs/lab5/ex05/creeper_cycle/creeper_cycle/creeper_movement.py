#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MovementsNode(Node):
    def __init__(self):
        super().__init__('movements_node')
        self.publisher_ = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle_step =  2 * math.pi / 1.5  # Угол для каждого лепестка
        self.current_angle = 0.0  # Текущий угол
        self.radius = 1.0  # Радиус круга
        self.linear_speed = 0.5  # Линейная скорость
        self.angular_speed = self.linear_speed / self.radius  # Угловая скорость
        self.petal_count = 0  # Счетчик лепестков

    def timer_callback(self):
        twist_msg = Twist()

        if self.petal_count < 5:
            # Движение по кругу
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = self.angular_speed
            self.current_angle += self.angular_speed * 0.1  # Обновляем текущий угол

            # Проверяем, если текущий угол больше или равен углу для лепестка
            if self.current_angle >= self.angle_step:
                self.current_angle = 0.0  # Сбрасываем текущий угол
                self.linear_speed = -self.linear_speed
                self.petal_count += 1  # Увеличиваем счетчик лепестков

        self.publisher_.publish(twist_msg)

        # Останавливаем таймер после завершения 5 лепестков
        if self.petal_count >= 5:
            self.petal_count =0

def main(args=None):
    rclpy.init(args=args)
    node = MovementsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()