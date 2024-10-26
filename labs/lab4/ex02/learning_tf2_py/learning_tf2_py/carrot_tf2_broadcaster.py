import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class CarrotTF2Broadcaster(Node):
    def __init__(self):
        super().__init__('carrot_tf2_broadcaster')
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('direction_of_rotation', 1)
        self.radius = self.get_parameter('radius').value
        self.direction = self.get_parameter('direction_of_rotation').value
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.handle_carrot_pose)
        self.angle = 0.0

    def handle_carrot_pose(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot'
        t.transform.translation.x = self.radius * math.cos(self.angle)
        t.transform.translation.y = self.radius * math.sin(self.angle)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)
        self.angle += self.direction * 0.1

def main(args=None):
    rclpy.init(args=args)
    node = CarrotTF2Broadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()