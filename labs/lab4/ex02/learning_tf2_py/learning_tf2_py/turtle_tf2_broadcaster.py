import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
import tf2_ros

class TurtleTF2Broadcaster(Node):
    def __init__(self):
        super().__init__('turtle_tf2_broadcaster')
        self.declare_parameter('turtlename', 'turtle1')
        self.turtlename = self.get_parameter('turtlename').value
        self.br = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Pose,
            f'/{self.turtlename}/pose',
            self.handle_turtle_pose,
            10
        )

    def handle_turtle_pose(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTF2Broadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()