import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from turtlesim.msg import Pose
import tf2_ros
import math

class Turtle2TF2Broadcaster(Node):
    def __init__(self):
        super().__init__('turtle2_tf2_broadcaster')
        self.declare_parameter('turtlename', 'turtle2')
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
        q = self.euler_to_quaternion(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = Turtle2TF2Broadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()