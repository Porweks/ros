import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys

class MoveToGoalNode(Node):
    def __init__(self, goal_x, goal_y, goal_theta):
        super().__init__('move_to_goal_node')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_theta = goal_theta
        self.current_pose = None
        self.state = 'moving'  # 'moving' or 'rotating'
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.flag = False

    def pose_callback(self, msg):
        self.current_pose = msg

    def timer_callback(self):
        if self.current_pose is None:
            return

        if self.state == 'moving':
            self.move_to_goal()
        elif self.state == 'rotating':
            self.rotate_to_goal()

    def move_to_goal(self):
        # Calculate the distance to the goal
        distance = math.sqrt((self.goal_x - self.current_pose.x) ** 2 + (self.goal_y - self.current_pose.y) ** 2)

        # Calculate the angle to the goal
        angle_to_goal = math.atan2(self.goal_y - self.current_pose.y, self.goal_x - self.current_pose.x)

        # Calculate the difference between the current angle and the goal angle
        angle_diff = angle_to_goal - self.current_pose.theta

        # Normalize the angle difference to [-pi, pi]
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Create a Twist message
        twist = Twist()

        # If the distance is greater than a threshold, move forward
        if distance > 0.1:
            twist.linear.x = 1.0 * distance
            twist.angular.z = 6.0 * angle_diff
        else:
            # If the distance is small, switch to rotating state
            self.state = 'rotating'
            return

        # Publish the Twist message
        self.publisher_.publish(twist)

    def rotate_to_goal(self):
        # Calculate the difference between the current angle and the goal angle
        angle_diff = self.goal_theta - self.current_pose.theta
        print(self.current_pose.theta, self.goal_theta, self.goal_theta - self.current_pose.theta)

        # Normalize the angle difference to [-pi, pi]
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Create a Twist message
        twist = Twist()

        # If the angle difference is greater than a threshold, rotate
        if abs(angle_diff) > 0.005:
            twist.angular.z = 6.0 * angle_diff
        else:
            # If the angle difference is small, stop the turtle
            twist.angular.z = 0.0
            self.get_logger().info('Goal reached!')
            self.timer.cancel()
            self.flag = True

        # Publish the Twist message
        self.publisher_.publish(twist)
        print(self.current_pose.theta)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 4:
        print("Usage: ros2 run move_to_goal move_to_goal_node <x> <y> <theta>")
        return

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    goal_theta = float(sys.argv[3])

    node = MoveToGoalNode(goal_x, goal_y, goal_theta)

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.flag:
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()