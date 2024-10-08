import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from action_desc.action import MessageTurtleCommands
import math
import threading

class TurtleActionServer(Node):
    def __init__(self):
        super().__init__('turtle_action_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'turtle_action',
            self.execute_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.current_pose = Pose()
        self.initial_pose = Pose()

    def pose_callback(self, msg):
        self.current_pose = msg

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Action got: {goal_handle.request.command}")
        feedback_msg = MessageTurtleCommands.Feedback()
        result = MessageTurtleCommands.Result()

        self.initial_pose = self.current_pose

        command = goal_handle.request.command
        distance = goal_handle.request.s
        angle = goal_handle.request.angle

        if command == "forward":
            self.move_forward(distance, feedback_msg, goal_handle)
        elif command == "turn_left":
            self.rotate(angle, feedback_msg, goal_handle)
        elif command == "turn_right":
            self.rotate(-angle, feedback_msg, goal_handle)

        goal_handle.succeed()
        result.result = True
        return result

    def move_forward(self, distance, feedback_msg, goal_handle):
        twist = Twist()
        twist.linear.x = 1.0

        while self.calculate_distance() < distance:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            
        feedback_msg.odom += int(self.calculate_distance())
        
        print(feedback_msg.odom)
        goal_handle.publish_feedback(feedback_msg)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

    def rotate(self, angle, feedback_msg, goal_handle):
        twist = Twist()
        # print(self.calculate_angle() - angle, angle*2)
        while self.calculate_angle() - angle > 0 and self.calculate_angle() - angle < abs(angle*2):
            # print(self.calculate_angle() - angle)
            twist.angular.z = (self.calculate_angle() - abs(angle))/10
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.001)
            
        feedback_msg.odom = 0

        goal_handle.publish_feedback(feedback_msg)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def calculate_distance(self):
        return math.sqrt((self.current_pose.x - self.initial_pose.x)**2 + (self.current_pose.y - self.initial_pose.y)**2)

    def calculate_angle(self):
        return abs(math.degrees(self.current_pose.theta - self.initial_pose.theta))

def main(args=None):
    rclpy.init(args=args)
    turtle_action_server = TurtleActionServer()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(turtle_action_server)

    # Запуск executor в отдельном потоке
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(turtle_action_server, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        turtle_action_server.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()