import threading
from group_project import coordinates
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
import signal
from group_project import AJBastroalign
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from math import sin, cos


class RoboNaut(Node):
    def __init__(self):
        super().__init__('robotnaut')
        self.declare_parameter('coordinates_file_path', '')
        coordinates_file_path = self.get_parameter('coordinates_file_path').get_parameter_value().string_value
        self.coordinates = coordinates.get_module_coordinates(coordinates_file_path)
        # naviagte to pose from lab 4
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # You can access the module coordinates like so:
        # Room 1:
        #   self.coordinates.module_1.entrance.x
        #   self.coordinates.module_1.entrance.y
        #   self.coordinates.module_1.center.x
        #   self.coordinates.module_1.center.y
        
        # Room 2:
        #   self.coordinates.module_1.entrance.x
        #   self.coordinates.module_2.entrance.x
        #   self.coordinates.module_2.entrance.y
        #   self.coordinates.module_2.center.x
        #   self.coordinates.module_2.center.y
    
    # send goal from lab 4
    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # NOTE: if you want, you can use the feedback while the robot is moving.


def main():
    def signal_handler(sig, frame):
        # TODO: make sure the robot stops properly here?
        rclpy.shutdown()

    rclpy.init(args=None)
    robonaut = RoboNaut()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robonaut,), daemon=True)
    thread.start()

    try:
        pass
        robonaut.send_goal(robonaut.coordinates.module_1.entrance.x,robonaut.coordinates.module_1.entrance.y,0)  # example coordinates
    except ROSInterruptException:
        pass


if __name__ == "__main__":
    main()


