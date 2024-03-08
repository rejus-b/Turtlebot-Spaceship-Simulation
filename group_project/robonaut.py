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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist

# checklist created by Leandro (fell free to change it/ modify)
# ============================================================================= #
# MOVEMENT
    # Objective -> the demo will last 5 minutes, hence that's our time limit.
    # Objective -> Ideally we want the robot to go fast when is distant from the target / from a turn 
    #              and slow close to a turn/ objective
    # Objective -> Find a good way to make sure the robot turn with the exaxt angulation:
    #              1) the robot is passing through the door. if 20% of the right part of the camera is a wall, turn left
    #              2) (suggestion from R.) we could use the map to understand the position of the robot and use that accordingly
    
    # Objective -> account for errors the robot could make. Some ways to do that would be:
    #              1) constantly check the robot sensors and adjust 
    #              2) use computer vision to keep the robot "in the right direction"
    
    # Objective -> planining, obstacle avoidance

# ============================================================================= #
# ============================================================================= #
# COMPUTER VISION
# TODO: take a picture of the windows 
# TODO: stich pictures
# TODO: recognise earth and moon, and do calculations
# TODO: recognise green and red circle (make sure it's a circle and not an hydrant), 
# TODO: then label the room as good or bad
# ============================================================================= #
# ============================================================================= #
# MOVEMENT AND COMPUTER VISION (actions that require both to be achieved)
# TODO: position the robot in the right way for taking a picture, hence position robot based on obstacles, lights etc
# 
# ============================================================================= #

# ============================================================================= #
# ENVIRONMENTS CREATION
# rafael suggested to copy the words and change the lighting and things inside to test everything. I think we will need to then map the place again

# ============================================================================= #
class RoboNaut(Node):
    def __init__(self):
        super().__init__('robotnaut')
        self.declare_parameter('coordinates_file_path', '')
        coordinates_file_path = self.get_parameter('coordinates_file_path').get_parameter_value().string_value
        self.coordinates = coordinates.get_module_coordinates(coordinates_file_path)
        # naviagte to pose from lab 4
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # camera subscription and bridge
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.camera_view, 10)
        self.bridge = CvBridge()
        # publisher for velocity
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz

        # You can access the module coordinates like so:
        # Room 1:
        #   self.coordinates.module_1.entrance.x
        #   self.coordinates.module_1.entrance.y
        #   self.coordinates.module_1.center.x
        #   self.coordinates.module_1.center.y
        
        # Room 2:
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
        
    def camera_view(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")   
        except CvBridgeError:
            return
       
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL) 
        cv2.imshow('camera_Feed', self.image)
        cv2.resizeWindow('camera_Feed', 320, 240) 
        cv2.waitKey(3)
    
    # rotate by a given angle
    def rotation(self, angle):
        desired_velocity = Twist()
         # Set desired angle in radians
        # desired_velocity..... =
        desired_velocity.angular.z = 3.141597 / 8
        # store current time: t0
        t0, _ = self.get_clock().now().seconds_nanoseconds()

        current_angle = 0
        # loop to publish the velocity estimate until desired angle achieved
        # current angle = current angular velocity * (t1 - t0)
        while (current_angle < angle):
            # Publish the velocity
            self.publisher.publish(desired_velocity)

            # t1 is the current time
            t1, _ = self.get_clock().now().seconds_nanoseconds()  # to_msg()

            # Calculate current angle
            current_angle = desired_velocity.angular.z * (t1 - t0)

            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop()

    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        self.publisher.publish(desired_velocity)

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
        robonaut.send_goal(robonaut.coordinates.module_1.entrance.x,robonaut.coordinates.module_1.entrance.y,0)  # example coordinates
        robonaut.rotation(2 * 3.141597)
    except ROSInterruptException:
        pass
    
    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


