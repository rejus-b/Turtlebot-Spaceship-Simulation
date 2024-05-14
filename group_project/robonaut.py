import threading
from group_project import coordinates
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
import signal
from group_project import AJBastroalign
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from math import sin, cos
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from .detect_window import detect_window
from .detect_button import detect_button
import time
import tf2_ros.buffer
from geometry_msgs.msg import TransformStamped
# import tf2_ros.listener

# checklist created by Leandro (Feel free to change it / modify)
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
# NOTE: The code section to walk around the actual room will start outside after 'greenFlag = True' or similar
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
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback,10)
        self.bridge = CvBridge()
        # publisher for velocity
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        
        self.robot_xyz = [0,0,0]
        self.detected_colour = 0
        self.robot_orientation = [0.0, 0.0, 0.0] # Roll Pitch Yaw (Unordered rn)
        
        #movement flags
        self.spun = False
        self.explore_room_flag = True # Used for knowing if a goal state was rejected or not when trying to explore a room
        self.explore = False # Test boolean for room explore once
        self.move_to_entrance_one = True # check if robot has moved to enterance one
        self.at_entrance_one = False # check if currently at entrance one
        
        
        # attempts for odomentry orientation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
            self.explore_room_flag = False
            return 

        self.get_logger().info('Goal accepted')
        self.explore_room_flag = True
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #print(feedback)
        # NOTE: if you want, you can use the feedback while the robot is moving.
    
    def odom_callback(self, msg):
        self.robot_xyz[0] = msg.pose.pose.position.x
        self.robot_xyz[1] = msg.pose.pose.position.y
        self.robot_xyz[2] = msg.pose.pose.position.z
        # print("X:", x, "Y:", y, "Z:", z)
        
        # care about yaw only

        while True:
            try:
                # self.get_logger().info('Attempting to get position')
                tf_output = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.1))
                self.get_logger().info(tf_output.transform.translation)
                break
            except Exception as e:
                print(e)
        
    def camera_view(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")   
        except CvBridgeError as e:
            print("An error has occoured while trying to convert image in cv: ", e)
        
        # detect window
        window_detected = detect_window(self.image)
        self.detected_colour = detect_button(self.image)
        
        
        #show camera feed
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL) 
        cv2.imshow('camera_Feed', self.image)
        cv2.resizeWindow('camera_Feed', 320, 240) 
        cv2.waitKey(1)
    
    # rotate by a given angle
    def rotation(self, end_angle):
        desired_velocity = Twist()
        # Set desired angle in radians
        # desired_velocity..... =
        desired_velocity.angular.z = 3.141597 / 8
        # store current time: t0
        t0, _ = self.get_clock().now().seconds_nanoseconds()

        current_angle = 0
        # loop to publish the velocity estimate until desired angle achieved
        # current angle = current angular velocity * (t1 - t0)
        while (current_angle < end_angle):
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
        
        
        
        
    # Function to enter the nearby room and walk around a bit
    # @PARAMS:
    #           room: (Integer, which room module you want to enter, 1 or 2) // Assume 2 rooms
    #           room_step: (Integer, which step of the room explore you want to execute, currently 1 or 2)
    def explore_room(self, room, room_step):
        # First determine which room you are trying to enter 
        if (room == 1):
            room = self.coordinates.module_1.center
        elif (room == 2):
            room = self.coordinates.module_2.center
        else:
            self.get_logger().info(f'A bad room code has been provided: {room}')
            # return(1)
        # Lets explore the 1st and 3rd quarter of the room horizontally  as our intial implementation
        
        # # First need to check if these locations are acceptable goal states        
        # if (self.goal_response_callback == 1): # Bad coords
        #     print("Bad coordinates, cannot move here.")
        #     # return(1)
        # elif (self.goal_response_callback == 0): # Good coords
        #     print("Good coordinates, moving ahead.")
        
        # You can access the module coordinates like so:
        # Room 1:
        #   self.coordinates.module_1.entrance.x
        #   self.coordinates.module_1.entrance.y
        #   self.coordinates.module_1.center.x
        #   self.coordinates.module_1.center.y
        
        # Find the equidistant locations of the room
        print(f"\nCenter of room is : {room.y}")
        
        if room_step == 1:   
            location = room.y / 4 # 1st 8th
        elif room_step == 2:
            location = room.y * 1.75 # 7 8th
        else:
            self.get_logger().info(f'The room_step provided was no accurate: {room_step}.')
        
        # Send the goal
        self.send_goal(room.x, location, 0)
        print(f"\Location is : {location}")

        # Set a local increment
        increment = location / 10 # The increment is room divided into 20ths

        '''
            This kind of solution would assume that somewhere along the y axis is traversable by the robot.
            If for example there is trash all along the midsection, this algorithm would fail to find somewhere.
        '''

        ## I need it to take some time to figure out the async calls otherwise it has a big chance of being lost
        import time
        time.sleep(2)

        # Check room flag and execute until it finds a valid location
        if self.explore_room_flag == True:
            self.get_logger().info(f'The goal was accepted: {location}')
            
        elif self.explore_room_flag == False:
            self.get_logger().info(f'The current goal was not accepted: {location}. Trying a new goal location.')
            while (self.explore_room_flag != True):
                print("Im here")
                self.send_goal(room.x, location + increment, 0)
                increment += room.y / 10 # The increment is room divided into 20ths
                if increment > room.y * 2: # If you have exceeded the entire room size stop
                    break
                elif self.explore_room_flag == True:
                    self.get_logger().info(f'The goal was accepted: {location}')

        #robonaut.send_goal(robonaut.coordinates.module_1.entrance.x,robonaut.coordinates.module_1.entrance.y,0)  # example coordinates
        

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
        while rclpy.ok():
            #print(robonaut.robot_xyz[0],robonaut.robot_xyz[1])
            if robonaut.move_to_entrance_one == True:
                robonaut.send_goal(robonaut.coordinates.module_1.entrance.x,robonaut.coordinates.module_1.entrance.y,0)  # example coordinates
                robonaut.move_to_entrance_one = False
            if abs(robonaut.robot_xyz[0] - robonaut.coordinates.module_1.entrance.x) <= 0.2 and abs(robonaut.robot_xyz[1] - robonaut.coordinates.module_1.entrance.y) <= 0.2:
                robonaut.at_entrance_one = True
            if robonaut.at_entrance_one and not robonaut.spun:
                robonaut.get_logger().info('Spin begin')
                robonaut.rotation(2 * 3.141597)
                #robonaut.spun = True
            if robonaut.at_entrance_one:
                if robonaut.detected_colour == 1:
                    robonaut.get_logger().info('At green room')
                elif robonaut.detected_colour == 2:
                    robonaut.get_logger().info('At red room')
                    
            '''
            if (robonaut.explore == False):
                robonaut.explore_room(1, 1) # Try send the bot to one side of the room after
                robonaut.rotation(2 * 3.141597)
                robonaut.explore_room(1, 2) 
                robonaut.explore = True'''
            pass
    except ROSInterruptException:
        pass
    
    # Cameras
    try:
        robonaut.camera_view()
    except:
        print("issue with the camera")
    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


