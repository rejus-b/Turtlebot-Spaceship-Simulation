import threading
from group_project import coordinates
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
import signal
from group_project import AJBastroalign
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class RoboNaut(Node):
    def __init__(self):
        super().__init__('robotnaut')
        self.declare_parameter('coordinates_file_path', '')
        coordinates_file_path = self.get_parameter('coordinates_file_path').get_parameter_value().string_value
        self.coordinates = coordinates.get_module_coordinates(coordinates_file_path)

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
        # TODO: your logic here.
    except ROSInterruptException:
        pass


if __name__ == "__main__":
    main()


