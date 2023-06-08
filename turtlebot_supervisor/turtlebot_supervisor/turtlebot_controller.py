import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int8

class TurtlebotController(Node):

    last_pose = []

    def __init__(self):
        super().__init__('turtlebot_controller')

        self.last_pose = [0, 0]
        self._handle_parameters()
        self._create_subscribers()
        self._create_publishers()
        self.get_logger().info(f'topic: {self.namespace}/amcl_pose')
        
    def _handle_parameters(self):
        self._declare_default_parameters()
        self.namespace = self.get_parameter('namespace').value
        self.sector_lower_bound = self.get_parameter('sector_lower_bound').value
        self.sector_upper_bound = self.get_parameter('sector_upper_bound').value
        self.size_x = self.get_parameter('size_x').value
        self.size_y = self.get_parameter('size_y').value

    def _declare_default_parameters(self):
        self.declare_parameter('namespace', '')
        self.declare_parameter('sector_lower_bound', 0.35)
        self.declare_parameter('sector_upper_bound', 0.65)
        self.declare_parameter('size_x', 5)
        self.declare_parameter('size_y', 5)

    def _create_subscribers(self):
        self.subscription_pose = self.create_subscription(PoseWithCovarianceStamped, self.namespace + '/amcl_pose', self.listener_callback_pose, 10)

    def _create_publishers(self):
        self.publisher_free_sector = self.create_publisher(Int8, '/supervisor/free_sector', 10)

    def listener_callback_pose(self, robot_pose):
        self.last_pose[0] = robot_pose.pose.pose.position.x
        self.last_pose[1] = robot_pose.pose.pose.position.y
        if self.is_decimal_in_range(self.last_pose[0]) and self.is_decimal_in_range(self.last_pose[1]):
            sector_id = Int8()
            sector_id.data = int(self.last_pose[0]) + (int(self.last_pose[1]) * self.size_x)
            self.get_logger().info(f'sector id: {sector_id.data}')
            self.publisher_free_sector.publish(sector_id)
        self.get_logger().info(f'robot pose: {self.last_pose}')

    def is_decimal_in_range(self, value):
        value_decimal = abs(value % 1)

        if self.sector_lower_bound <= value_decimal <= self.sector_upper_bound:
            return True
        else:
            return False

def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()