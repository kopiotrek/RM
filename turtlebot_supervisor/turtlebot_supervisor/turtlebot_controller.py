import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Int8
from turtlebot_interfaces.action import ReserveSector

class TurtlebotController(Node):

    last_pose = []

    def __init__(self):
        super().__init__('turtlebot_controller')

        self.last_pose = [0, 0]
        self.next_pose = [0, 0]
        self.last_requested_sector_id = -1
        self.last_sector_id = -1
        self.current_sector_id = -1
        self._handle_parameters()
        self._create_subscribers()
        self._create_publishers()
        self.get_logger().info(f'topic: {self.namespace}/amcl_pose')
        self.get_logger().info(f'topic: {self.namespace}/current_goal')

        self._action_client = ActionClient(self, ReserveSector, 'supervisor')

    def send_goal(self, order):
        goal_msg = ReserveSector.Goal()
        goal_msg.sector = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)
        
    def _handle_parameters(self):
        self._declare_default_parameters()
        self.namespace = self.get_parameter('namespace').value
        self.sector_lower_bound = self.get_parameter('sector_lower_bound').value
        self.sector_upper_bound = self.get_parameter('sector_upper_bound').value
        self.size_x = self.get_parameter('size_x').value
        self.size_y = self.get_parameter('size_y').value

    def _declare_default_parameters(self):
        self.declare_parameter('namespace', '/robot1')
        self.declare_parameter('sector_lower_bound', 0.15)
        self.declare_parameter('sector_upper_bound', 0.85)
        self.declare_parameter('size_x', 10)
        self.declare_parameter('size_y', 10)

    def _create_subscribers(self):
        self.subscription_pose = self.create_subscription(PoseWithCovarianceStamped, self.namespace + '/amcl_pose', self.listener_callback_pose, 10)
        self.subscription_next_sector = self.create_subscription(PoseStamped, self.namespace + '/current_goal', self.listener_callback_next_sector, 10)

    def _create_publishers(self):
        self.publisher_free_sector = self.create_publisher(Int8, '/supervisor/free_sector', 10)

    def listener_callback_pose(self, robot_pose):
        self.last_pose[0] = robot_pose.pose.pose.position.x
        self.last_pose[1] = robot_pose.pose.pose.position.y

        if self.is_decimal_in_range(self.last_pose[0]) and self.is_decimal_in_range(self.last_pose[1]):
            
            self.current_sector_id = self.calculate_sector_id(math.floor(self.last_pose[0]), math.floor(self.last_pose[1]))

            if self.current_sector_id != self.last_sector_id:
                sector_id = Int8()
                sector_id.data = self.last_sector_id
                self.publisher_free_sector.publish(sector_id)
                self.last_sector_id = self.current_sector_id

            self.get_logger().info(f'sector id: {self.last_sector_id}')

    def listener_callback_next_sector(self, robot_pose):
        self.next_pose[0] = math.floor(robot_pose.pose.position.x)
        self.next_pose[1] = math.floor(robot_pose.pose.position.y)
        self.get_logger().info(f'robot requested pose: {self.next_pose}')
        sector_id = self.calculate_sector_id(self.next_pose[0], self.next_pose[1])
        if sector_id != self.last_requested_sector_id:
            self.send_goal(sector_id)
            self.last_requested_sector_id = sector_id
        # self.get_logger().info(f'robot pose: {self.next_pose}')

    def is_decimal_in_range(self, value):
        value_decimal = abs(value % 1)

        if self.sector_lower_bound <= value_decimal <= self.sector_upper_bound:
            return True
        else:
            return False
    
    def calculate_sector_id(self, x, y):
        transformed_x = x + self.size_x // 2
        transformed_y = y + self.size_y // 2
        sector_id = (self.size_y - transformed_y - 1) * self.size_x + transformed_x
        return int(sector_id)


def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()