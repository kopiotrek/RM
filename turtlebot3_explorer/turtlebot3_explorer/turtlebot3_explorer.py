import rclpy, math
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from .navigator import BasicNavigator
from action_msgs.msg import GoalStatus

class GoalPositionPlanner(Node):
    def __init__(self):
        super().__init__("goal_position_planner")
        self.subscription = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10
        )
        self._init_nav2()
        self.goal_pub = self.create_publisher(
            PoseStamped, 'goal_pose', 10)  # 10 is queue_size
        self.unknown_threshold = 50  # Customize the threshold to determine unknown areas
        self.best_goal = Point()
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initialized = False
        self.get_logger().info('Initialized')

    def map_callback(self, msg):
        # Process the map data and determine the best goal position
        map_data = msg.data
        map_width = msg.info.width
        map_height = msg.info.height
        resolution = msg.info.resolution

        # Convert the map data to a 2D grid representation
        grid = [[map_data[y * map_width + x] for x in range(map_width)] for y in range(map_height)]

        # Find the unknown areas
        unknown_cells = []
        for y in range(map_height):
            for x in range(map_width):
                if grid[y][x] < self.unknown_threshold:
                    unknown_cells.append((x, y))

        # Evaluate frontier points and select the best goal position
        self.evaluate_frontier_points(grid, unknown_cells, resolution)



        # Send the goal position to the robot's control system
        if self.initialized is False:
            self.send_goal_position()
            self.initialized = True
        # while self.navigator.getResult() == GoalStatus.STATUS_SUCCEEDED:
        #     self.send_goal_position()

        # 'STATUS_UNKNOWN': 0,
        # 'STATUS_ACCEPTED': 1,
        # 'STATUS_EXECUTING': 2,
        # 'STATUS_CANCELING': 3,
        # 'STATUS_SUCCEEDED': 4,
        # 'STATUS_CANCELED': 5,
        # 'STATUS_ABORTED': 6,
            
        if self.navigator.isNavComplete() is True and self.navigator.getResult() is not None:
            self.send_goal_position()
        self.get_logger().info('Goal has status code: {0}'.format(self.navigator.getResult()))

    def evaluate_frontier_points(self, grid, frontier_points, resolution):
        best_information_gain = 0.0

        for frontier_point in frontier_points:
            x, y = frontier_point
            goal = Point()
            goal.x = x * resolution
            goal.y = y * resolution

            # Compute the information gain for the current frontier point
            information_gain = self.compute_information_gain(grid, x, y)

            if information_gain > best_information_gain:
                self.best_goal = goal
                best_information_gain = information_gain

    def compute_information_gain(self, grid, x, y):
        information_gain = 0.0
        radius = 5  # Radius around the frontier point to calculate information gain
        obstacle_threshold = 90  # Threshold for considering a cell as an obstacle

        # Calculate the coordinates of the cells within the radius
        start_x = max(x - radius, 0)
        end_x = min(x + radius, len(grid[0]) - 1)
        start_y = max(y - radius, 0)
        end_y = min(y + radius, len(grid) - 1)

        # Count the number of unknown and obstacle cells within the radius
        unknown_count = 0
        obstacle_count = 0
        for j in range(start_y, end_y + 1):
            for i in range(start_x, end_x + 1):
                if grid[j][i] < self.unknown_threshold:
                    unknown_count += 1
                if grid[j][i] >= obstacle_threshold:
                    obstacle_count += 1

        # Calculate the probability of encountering an obstacle or unknown cell
        total_cells = (end_x - start_x + 1) * (end_y - start_y + 1)
        p_obstacle = obstacle_count / total_cells
        p_unknown = unknown_count / total_cells


        # Calculate the information gain using entropy formula
        if p_obstacle > 0:
            information_gain -= p_obstacle * math.log2(p_obstacle)
        if p_unknown > 0:
            information_gain -= p_unknown * math.log2(p_unknown)

        return information_gain

    # def send_goal_position(self):
    #     # Send the goal position to the robot's control system
    #     if self.best_goal is not None:
    #         self.get_logger().info(f"Sending goal position: {self.best_goal.x}, {self.best_goal.y}")
    #         goal_msg = NavigateToPose.Goal()
    #         goal_msg.pose.position.x=self.best_goal.x
    #         goal_msg.pose.position.y=self.best_goal.y
    #         goal_msg.pose = self.best_goal
    #         self.goal_pub.publish(self.best_goal)
    #         send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
    #                                                                self._feedbackCallback)
    #         rclpy.spin_until_future_complete(self, send_goal_future)
    #         self.goal_handle = send_goal_future.result()
    #     else:
    #         self.get_logger().warn("No best goal found!")

    #     self.result_future = self.goal_handle.get_result_async()

    def send_goal_position(self):
        self.get_logger().info('Started send_goal_position()')
        self.timeout_duration = Duration(seconds=120)
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = self.best_goal.x
        goal_pose.pose.position.y = self.best_goal.y
        self.navigator.goToPose(goal_pose)
        self.start_time = self.navigator.get_clock().now()
        self.i = 0
        self.get_logger().info('Going to:\t x: %s\t y: %s\t z: %s\t w: %s' % (goal_pose.pose.position.x,
                                                                    goal_pose.pose.position.y,
                                                                    goal_pose.pose.orientation.z,
                                                                    goal_pose.pose.orientation.w))

    def _init_nav2(self):
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.start_time = self.navigator.get_clock().now()
        self.timeout_duration = Duration(seconds=120)




def main(args=None):
    rclpy.init(args=args)
    goal_position_planner = GoalPositionPlanner()
    rclpy.spin(goal_position_planner)
    goal_position_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
