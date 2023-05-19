import rclpy
import math
import time
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from .navigator import BasicNavigator
from action_msgs.msg import GoalStatus


class GoalPositionPlanner(Node):
    def __init__(self):
        super().__init__("goal_position_planner")
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10
        )
        self._init_nav2()
        self.goal_pub = self.create_publisher(
            PoseStamped, 'goal_pose', 10)  # 10 is queue_size
        self.unknown_threshold = 1  # Customize the threshold to determine unknown areas
        self.best_goal = PoseStamped()
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initialized = False
        self.calibrated = False
        self.finished = False
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
        border_cells = []
        for y in range(map_height):
            for x in range(map_width):
                if grid[y][x] < self.unknown_threshold:
                    # if self.is_border_cell(grid, x, y, map_width, map_height):
                    #     border_cells.append((x, y))
                    # else:
                    unknown_cells.append((x, y))

        if self.calibrated is not True:
            self.x_origin = msg.info.origin.position.x
            self.y_origin = msg.info.origin.position.y
            self.calibrated = True

        # Evaluate frontier points and select the best goal position
        self.evaluate_frontier_points(grid, unknown_cells, resolution)

        # Send the goal position to the robot's control system
        if not self.initialized:
            self.navigator.goToPose(self.best_goal)
            self.initialized = True

        if self.finished:
            self.get_logger().info('------------Exploring finished-------------')
            return

        if self.navigator.isNavComplete() and self.navigator.getResult() is not None:
            self.navigator.goToPose(self.best_goal)
        self.get_logger().info('Goal has status code: {0}'.format(self.navigator.getResult()))

    def is_border_cell(self, grid, x, y, map_width, map_height):
        # Check if the cell is a border cell by checking its neighbors
        neighbors = [
            (x - 1, y),
            (x + 1, y),
            (x, y - 1),
            (x, y + 1)
        ]

        for nx, ny in neighbors:
            if nx < 0 or nx >= map_width or ny < 0 or ny >= map_height:
                continue
            if grid[ny][nx] >= self.unknown_threshold:
                return True

        return False

    def evaluate_frontier_points(self, grid, frontier_points, resolution):
        if not frontier_points:
            self.finished = True
            return

        best_information_gain = float('-inf')  # Initialize with a negative value
        map_width = len(grid[0])
        map_height = len(grid)
        for frontier_point in frontier_points:
            x, y = frontier_point
            goal = PoseStamped()
            goal.pose.position.x = x * resolution + self.x_origin
            goal.pose.position.y = y * resolution + self.y_origin

            # Check if the goal coordinates are within the map grid
            if goal.pose.position.x < 0 or goal.pose.position.x >= map_width * resolution:
                continue
            if goal.pose.position.y < 0 or goal.pose.position.y >= map_height * resolution:
                continue

            # Compute the information gain for the current frontier point
            information_gain = self.compute_information_gain(grid, x, y)

            if information_gain > best_information_gain:
                self.best_goal = goal
                best_information_gain = information_gain

        self.get_logger().info('grid dimensions = {:.2f} {:.2f}'.format(len(grid)*resolution, len(grid[0])*resolution))
        self.get_logger().info('self.best_goal = {:.2f} {:.2f}'.format(self.best_goal.pose.position.x, self.best_goal.pose.position.y))
    

    def compute_information_gain(self, grid, x, y):
        information_gain = 0.0
        radius = 5  # Radius around the frontier point to calculate information gain
        obstacle_threshold = 70  # Threshold for considering a cell as an obstacle

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
            information_gain += p_unknown * math.log2(p_unknown)

        return information_gain

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
