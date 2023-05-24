import rclpy, math
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
from .navigator import BasicNavigator

class Explorer(Node):
    def __init__(self):
        super().__init__("goal_position_planner")
        self.subscription = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10
        )
        self.costmap_subscription = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.costmap_callback,
            10
        )
        self.unknown_threshold = 0  # Customize the threshold to determine unknown areas
        self.radius = 10  # radius around the frontier point to calculate information gain
        self.obstacle_treshold = 100  # Threshold for considering a cell as an obstacle
        self.best_goal = None
        self.initialized = False
        self.costmap_iniatilized = False
        self.costmap_origin = Pose()
        self._init_nav2()

    def costmap_callback(self, msg):
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin = msg.info.origin
        if self.costmap_iniatilized is False:
            self.costmap_iniatilized = True


        
    def _init_nav2(self):
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.start_time = self.navigator.get_clock().now()
        self.timeout_duration = Duration(seconds=120)

    def get_best_goal(self, msg):
        # Process the map data and determine the best goal position
        self.map_data = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution

        # Convert the map data to a 2D grid representation
        grid = [[self.map_data[y * self.map_width + x] for x in range(self.map_width)] for y in range(self.map_height)]

        # Find the unknown areas
        frontier_points = []
        for y in range(self.map_height):
            for x in range(self.map_width):
                if grid[y][x] < self.unknown_threshold and self.is_border_cell(grid, x, y): #TODO Here was [x][y], check for possible bugs
                    frontier_points.append((x, y))

                

        # Evaluate frontier points and select the best goal position
        self.evaluate_frontier_points(grid, frontier_points)

    def is_border_cell(self, grid, x, y):
            # Check if the cell is a border cell by checking its neighbors
            neighbors = [
                (x - 1, y),
                (x + 1, y),
                (x, y - 1),
                (x, y + 1)
            ]

            for nx, ny in neighbors:
                if nx < 0 or nx >= self.map_width or ny < 0 or ny >= self.map_height:
                    continue
                if grid[ny][nx] is self.unknown_threshold:
                    return True

            return False


    def map_callback(self, msg):
        if self.initialized is False and self.costmap_iniatilized:
            self.get_best_goal(msg)
            self.send_goal_position()
            self.initialized = True
        
        elif self.costmap_iniatilized:
            self.get_best_goal(msg)
            if self.navigator.isTaskComplete():
                self.send_goal_position()
        

    def evaluate_frontier_points(self, grid, frontier_points):
        best_information_gain = float('-inf')

        for frontier_point in frontier_points:
            x, y = frontier_point
            goal = PoseStamped()
            goal.pose.position.x = x * self.resolution + self.costmap_origin.position.x
            goal.pose.position.y = y * self.resolution + self.costmap_origin.position.y

            # Compute the information gain for the current frontier point
            information_gain = self.compute_information_gain(grid, x, y)

            if information_gain > best_information_gain:
                self.best_goal = goal
                best_information_gain = information_gain
        
        if best_information_gain is float('-inf'):
            self.get_logger().warn("Best information gain is -inf!")

    def compute_information_gain(self, grid, x, y):
        information_gain = 0.0
        if abs(x) > self.costmap_width / 2:
            return float('-inf')
        if abs(y) > self.costmap_height / 2:
            return float('-inf')

        # Calculate the coordinates of the cells within the self.radius
        start_x = max(x - self.radius, 0)
        end_x = min(x + self.radius, len(grid[0]) - 1)
        start_y = max(y - self.radius, 0)
        end_y = min(y + self.radius, len(grid) - 1)

        # Count the number of unknown and obstacle cells within the self.radius
        unknown_count = 0
        obstacle_count = 0
        for j in range(start_y, end_y + 1):
            for i in range(start_x, end_x + 1):
                if grid[j][i] < self.unknown_threshold:
                    unknown_count += 1
                if grid[j][i] >= self.obstacle_treshold:
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

    def send_goal_position(self):
        # Send the goal position to the robot's control system
        if self.best_goal is not None:
            self.get_logger().info(f"Sending goal position: {self.best_goal.pose.position.x}, {self.best_goal.pose.position.y}")
            self.navigator.goToPose(self.best_goal)
            # Add code to send the goal position to the robot's control system
        else:
            self.get_logger().warn("No best goal found!")

def main(args=None):
    rclpy.init(args=args)
    goal_position_planner = Explorer()
    rclpy.spin(goal_position_planner)
    goal_position_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
