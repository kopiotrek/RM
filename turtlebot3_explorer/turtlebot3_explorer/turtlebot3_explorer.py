import rclpy, math, csv, numpy
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
        self.radius = 5  # radius around the frontier point to calculate information gain
        self.obstacle_treshold = 80  # Threshold for considering a cell as an obstacle
        self.best_goal = PoseStamped()
        self.previous_goals = []
        self.initialized = False
        self.costmap_iniatilized = False
        self.costmap_origin = Pose()
        self._init_nav2()
        self.csv_file = "/home/pk/Documents/map_data.csv"

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

    def map_callback(self, msg):
        if self.initialized is False and self.costmap_iniatilized:
            self.get_best_goal(msg)
            self.send_goal_position()
            self.initialized = True
        
        elif self.costmap_iniatilized:
            self.get_best_goal(msg)
            if self.navigator.isTaskComplete():
                self.send_goal_position()

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
                if grid[y][x] < self.unknown_threshold:
                    if self.is_border_cell(grid, x, y):
                        frontier_points.append((x, y))

        self.get_logger().info(f"Frontier points count: {len(frontier_points)}")
        if len(frontier_points) < 50:
            self.get_logger().info("----Mapping finished----")
            raise SystemExit

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
                    return False
                if grid[ny][nx] >= self.unknown_threshold and grid[ny][nx] < self.obstacle_treshold:
                    return True

            return False

    def check_goal_repeatition(self,current_goal,last_goal,pre_last_goal):
        if math.isclose(current_goal.pose.position.x, last_goal.pose.position.x, abs_tol = 0.1):
            if math.isclose(current_goal.pose.position.y, last_goal.pose.position.y, abs_tol = 0.1):
                return True
        if math.isclose(current_goal.pose.position.x, pre_last_goal.pose.position.x, abs_tol = 0.1):
            if math.isclose(current_goal.pose.position.y, pre_last_goal.pose.position.y, abs_tol = 0.1):
                return True
        return False

    def evaluate_frontier_points(self, grid, frontier_points):
        best_information_gain = float('-inf')
        goal_tmp = PoseStamped()
        self.last_goal = PoseStamped()
        self.preLast_goal = PoseStamped()
        goal_repeated = False

        for frontier_point in frontier_points:
            x, y = frontier_point
            # Compute the information gain for the current frontier point
            information_gain = self.compute_information_gain(grid, x, y)
            
            if information_gain > best_information_gain:
                goal_tmp.pose.position.x = x * self.resolution + self.costmap_origin.position.x
                goal_tmp.pose.position.y = y * self.resolution + self.costmap_origin.position.y     
                # goal_tmp.pose.position.x = 1.0
                # goal_tmp.pose.position.y = 2.0
                for goal in self.previous_goals:
                    if math.isclose(goal.pose.position.x, goal_tmp.pose.position.x, abs_tol = 0.1):
                        if math.isclose(goal.pose.position.y, goal_tmp.pose.position.y, abs_tol = 0.1):
                            goal_repeated = True
                            # self.get_logger().info("Repeated goal aborted")
                        
                if goal_repeated is False:
                    if self.last_goal.pose.position.x is not goal_tmp.pose.position.x:
                        if self.last_goal.pose.position.y is not goal_tmp.pose.position.y:
                            if self.last_goal.pose.position.x is not self.preLast_goal.pose.position.x:
                                if self.last_goal.pose.position.y is not self.preLast_goal.pose.position.y:
                                    best_information_gain = information_gain
                                    self.previous_goals.append(self.best_goal)
                                    self.best_goal.pose.position.x = goal_tmp.pose.position.x
                                    self.best_goal.pose.position.y = goal_tmp.pose.position.y
                                    self.preLast_goal.pose.position.x = self.last_goal.pose.position.x
                                    self.preLast_goal.pose.position.y = self.last_goal.pose.position.y
                                    self.last_goal.pose.position.x = goal_tmp.pose.position.x
                                    self.last_goal.pose.position.y = goal_tmp.pose.position.y
                                else:
                                    self.best_goal.pose.position.x = -6.0
                                    self.best_goal.pose.position.y = 0.0
                                    self.get_logger().info("Going back to starting point")
                
                goal_repeated = False

        if best_information_gain is float('-inf'):
            self.get_logger().warn("Best information gain is -inf!")

    def compute_information_gain(self, grid, x, y):
        information_gain = 0.0
        if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
            return float('-inf')

        # Calculate the coordinates of the cells within the self.radius
        start_x = max(x - self.radius, 0)
        end_x = min(x + self.radius, self.map_width - 1)
        start_y = max(y - self.radius, 0)
        end_y = min(y + self.radius,  self.map_height - 1)

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
        if self.initialized is True:
            self.get_logger().info(f"Sending goal position: {self.best_goal.pose.position.x}, {self.best_goal.pose.position.y}")
            # self.get_logger().info(f"Goal grid position: {self.goal_grid[0]}, {self.goal_grid[1]}")
            self.navigator.goToPose(self.best_goal)
            # Add code to send the goal position to the robot's control system
        else:
            self.get_logger().warn("No best goal found!")

def main(args=None):
    rclpy.init(args=args)
    goal_position_planner = Explorer()
    try:
        rclpy.spin(goal_position_planner)
    except SystemExit:
        pass
    goal_position_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
