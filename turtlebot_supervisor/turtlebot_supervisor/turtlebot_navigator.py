#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from enum import Enum
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.duration import Duration


class NavigationResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class BasicNavigator(Node):

    def __init__(self):
        super().__init__(node_name='basic_navigator')
        self._handle_parameters()
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.initial_pose_received = True
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, self.namespace + 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(
            self, FollowWaypoints, self.namespace + 'FollowWaypoints')
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose,
                                                        self.namespace + 'compute_path_to_pose')
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              self.namespace + 'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      self.namespace + 'initialpose',
                                                      10)
        self.current_goal_pub = self.create_publisher(PoseStamped,
                                                      self.namespace + 'current_goal',
                                                      10)
        self.change_maps_srv = self.create_client(
            LoadMap, '/map_server/load_map')
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.get_costmap_global_srv = self.create_client(
            GetCostmap, '/global_costmap/get_costmap')
        self.get_costmap_local_srv = self.create_client(
            GetCostmap, '/local_costmap/get_costmap')
            
    def _handle_parameters(self):
        self._declare_default_parameters()
        self.namespace = self.get_parameter('namespace').value

    def _declare_default_parameters(self):
        self.declare_parameter('namespace', '')

    def setInitialPose(self, initial_pose):
        """Set the initial pose to the localization system."""
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

    def followWaypoints(self, poses):
        """Send a `FollowWaypoints` action request."""
        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info(f"{self.namespace}/FollowWaypoints action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.info('Following ' + str(len(goal_msg.poses)) + ' goals.' + '...')
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg,
                                                                        self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Following ' + str(len(poses)) +
                       ' waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        """Cancel pending navigation request of any type."""
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        """Check if the navigation request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(
            self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(
                    'Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Goal succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return NavigationResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return NavigationResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return NavigationResult.CANCELED
        else:
            return NavigationResult.UNKNOWN

    def waitUntilNav2Active(self):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return


    def changeMap(self, map_filepath):
        """Change the current static map in the map server."""
        while not self.change_maps_srv.wait_for_service(timeout_sec=1.0):
            self.info('change map service not available, waiting...')
        req = LoadMap.Request()
        req.map_url = map_filepath
        future = self.change_maps_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        status = future.result().result
        if status != LoadMap.Response().RESULT_SUCCESS:
            self.error('Change map request failed!')
        else:
            self.info('Change map request was successful!')
        return

    def clearAllCostmaps(self):
        """Clear all costmaps."""
        self.clearLocalCostmap()
        self.clearGlobalCostmap()
        return

    def clearLocalCostmap(self):
        """Clear local costmap."""
        while not self.clear_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear local costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def clearGlobalCostmap(self):
        """Clear global costmap."""
        while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear global costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def getGlobalCostmap(self):
        """Get the global costmap."""
        while not self.get_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get global costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def getLocalCostmap(self):
        """Get the local costmap."""
        while not self.get_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get local costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def lifecycleStartup(self):
        """Startup nav2 lifecycle system."""
        self.info('Starting up lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info('Starting up ' + srv_name)
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(srv_name + ' service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().STARTUP
                future = mgr_client.call_async(req)

                # starting up requires a full map->odom->base_link TF tree
                # so if we're not successful, try forwarding the initial pose
                while True:
                    rclpy.spin_until_future_complete(
                        self, future, timeout_sec=0.10)
                    if not future:
                        self._waitForInitialPose()
                    else:
                        break
        self.info('Nav2 is ready for use!')
        return

    def lifecycleShutdown(self):
        """Shutdown nav2 lifecycle system."""
        self.info('Shutting down lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info('Shutting down ' + srv_name)
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(srv_name + ' service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = self.namespace + node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

def startGoingAround():

    navigator = BasicNavigator()

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()
    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()
    while True:
        
        # set our demo's goal poses to follow
        goal_poses = []
        goal_pose1 = PoseStamped()
        goal_pose2 = PoseStamped()
        goal_pose3 = PoseStamped()
        goal_pose4 = PoseStamped()
        if navigator.namespace == '/robot2/':
            goal_pose1.pose.position.x = -0.5
            goal_pose1.pose.position.y = -0.5
            goal_pose1.pose.orientation.w = 0.9238795
            goal_pose1.pose.orientation.z = 0.3826834

            goal_pose2.pose.position.x = -0.5
            goal_pose2.pose.position.y = 0.5
            goal_pose2.pose.orientation.w = 0.3826834
            goal_pose2.pose.orientation.z = 0.9238795

            goal_pose3.pose.position.x = -1.5
            goal_pose3.pose.position.y = 0.5
            goal_pose3.pose.orientation.w = -0.3826834
            goal_pose3.pose.orientation.z = 0.9238795

            goal_pose4.pose.position.x = -1.5
            goal_pose4.pose.position.y = -0.5
            goal_pose4.pose.orientation.w = -0.9238795
            goal_pose4.pose.orientation.z = 0.3826834

        elif navigator.namespace == '/robot1/':
            goal_pose1.pose.position.x = 0.5
            goal_pose1.pose.position.y = -1.5
            goal_pose1.pose.orientation.w = 0.9238795
            goal_pose1.pose.orientation.z = 0.3826834

            goal_pose2.pose.position.x = 0.5
            goal_pose2.pose.position.y = -0.5
            goal_pose2.pose.orientation.w = 0.3826834
            goal_pose2.pose.orientation.z = 0.9238795

            goal_pose3.pose.position.x = -0.5
            goal_pose3.pose.position.y = -0.5
            goal_pose3.pose.orientation.w = -0.3826834
            goal_pose3.pose.orientation.z = 0.9238795

            goal_pose4.pose.position.x = -0.5
            goal_pose4.pose.position.y = -1.5
            goal_pose4.pose.orientation.w = -0.9238795
            goal_pose4.pose.orientation.z = 0.3826834


        goal_pose1.header.frame_id = 'map'
        goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
        
        goal_poses.append(goal_pose1)

        # additional goals can be appended
        goal_pose2.header.frame_id = 'map'
        goal_pose2.header.stamp = navigator.get_clock().now().to_msg()

        goal_poses.append(goal_pose2)
        goal_pose3.header.frame_id = 'map'
        goal_pose3.header.stamp = navigator.get_clock().now().to_msg()

        goal_poses.append(goal_pose3)
        goal_pose4.header.frame_id = 'map'
        goal_pose4.header.stamp = navigator.get_clock().now().to_msg()

        goal_poses.append(goal_pose4)

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose1)

        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(goal_poses)

        i = 0
        while not navigator.isNavComplete(): #isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                      str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
                now = navigator.get_clock().now()
                navigator.current_goal_pub.publish(goal_poses[feedback.current_waypoint])

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=600.0):
                    navigator.cancelNav() #cancelTask()

        # Do something depending on the return code
        result = navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
            break
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
            break
        else:
            print('Goal has an invalid return status!')
            break

    navigator.lifecycleShutdown()

    exit(0)


def main():
    rclpy.init()
    startGoingAround()




if __name__ == '__main__':
    main()



    # goal_poses = []
    # goal_pose1 = PoseStamped()
    # goal_pose1.header.frame_id = 'map'
    # goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose1.pose.position.x = 1.5
    # goal_pose1.pose.position.y = 0.55
    # goal_pose1.pose.orientation.w = 0.707
    # goal_pose1.pose.orientation.z = 0.707
    # goal_poses.append(goal_pose1)

    # # additional goals can be appended
    # goal_pose2 = PoseStamped()
    # goal_pose2.header.frame_id = 'map'
    # goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose2.pose.position.x = -1.5
    # goal_pose2.pose.position.y = -0.55
    # goal_pose2.pose.orientation.w = 0.707
    # goal_pose2.pose.orientation.z = 0.707
    # goal_poses.append(goal_pose2)
    # goal_pose3 = PoseStamped()
    # goal_pose3.header.frame_id = 'map'
    # goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose3.pose.position.x = -1.5
    # goal_pose3.pose.position.y = 0.55
    # goal_pose3.pose.orientation.w = 0.707
    # goal_pose3.pose.orientation.z = 0.707
    # goal_poses.append(goal_pose3)

    # # sanity check a valid path exists
    # # path = navigator.getPath(initial_pose, goal_pose1)

    # nav_start = navigator.get_clock().now()
    # navigator.followWaypoints(goal_poses)

    # i = 0
    # while not navigator.isNavComplete(): #isTaskComplete():
    #     ################################################
    #     #
    #     # Implement some code here for your application!
    #     #
    #     ################################################

    #     # Do something with the feedback
    #     i = i + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and i % 5 == 0:
    #         print('Executing current waypoint: ' +
    #               str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
    #         now = navigator.get_clock().now()

    #         # Some navigation timeout to demo cancellation
    #         if now - nav_start > Duration(seconds=600.0):
    #             navigator.cancelNav() #cancelTask()

    #         # # Some follow waypoints request change to demo preemption
    #         # if now - nav_start > Duration(seconds=35.0):
    #         #     goal_pose4 = PoseStamped()
    #         #     goal_pose4.header.frame_id = 'map'
    #         #     goal_pose4.header.stamp = now.to_msg()
    #         #     goal_pose4.pose.position.x = -5.0
    #         #     goal_pose4.pose.position.y = -4.75
    #         #     goal_pose4.pose.orientation.w = 0.707
    #         #     goal_pose4.pose.orientation.z = 0.707
    #         #     goal_poses = [goal_pose4]
    #         #     nav_start = now
    #         #     navigator.followWaypoints(goal_poses)

    # # Do something depending on the return code
    # result = navigator.getResult()
    # if result == NavigationResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == NavigationResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == NavigationResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')