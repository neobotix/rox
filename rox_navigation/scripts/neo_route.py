#! /usr/bin/env python3
# Copyright 2025 Open Navigation LLC
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

from geometry_msgs.msg import Point, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, RunningTask, TaskResult
from nav2_simple_commander.utils import euler_to_quaternion
import rclpy
from std_msgs.msg import Header
from typing import List

"""
Basic navigation demo to using the route server.
"""


def toPoseStamped(pt: Point, header: Header) -> PoseStamped:
    pose = PoseStamped()
    pose.pose.position.x = pt.x
    pose.pose.position.y = pt.y
    pose.header = header
    return pose

def main() -> None:
    rclpy.init()

    node = rclpy.create_node('neo_route_node')

    node.declare_parameter('initial_pose.x', 0.0)
    node.declare_parameter('initial_pose.y', 0.0)
    node.declare_parameter('initial_pose.yaw', 0.0)

    initial_pose_x = node.get_parameter('initial_pose.x').value
    initial_pose_y = node.get_parameter('initial_pose.y').value
    initial_pose_yaw = node.get_parameter('initial_pose.yaw').value

    # List of node IDs
    node.declare_parameter('node_ids', [0])

    node_ids = node.get_parameter('node_ids').value

    node.destroy_node()

    navigator = BasicNavigator()

    # Set robot initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = initial_pose_x
    initial_pose.pose.position.y = initial_pose_y
    initial_pose.pose.orientation = euler_to_quaternion(0.0, 0.0, initial_pose_yaw)
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active('bt_navigator', 'robot_localization')

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Validate the node_ids list
    if not node_ids or len(node_ids) < 2:
        print('Error: node_ids must contain at least 2 nodes')
        navigator.destroy_node()
        rclpy.shutdown()
        return

    print(f'Navigating through {len(node_ids)} nodes: {node_ids}')

    # Navigate through each pair of consecutive node IDs
    task_canceled = False
    full_task_success = True
    
    for i in range(len(node_ids) - 1):
        start_node = node_ids[i]
        end_node = node_ids[i + 1]
        
        route_tracking_task = navigator.getAndTrackRoute(start_node, end_node)

        # Note for the route server, we have a special route argument in the API b/c it may be
        # providing feedback messages simultaneously to others (e.g. controller or WPF as below)
        last_feedback = None
        follow_path_task = RunningTask.NONE
        while not navigator.isTaskComplete(task=route_tracking_task):
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback, which contains the route / path if tracking
            feedback = navigator.getFeedback(task=route_tracking_task)
            while feedback is not None:
                if not last_feedback or \
                    (feedback.last_node_id != last_feedback.last_node_id or
                        feedback.next_node_id != last_feedback.next_node_id):
                    print('Passed node ' + str(feedback.last_node_id) +
                          ' to next node ' + str(feedback.next_node_id) +
                          ' along edge ' + str(feedback.current_edge_id) + '.')

                last_feedback = feedback

                if feedback.rerouted:  # or follow_path_task == RunningTask.None
                    # Follow the path from the route server using the controller server
                    print('Passing new route to controller!')
                    follow_path_task = navigator.followPath(feedback.path)

                    # May instead use the waypoint follower
                    # (or nav through poses) and use the route's sparse nodes!
                    # print("Passing route to waypoint follower!")
                    # nodes =
                    # [toPoseStamped(x.position, feedback.route.header) for x in feedback.route.nodes]
                    # navigator.followWaypoints(nodes)
                    # Or navigator.navigateThroughPoses(nodes)
                    # Consider sending only the first few and iterating

                feedback = navigator.getFeedback(task=route_tracking_task)

            # Check if followPath or WPF task is done (or failed),
            # will cancel all current tasks, including route
            if navigator.isTaskComplete(task=follow_path_task):
                print('Controller or waypoint follower server completed its task!')
                navigator.cancelTask()
                task_canceled = True

        # Route server will return completed status before the controller / WPF server
        # so wait for the actual robot task processing server to complete
        while not navigator.isTaskComplete(task=follow_path_task) and not task_canceled:
            pass

        # Check result for this segment
        segment_result = navigator.getResult()
        if segment_result == TaskResult.SUCCEEDED:
            print(f'Nodes {i+1} completed successfully')
        elif segment_result == TaskResult.CANCELED:
            print(f'Nodes {i+1} was canceled, stopping route')
            full_task_success = False
            break
        elif segment_result == TaskResult.FAILED:
            print(f'Nodes {i+1} failed, stopping route')
            full_task_success = False
            break

    # Final result
    if full_task_success and not task_canceled:
        print('All nodes were successfully traversed!')
    elif task_canceled:
        print('Route navigation was canceled!')
    else:
        print('Route navigation failed!')

    while rclpy.ok():
        rclpy.spin_once(navigator, timeout_sec=0.5)

    navigator.destroy_node()
    rclpy.shutdown()

    # navigator.lifecycleShutdown()
    # exit(0)

if __name__ == '__main__':
    main()
