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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from hermes_interfaces.msg import GpsFixed
import pymap3d as pm

#Basestation cordinates
LAT0, LON0, ALT0 = 58.3428685594, 25.5692475361, 91.357


class GpsSubscriber(rclpy.node.Node):

    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            GpsFixed,
            'gps_data', #might require '/'.
            self.listener_callback, 10)
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.course = 0
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longtitude
        self.alt = msg.altitude
        self.course = msg.course_over_ground_deg
        


"""
Basic navigation demo to go to pose.
"""


def main():
    rclpy.init()
    gps_data = GpsSubscriber()
    navigator = BasicNavigator()

    # Set our demo's initial pose
    lat, lon, alt = gps_data.lat, gps_data.lon, gps_data.alt
    x, y, z = pm.geodetic2enu(lat, lon, alt, LAT0, LON0, ALT0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = x
    initial_pose.pose.position.y = y
    initial_pose.pose.orientation.z = z
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    navigator.changeMap('/path/to/map.yaml') #Our saved map

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()
    goal_x_y = [x, y]
    print(goal_x_y)
    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = float(goal_x_y[0])
    goal_pose.pose.position.y = float(goal_x_y[1])
    goal_pose.pose.orientation.w = 0.0

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        lat, lon, alt = gps_data.lat, gps_data.lon, gps_data.alt
        x, y, z = pm.geodetic2enu(lat, lon, alt, LAT0, LON0, ALT0)
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = -3.0
                navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()