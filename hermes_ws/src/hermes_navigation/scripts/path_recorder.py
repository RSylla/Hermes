from navigation import GpsSubscriber
from hermes_interfaces.msg import GpsFixed
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import pymap3d as pm
import time

def main():
    navigator = BasicNavigator()
    gps_data = GpsSubscriber()

    LAT0, LON0, ALT0 = 58.3428685594, 25.5692475361, 91.357

    goal_poses = []
    print("Recording poses...")
    while not KeyboardInterrupt:
        
        lat, lon, alt = gps_data.lat, gps_data.lon, gps_data.alt
        x, y, z = pm.geodetic2enu(lat, lon, alt, LAT0, LON0, ALT0)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = z
        goal_pose.pose.orientation.w = 0.0    
        goal_poses.append(goal_pose)
        time.sleep(0.1)
    return goal_poses


if __name__ == "__main__":    
    goal_poses = main()
    path = open("path.txt")
    with path as p:
        p.write(goal_poses)
    print("path saved")
