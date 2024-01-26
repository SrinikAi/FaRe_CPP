

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class MapUpdater:
    def __init__(self):
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.local_costmap_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.local_costmap_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.updated_map = None
        self.map_resolution = None
        self.map_width = None
        self.local_costmap = None
        self.robot_pose = None

    def map_callback(self, msg):
        # Store the received map
        self.updated_map = msg
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width

    def scan_callback(self, msg):
        if self.updated_map is None or self.map_resolution is None or self.map_width is None:
            return
        
        # Process laser scan data and update the map
        self.update_map_with_scan_data(msg)

    def local_costmap_callback(self, msg):
        self.local_costmap = msg

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose
      
    def update_map_with_scan_data(self, laser_scan_msg):
        if self.local_costmap is None or self.robot_pose is None:
            return
        
        # Convert laser scan angles to indices in the map
        angle_min = laser_scan_msg.angle_min
        angle_increment = laser_scan_msg.angle_increment
        ranges = laser_scan_msg.ranges

        angle_indices = np.arange(angle_min, angle_min + len(ranges) * angle_increment, angle_increment)
        x_indices = np.floor((ranges / self.map_resolution) * np.cos(angle_indices) + self.robot_pose.position.x / self.map_resolution)
        y_indices = np.floor((ranges / self.map_resolution) * np.sin(angle_indices) + self.robot_pose.position.y / self.map_resolution)

        # Update map based on laser scan data and local costmap
        for x, y in zip(x_indices, y_indices):
            if 0 <= x < self.map_width and 0 <= y < self.map_width:
                # Update map cells if they are free in local costmap and unoccupied in the map
                local_costmap_value = self.local_costmap.data[int(y) * self.map_width + int(x)]
                if local_costmap_value == 0 and self.updated_map.data[int(y) * self.map_width + int(x)] == -1:
                    self.updated_map.data[int(y) * self.map_width + int(x)] = 0

        # Publish the updated map
        self.map_pub.publish(self.updated_map)
        
       
if __name__ == '__main__':
    rospy.init_node('map_updater')
    map_updater = MapUpdater()
    rospy.spin()



