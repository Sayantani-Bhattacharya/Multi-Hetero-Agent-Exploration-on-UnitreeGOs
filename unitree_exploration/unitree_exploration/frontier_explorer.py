import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import String
from std_srvs.srv import Empty
from unitree_nav_interfaces.srv import NavToPose
import tf2_ros
from tf2_ros import TransformListener, Buffer
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import cv2
from enum import Enum
import math

class State(Enum):
    IDLE = 0
    EXPLORING = 1

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        self.map_metadata = None
        self.map_data = None
        self.frontiers = []
        self.current_frontier = None
        self.current_state = State.IDLE

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.goal_completed_subscription = self.create_subscription(
            String,
            '/goal_completed',
            self.goal_completed_callback,
            10)

        self.goal_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/frontier_markers', 10)

        self.unitree_nav_to_pose_client = self.create_client(NavToPose, '/unitree_nav_to_pose')
        self.cancel_nav_client = self.create_client(Empty, '/unitree_cancel_nav')

        self.explore_srv = self.create_service(Empty, '/explore', self.explore_callback)

        # Create a transform listener and buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def explore_callback(self, request, response):
        # Check if already exploring, if not start exploring
        if self.current_state == State.IDLE:
            self.current_state = State.EXPLORING
            self.get_logger().info("Exploration Started!")
        else:
            self.get_logger().info("Already Exploring")
        return response

    def unitree_nav_to_pose(self, x, y, theta):
        request = NavToPose.Request()
        request.x = x
        request.y = y
        request.theta = theta
        future = self.unitree_nav_to_pose_client.call_async(request)

    def goal_completed_callback(self, msg):
        if self.current_state == State.EXPLORING:
            # Remove the current frontier from the list
            self.frontiers.remove(self.current_frontier)

            # If there are more frontiers, navigate to the next one
            if self.frontiers:
                self.current_frontier = self.frontiers[0]
                x = self.current_frontier[0] * self.map_metadata.resolution + self.map_metadata.origin.position.x
                y = self.current_frontier[1] * self.map_metadata.resolution + self.map_metadata.origin.position.y
                theta = self.current_frontier[2]  # Set theta as needed
                self.unitree_nav_to_pose(x, y, theta)
            else:
                # No more frontiers, exploration complete
                self.current_state = State.IDLE

    def cancel_navigation(self):
        future = self.cancel_nav_client.call_async()

    def map_callback(self, msg):
        self.map_metadata = msg.info
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        if self.current_state == State.EXPLORING:
            self.find_frontiers()

    def find_frontiers(self):
        # Reset frontiers
        self.frontiers = []

        # Dilate occupied cells
        kernel_size = 16
        dilation_kernel = np.ones((kernel_size, kernel_size), np.uint8)
        dilated_map = cv2.dilate((self.map_data == 0).astype(np.uint8), dilation_kernel)

        distance_threshold = 2.0

        if np.any(dilated_map == 1):
            self.get_logger().info("Not all zeros")

        else:
            self.get_logger().info("All zeros")

        # Find frontiers
        # for i in range(self.map_metadata.height):
        #     for j in range(self.map_metadata.width):
        #         if self.map_data[i][j] == 0:
        #             if np.sum(dilated_map[i - 1:i + 2, j - 1:j + 2]) > 0:
        #                 self.frontiers.append((j, i))
        #                 # self.frontiers.append((i, j))
        robot_position = self.get_robot_position()

        # Find frontiers
        for j in range(self.map_metadata.height):
            for i in range(self.map_metadata.width):
                if self.map_data[j][i] == 0:  # If the current cell is free
                    unexplored_directions = []  # List to store directions towards unexplored cells
                    for dx in [-1, 0, 1]:  # Check neighboring cells
                        for dy in [-1, 0, 1]:
                            ni, nj = i + dx, j + dy
                            if ni >= 0 and ni < self.map_metadata.height and nj >= 0 and nj < self.map_metadata.width:
                                if self.map_data[nj][ni] == -1:  # If neighboring cell is unexplored  
                                    if dilated_map[ni][nj] == 0:  # If neighboring cell is unexplored after dilation
                                        theta = math.atan2(nj - j, ni - i)  # Calculate angle towards unexplored cell
                                        unexplored_directions.append(theta)

                    if unexplored_directions:  # If unexplored cells found
                        if ((i * self.map_metadata.resolution + self.map_metadata.origin.position.x - robot_position.x)**2 
                            + (j * self.map_metadata.resolution + self.map_metadata.origin.position.y - robot_position.y)**2)**0.5 > distance_threshold:
                            
                            # Calculate average direction towards unexplored cells
                            avg_theta = sum(unexplored_directions) / len(unexplored_directions)
                            self.frontiers.append((i * self.map_metadata.resolution + self.map_metadata.origin.position.x, # coordinates in /map frame
                                                    j * self.map_metadata.resolution + self.map_metadata.origin.position.y, 
                                                    avg_theta))
        
        # Sort frontiers based on distance from robot's current position (closest first)
        if robot_position is not None:
            self.frontiers.sort(key=lambda x: np.sqrt((x[0] - robot_position.x) ** 2 +
                                                      (x[1] - robot_position.y) ** 2))

        # Publish marker array
        self.publish_frontier_markers()

        # self.get_logger().info("Finding frontiers")
        # If there are frontiers available, navigate to the closest one
        if self.frontiers:
            # self.get_logger().info("We have frontiers")
            self.current_frontier = self.frontiers[0]
            x = self.current_frontier[0] 
            y = self.current_frontier[1] 
            theta = self.current_frontier[2]  
            self.unitree_nav_to_pose(x, y, theta)
        else:
            # No frontiers found, exploration complete
            self.current_state = State.IDLE
            self.get_logger().info("No more frontiers")

    def publish_frontier_markers(self):
        marker_array = MarkerArray()

        # Create marker for each frontier
        for i, frontier in enumerate(self.frontiers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = frontier[0] 
            marker.pose.position.y = frontier[1] 
            marker.pose.orientation.w = np.cos(frontier[2] / 2) # yaw to quaternion
            marker.pose.orientation.z = np.sin(frontier[2] / 2)
            marker.scale.x = 0.5  # Arrow dimensions
            marker.scale.y = 0.1  # Arrow dimensions
            marker.scale.z = 0.1  # Arrow dimensions
            if i == 0:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0  # Blue color
                marker.color.a = 1.0  # Fully opaque
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0  # Blue color
                marker.color.a = 0.5  # Fully opaque
            marker.id = i
            marker_array.markers.append(marker)

        # Publish marker array
        self.marker_publisher.publish(marker_array)

    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return transform.transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().error("Failed to lookup transform from 'map' to 'base_footprint'")
            return None

    def main_loop(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    frontier_explorer = FrontierExplorer()
    frontier_explorer.main_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
