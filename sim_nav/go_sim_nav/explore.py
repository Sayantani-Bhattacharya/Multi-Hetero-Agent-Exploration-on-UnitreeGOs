import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
# nav2_simple_commander
import time

class Explore(Node):
    """Autonomous navigation."""

    def __init__(self):
        super().__init__('explore')
        self.get_logger().info('Exploring the navigation stack init')
        self.map = None

        #  to get the current map.
        self.map_sub = self.create_subscription(OccupancyGrid, "map", self.map_callback, qos_profile=10)

        self.targetPose = PoseStamped()
        self.targetPose.header.frame_id = "map"
        self.currentPose = PoseStamped()
        self.currentPose.header.frame_id = "map"

        self.posesub = self.create_subscription(PoseWithCovarianceStamped, "pose", self.pose_callback, qos_profile=10)
        self.goalposepub = self.create_publisher(PoseStamped, "goal_pose", 10)

        self.timer = self.create_timer(1.0, self.timer_callback)


    def map_callback(self, msg:OccupancyGrid):
        """
        Get the map explored by nubot and convert it to a grid format 
        that can be used by the exploration algo.
        """
        self.map = msg

        # Map information
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_load_time = msg.info.map_load_time

        self.grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        # self.get_logger().info(f"Map received: shape={self.grid.shape}")
        # self.get_logger().info(f"Map received: height={self.map_height} widht{self.map_width} resolution={self.map_resolution}")
        # self.get_logger().info(f"Free cells: {np.sum(self.grid == 0)}, Occupied cells: {np.sum(self.grid == 100)}, Unknown cells: {np.sum(self.grid == -1)}")


    def pose_callback(self,msg:PoseWithCovarianceStamped):
        """
        Get the current pose of the nubot.
        """
        self.currentPose.header.stamp = self.get_clock().now().to_msg()
        self.currentPose.pose = msg.pose.pose
        # self.get_logger().info(f'Current pose received {self.currentPose.pose}')        

    def detect_frontiers(self, occupancy_grid):
        '''
        Occupancy grid is used to detect all the possible frontiers in the current map.

        Occupancy Grids:
            # open: having an occupancy probability < prior probability       ----> 0
            # unknown: having an occupancy probability = prior probability    ----> -1
            # occupied: having an occupancy probability > prior probability   ----> 100
        '''
        frontiers = []
        for r in range(self.map_height):
            for c in range(self.map_width):
                # Check if the current cell is free space
                if occupancy_grid[r, c] == 0:
                    # Check if it is adjacent to at least one unknown cell
                    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                        nr, nc = r + dr, c + dc  # Neighbor coordinates
                        if 0 <= nr < self.map_height and 0 <= nc < self.map_width:  # Check bounds
                            if occupancy_grid[nr, nc] == -1:
                                # Add the current cell as a frontier
                                frontiers.append((r, c))
                                break        
        return frontiers
    

    def frontier_exp_algo(self,occupancy_grid):
        '''
        function to identify unexplored areas (frontiers) and send the next navigation goal to the nubot.
        '''
        # Evedence Grids:
            # open: having an occupancy probability < prior probability       ----> 0
            # unknown: having an occupancy probability = prior probability    ----> -1
            # occupied: having an occupancy probability > prior probability   ----> 100

        frontiers = self.detect_frontiers(occupancy_grid=occupancy_grid)

        # Find the closest frontier
        pose = []
        pose.append(self.currentPose.pose.position.x)
        pose.append(self.currentPose.pose.position.y)

        # # debug
        # self.get_logger().info('DEBUGG 1') 
        # for r,c in frontiers:
        #     if (r>=self.map_height or r<=0 ):
        #         self.get_logger().info(f'Wrong value of r : {r}.') 
        #     elif (c>= self.map_width or c<=0):
        #         self.get_logger().info(f'Wrong value of C : {c}.') 


        # Convert all frontier grid cells to world coordinates first
        frontiers_world = [
            (
                self.map_origin.position.x + (c * self.map_resolution),
                self.map_origin.position.y + (r * self.map_resolution),
            )
            for r, c in frontiers
        ]
        x_max = self.map_origin.position.x + (self.map_resolution * self.map_width)
        y_max = self.map_origin.position.y + (self.map_resolution * self.map_height)
        x_min = self.map_origin.position.x 
        y_min = self.map_origin.position.y 

    # # debug
        # self.get_logger().info('DEBUGG 2')         
        # for r,c in frontiers_world:
        #     if (r>=x_max or r<=x_min):
        #         self.get_logger().info(f'Wrong value of r : {r}.') 
        #     elif (c>=y_max or c<=y_min):
        #         self.get_logger().info(f'Wrong value of C : {c}.') 

        # Calculate distances to the target point (current robot position in world coordinates)
        distances = [
            np.sqrt((world_r - self.currentPose.pose.position.x)**2 +
                    (world_c - self.currentPose.pose.position.y)**2)
            for world_r, world_c in frontiers_world
        ]
        # Setting a min distance to ensure robot moves.
        # You can increase this value to explore faster, but too high will leave sections unexplored [Works well with 0.6]
        threshold_dist = 0.6
        filtered_distances = [dist for dist in distances if dist > threshold_dist]
        if filtered_distances:
            # Find the index of the minimum distance from the filtered list
            min_index = np.argmin(filtered_distances)
            nextPose = frontiers_world[distances.index(filtered_distances[min_index])]
        else:
            self.get_logger().info('No valid frontiers found.') 


        # # Find the index of the closest frontier in world coordinates
        # min_index = np.argmin(distances)
        # nextPose = frontiers_world[min_index]


        self.get_logger().info(f'THE CURRENT POSE: {pose}.') 

        self.get_logger().info(f'THE NEXT POSE: {nextPose}.') 

        # Calculate the centroid of the chosen frontier cluster as the next target. = nextPose
        return nextPose

    def timer_callback(self):
        '''
        The function will explore the map till a tollerance limit and once it reached the condition, 
        it will stop publishing poses to robot and save the final map in the /maps directory.
        '''

        if self.map is None:
            self.get_logger().info('Waiting for map...')
            return
        else:
            unknown_cells_left = np.sum(self.grid == -1)
            tollerance = 0.50 * (self.map_width * self.map_height)
            x_max = self.map_origin.position.x + (self.map_resolution * self.map_width)
            y_max = self.map_origin.position.y + (self.map_resolution * self.map_height)
            x_min = self.map_origin.position.x 
            y_min = self.map_origin.position.y 
            

            if (unknown_cells_left <= tollerance):
               self.get_logger().info('Congratulations, The map is fully explored!.') 
               self.save_final_map(self.map)
            else:
                self.get_logger().info('Map is being explored.') 
                nextPose = self.frontier_exp_algo(self.grid)

                self.targetPose.header.stamp = self.get_clock().now().to_msg()
                # self.targetPose.pose = self.currentPose.pose 
                # self.targetPose.pose.position.x = (self.currentPose.pose.position.x + 0.1)  
                # self.targetPose.pose.position.y = (self.currentPose.pose.position.y + 0.2)  


                self.targetPose.pose.position.x =  max(min(nextPose[0],x_max),x_min)
                self.targetPose.pose.position.y = max(min(nextPose[1],y_max),y_min)
                self.get_logger().info(f'MAP BOUNDS: x: {x_min} MAX {x_max} AND YMIN: { y_min} AND MAX : {y_max}' )

                self.get_logger().info(f'Goal Pose: {self.targetPose.pose.position.x}, {self.targetPose.pose.position.y}')

                self.goalposepub.publish(self.targetPose)
                # for the robot to get achieve the pose given
                # time.sleep(2)

    def save_final_map(self,map):
        self.get_logger().info('Saving the final generated map.')
        # Used the following command: can be automated here later
        # ros2 run nav2_map_server map_saver_cli -f <map_file_name>


def main(args=None):
    rclpy.init(args=args)
    node = Explore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(args=sys.argv)