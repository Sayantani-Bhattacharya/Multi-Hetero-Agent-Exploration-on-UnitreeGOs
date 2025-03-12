// A node that subscribes to /map data and publishes a ed nav goal for the robot to explore the map.
// The robot will explore the map by moving to the nearest frontier point.
// The robot will stop exploring when the map is fully explored or with a user service call.
// Increase the footprint of the robot in the costmap to avoid collision with the map.: this is done by setting the radius of the robot to 0.3m in the costmap_common_params.yaml file. -----> see

// Services:
// 1. /explore_start : Service to start exploring the map.
// 2. /explore_stop : Service to stop the robot from exploring the map.

// Subscribers:
// 1. /map : Subscribes to the map data.
// 2. /goal_reached : Subscribes to the goal pose for the robot to move to.
// 3. lookup transform

// Algorithm:
// seach frontiers, find the nearest frontier, publish the goal pose, move to the goal pose, repeat.

// Publishers:
// frontier_markers : Publishes the frontier points on the map.

#include <exception>
#include <vector>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// from nav_msgs.msg import OccupancyGrid
#include "go2_exploration_interfaces/srv/empty.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "unitree_go2_nav_interfaces/msg/nav_to_pose.hpp"
#include "go2_exploration_interfaces/srv/empty.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// State machine
enum class State
{
    IDLE,
    // MANUAL_NAV_MODE,
    // AUTO_NAV_MODE,
    START,
    MOVING,
    REACHED_GOAL,
    E_STOP,
};

using namespace std::chrono_literals;

class Explore : public rclcpp::Node
{
    public:
     Explore():Node("explore")
     { 
        // Services
        nav_start_trigger = create_service<go2_exploration_interfaces::srv::Empty>("explore_start", std::bind(&Explore::nav_start_trigger_callback, this, std::placeholders::_1, std::placeholders::_2));
        nav_stop_trigger = create_service<go2_exploration_interfaces::srv::Empty>("explore_stop", std::bind(&Explore::nav_stop_trigger_callback, this, std::placeholders::_1, std::placeholders::_2));    

        // Subscribers
        // /goal_reached -- if needed.
        map_sub = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&Explore::map_sub_callback, this, std::placeholders::_1));
        
        // Transform listener
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Publishers
        goal_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10); 
        frontier_marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>("/frontier_markers", 10); 
        goal_marker_pub = create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);

        // Timer
        mTimer = this->create_wall_timer(100ms, std::bind(&Explore::timer_callback, this));    
     }     

    private:
      rclcpp::TimerBase::SharedPtr mTimer;
      State mRobotState = State::IDLE;
      float mDiagonalTollerance = 0.3; // this is for goal reached.
      std::pair<int,int> mCurrGoalPose = {-100, -100};
      std::vector<std::pair<int, int>> mFrontiers;
      int mMinExploreDistance = 0.7; // 8: out of bounds for nav pkg....no check again


      // Map data
      nav_msgs::msg::OccupancyGrid mMap;
      float mMapWidth;
      float mMapHeight;
      float mMapResolution;
      geometry_msgs::msg::Pose mMapOrigin;
      rclcpp::Time mMapLoadTime;
      std::vector<std::vector<int>> mMapGrid;

      // TF
      std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
      std::unique_ptr<tf2_ros::Buffer> tf_buffer;
      std::string target_frame_;
      std::vector<float> mCurrPose_wrt_map = {0, 0, 0}; // x, y, z
    
      rclcpp::Service<go2_exploration_interfaces::srv::Empty>::SharedPtr nav_start_trigger;
      rclcpp::Service<go2_exploration_interfaces::srv::Empty>::SharedPtr nav_stop_trigger;
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr curr_pose_sub;
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_marker_pub;
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub;


      void nav_start_trigger_callback(const std::shared_ptr<go2_exploration_interfaces::srv::Empty::Request> request, std::shared_ptr<go2_exploration_interfaces::srv::Empty::Response> response)
      {
        if(mRobotState == State::START)
        {
          RCLCPP_INFO(this->get_logger(), "Already exploring.");
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "Starting Exploration.");
          mRobotState = State::START;
        }
        response->r = true;
      } 

      void nav_stop_trigger_callback(const std::shared_ptr<go2_exploration_interfaces::srv::Empty::Request> request, std::shared_ptr<go2_exploration_interfaces::srv::Empty::Response> response)
      {
        RCLCPP_INFO(this->get_logger(), "Stopping Exploration.");
        mRobotState = State::E_STOP;
        response->r = true;
      } 

      void map_sub_callback(const nav_msgs::msg::OccupancyGrid msg)
      {
        /*
        Get the map explored by GO2 and convert it to a grid format 
        that can be used by the exploration algo.
        */

        mMap = msg;
        std::set<int> unique_values;

        // Map information
        mMapWidth = msg.info.width;
        mMapHeight = msg.info.height;
        mMapResolution = msg.info.resolution;
        mMapOrigin = msg.info.origin;
        mMapLoadTime = msg.info.map_load_time;
        mMapGrid.clear();
        mMapGrid.resize(mMapHeight, std::vector<int>(mMapWidth, 0));
        for (int i = 0; i < mMapHeight; i++)
        {
            for (int j = 0; j < mMapWidth; j++)
            {
                int index = j + i * mMapWidth;  
                if (index >= 0 && index < msg.data.size()) {
                    mMapGrid[i][j] = msg.data[index];
                    unique_values.insert(msg.data[index]);
                } 
                else 
                {
                    RCLCPP_WARN(this->get_logger(), "Index out of bounds: %d", index);
                }
            }
        }
        // RCLCPP_INFO(this->get_logger(), "Map received: width= %f", mMapWidth);
        // RCLCPP_INFO(this->get_logger(), "Map received: height= %f", mMapHeight);
        // RCLCPP_INFO(this->get_logger(), "Map received: resolution= %f", mMapResolution);
        // RCLCPP_INFO(this->get_logger(), "Map received: origin x= %f", mMapOrigin.position.x);
        // RCLCPP_INFO(this->get_logger(), "Map received: origin y= %f", mMapOrigin.position.y);
        // RCLCPP_INFO(this->get_logger(), "Map received: origin z= %f", mMapOrigin.position.z);      
        // RCLCPP_INFO(this->get_logger(), "Map received: shape= %i", mMapGrid.size());

        // Count occurrences
        int count_100 = 0, count_0 = 0, count_neg1 = 0;
        for (const auto& row : mMapGrid) {
            for (int val : row) {
                if (val == 100) count_100++;
                // else if (val == 0) count_0++;
                else if (val >= 0 && val < 100) count_0++;  // Adjust threshold if necessary
                else if (val == -1) count_neg1++;
                else RCLCPP_WARN(this->get_logger(), "Unexpected map value: %d", val);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Occupied cells: %i, Free cells: %i, Unknown cells: %i", count_100, count_0, count_neg1);

        // Printing unique vals:
        std::string unique_vals_str;
        for (int val : unique_values) 
        {
            unique_vals_str += std::to_string(val) + " ";
        }
        // RCLCPP_INFO(this->get_logger(), "Unique values in map: %s", unique_vals_str.c_str());

        detect_frontiers();
      }

      // void curr_pose_sub_callback(const geometry_msgs::msg::PoseStamped msg)
      // {
      //   mCurrPose = msg;
      //   RCLCPP_INFO(this->get_logger(), "Current pose received: x= %f, y= %f", msg.pose.position.x, msg.pose.position.y);
      // }

      void publish_frontier_markers(std::vector<std::pair<int, int>> frontiers)
      {
        //std::vector<std::pair<int, int>> frontiers
        if (frontiers.empty())
        {
          RCLCPP_INFO(this->get_logger(), "No frontiers to publish.");
          return;
        }
        visualization_msgs::msg::MarkerArray markerArray; 
        int id = 0;
        for (const auto& [fx,fy]: frontiers)
        {
          visualization_msgs::msg::Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = rclcpp::Clock().now();
          marker.ns = "basic_shapes";
          marker.id = id;

          marker.type = visualization_msgs::msg::Marker::ARROW;
          marker.action = visualization_msgs::msg::Marker::ADD;

          marker.pose.position.x = fx*mMapResolution;
          marker.pose.position.y = fy*mMapResolution;
          marker.pose.orientation.z = std::cos(fy / 2);
          marker.pose.orientation.w = std::sin(fy/ 2);
          // Arrow dimention
          marker.scale.x = 0.5;
          marker.scale.y = 0.01;
          marker.scale.z = 0.01;

          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          marker.color.a = 1.0;   // Don't forget to set the alpha!
          markerArray.markers.push_back(marker);
          id++;
        }

        // only if using a MESH_RESOURCE marker type:
        // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        // marker.lifetime = rclcpp::Duration::from_nanoseconds(1000);
        frontier_marker_pub->publish(markerArray);
        return;
      }

      void publish_goal_marker(std::pair<int, int> goalPose)
      {
        // have a default wierd value.
        // if (goalPose.first.empty() || goalPose.second.empty())
        // {
        //   RCLCPP_INFO(this->get_logger(), "Goal Pose not exist.");
        //   return;
        // }        
        visualization_msgs::msg::Marker marker; 
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = goalPose.first*mMapResolution;
        marker.pose.position.y = goalPose.second*mMapResolution;
        marker.pose.orientation.z = std::cos(goalPose.second/ 2);
        marker.pose.orientation.w = std::sin(goalPose.second/ 2);
        // Arrow dimention
        marker.scale.x = 0.45;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;   // Don't forget to set the alpha!
        goal_marker_pub->publish(marker);
        return;
      }

      std::vector<std::pair<int, int>> detect_frontiers()
      {
        /*
        Occupancy grid is used to detect all the possible frontiers in the current map.
        Occupancy Grids:
            # open: having an occupancy probability < prior probability       ----> 0
            # unknown: having an occupancy probability = prior probability    ----> -1
            # occupied: having an occupancy probability > prior probability   ----> 100
        */
        std::vector<std::pair<int, int>> frontiers;
        // RCLCPP_INFO(this->get_logger(), "Map received: width= %f", mMapWidth);
        // RCLCPP_INFO(this->get_logger(), "Map received: height= %f", mMapHeight);
        RCLCPP_INFO(this->get_logger(), "[Explore] Detecting frontier.");
        std::vector<std::vector<bool>> visited(mMapHeight, std::vector<bool>(mMapWidth, false));

        for (int r = 0; r < mMapHeight; ++r) {
            for (int c = 0; c < mMapWidth; ++c) {
                // Check if the current cell is free space : changed the logic to current cell being not occupied and not detected as a frontier before. !!!!!!!!!!!!!!
                if (mMapGrid[r][c] != 100 && !visited[r][c]) { // change this to mMapGrid[r][c] == 0 && later.!!!!!!!!!!
                    // Mark the cell as visited 
                    visited[r][c] = true;

                    // Check if it is adjacent to at least one unknown cell
                    for (const auto& [dr, dc] : std::vector<std::pair<int, int>>{{-1, 0}, {1, 0}, {0, -1}, {0, 1} , {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}) 
                    {
                        int nr = r + dr, nc = c + dc; // Neighbour coordinates
                        if (nr >= 0 && nr < mMapHeight && nc >= 0 && nc < mMapWidth) { // Check bounds
                            // if the neighbour cell is unknown.
                            if (mMapGrid[nr][nc] == -1) {
                                // Add the current cell as a frontier
                                // Convert to world coordinates.
                                auto val = map_to_world(r, c);
                                // fx = val.first;
                                // fy = val.second;
                                frontiers.emplace_back(val.first, val.second);
                                break;
                            }
                        }
                    }
                }
            }
        }
        mFrontiers = frontiers;
        RCLCPP_INFO(this->get_logger(), "[Explore] Detection algo ended.");
        RCLCPP_INFO(this->get_logger(), "[Explore] Frontiers detected: %i", frontiers.size());
        return frontiers;        
      }

      // std::vector<std::pair<int, int>> detect_frontiers()
      // {
      //     /*
      //     Occupancy grid is used to detect all the possible frontiers in the current map.
      //     Occupancy Grids:
      //         # open: having an occupancy probability < prior probability       ----> 0
      //         # unknown: having an occupancy probability = prior probability    ----> -1
      //         # occupied: having an occupancy probability > prior probability   ----> 100
      //     */
          
      //     std::vector<std::pair<int, int>> frontiers;
      //     RCLCPP_INFO(this->get_logger(), "[Explore] Detecting frontier.");
      //     std::vector<std::vector<bool>> visited(mMapHeight, std::vector<bool>(mMapWidth, false));

      //     // Robot's odometry position (should be set from actual odom data)
      //     int odom_r = mCurrPose_wrt_map[0];
      //     int odom_c = mCurrPose_wrt_map[1];
      //     const int max_dist_threshold = 4; // Max cells away from curr pose to consider as frontiers
      //     const int min_dist_threshold = 0.1;  // Min Manhattan distance between frontiers

      //     for (int r = 0; r < mMapHeight; ++r) {
      //         for (int c = 0; c < mMapWidth; ++c) {
      //             // Check distance threshold from odometry
      //             // Change this to mMapGrid[r][c] == 0 && later.!!!!!!!!!!
      //             if (std::abs(r - odom_r) + std::abs(c - odom_c) > max_dist_threshold) {
      //                 continue; // Skip if beyond max distance
      //             }

      //             // Check if the current cell is free space and not visited
      //             // hange this to mMapGrid[r][c] == 0 && later.!!!!!!!!!!
      //             if (mMapGrid[r][c] != 100 && !visited[r][c]) {
      //                 visited[r][c] = true; // Mark as visited

      //                 // Check if adjacent to at least one unknown cell
      //                 for (const auto& [dr, dc] : std::vector<std::pair<int, int>>{
      //                         {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}})
      //                 {
      //                     int nr = r + dr, nc = c + dc;
      //                     if (nr >= 0 && nr < mMapHeight && nc >= 0 && nc < mMapWidth) {
      //                         if (mMapGrid[nr][nc] == -1) {
      //                             // Ensure new frontier is not too close to an existing one
      //                             bool too_close = false;
      //                             for (const auto& [fr, fc] : frontiers) {
      //                                 if (std::abs(fr - r) + std::abs(fc - c) < min_dist_threshold) {
      //                                     too_close = true;
      //                                     break;
      //                                 }
      //                             }

      //                             if (!too_close) {
      //                                 frontiers.emplace_back(r, c);
      //                             }
      //                             break;
      //                         }
      //                     }
      //                 }
      //             }
      //         }
      //     }
          
      //     mFrontiers = frontiers;
      //     RCLCPP_INFO(this->get_logger(), "[Explore] Detection algo ended.");
      //     RCLCPP_INFO(this->get_logger(), "[Explore] Frontiers detected: %i", frontiers.size());
          
      //     return frontiers;
      // }

      
      
      std::pair<int,int> explore()
      {
        // Todo: add direction (yaw) calculation.
        auto frontiers = detect_frontiers();
        // Find the Nearest frontier above the mMinExploreDistance.
        if (frontiers.empty()) 
        {
          RCLCPP_INFO(this->get_logger(), "[Explore] Frontiers detected: %i", mFrontiers.size());
          RCLCPP_INFO(this->get_logger(), "[Explore] No frontiers left.");
          mRobotState = State::IDLE;
          return {-100, -100}; // Return an invalid position if there are no frontiers
        }
        std::pair<int, int> nearest_frontier;
        // double min_distance = std::numeric_limits<double>::max();
        // const auto row;
        // const auto col;
        for (auto& [fx, fy] : frontiers) {
            // // Convert to world coordinates.
            // auto val = map_to_world(fx, fy);
            // fx = val.first;
            // fy = val.second;
            // Add the pose vals as printing -100, -100 here.

            double distance = std::hypot(fx - mCurrPose_wrt_map[0], fy - mCurrPose_wrt_map[1]); 
            RCLCPP_INFO(this->get_logger(), "[Explore] Nearest frontier would be calculated");
            RCLCPP_INFO(this->get_logger(), "[Explore] distnace , and threshold val: %d, %f", distance, mMinExploreDistance);

            if (distance < mMinExploreDistance) continue;
            else
            {
              RCLCPP_INFO(this->get_logger(), "[Explore] Nearest frontier: %i, %i", fx, fy);
              nearest_frontier = {fx, fy};
              break;            
            }
        }
        // // Convert to map coordinates
        // const auto goalY = mMapOrigin.position.x + mMapResolution * row;
        // const auto goalX = mMapOrigin.position.y + mMapResolution * col;
        // nearest_frontier = {goalX, goalY};
        return nearest_frontier;
      }

      std::pair<float, float> map_to_world(int x, int y)
      {
        // Convert map coordinates to world coordinates
        const auto worldX = mMapOrigin.position.x + mMapResolution * x;
        const auto worldY = mMapOrigin.position.y + mMapResolution * y;
        return {worldX, worldY};
      }

      void lookup_transform()
      {
        
        geometry_msgs::msg::TransformStamped t;
        if (!tf_buffer) 
        {
            RCLCPP_WARN(this->get_logger(), "tf_buffer is not initialized!");
            return;
        }

        if (!tf_buffer->canTransform("odom", "base_footprint", tf2::TimePointZero, 3s)) 
        {
            RCLCPP_WARN(this->get_logger(), "Transform not available yet.");
            return;
        }

        try {
          t = tf_buffer->lookupTransform("odom", "base_footprint", tf2::TimePointZero, 3s);
        } 
        catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s hi", "toFrameRel.c_str()", "fromFrameRel.c_str()", ex.what());
          return;
        }

        float x = t.transform.translation.x;
        float y = t.transform.translation.y;
        float z = t.transform.translation.z;
        // RCLCPP_INFO(this->get_logger(), "Transform btw %s and %s: x= %f, y= %f, z= %f","base_footprint", "map", x, y, z);
        mCurrPose_wrt_map = {x, y, z};          
      }

      void publish_goal(std::pair<int, int> goalPose)
      {
        geometry_msgs::msg::PoseStamped goalMsg;          
        goalMsg.pose.position.x = goalPose.first;
        goalMsg.pose.position.y = goalPose.second;
        // goalMsg.pose.orientation.w = 0;
        goalMsg.header.frame_id = "base_link";  // map
        goalMsg.header.stamp = this->now();        
        goal_pose_pub->publish(goalMsg);
        RCLCPP_INFO(this->get_logger(), "[Explore] Goal pose published: x= %f, y= %f", goalMsg.pose.position.x, goalMsg.pose.position.y);
        // RCLCPP_INFO(this->get_logger(), "[Explore] Robot state moving.");
      }

      void timer_callback()
      {
        lookup_transform();
        if (!mFrontiers.empty())
        {
          publish_frontier_markers(mFrontiers);          
        }
        publish_goal_marker(mCurrGoalPose);        
        if (mRobotState != State::E_STOP)
        {  
          // RCLCPP_INFO(this->get_logger(), "[Explore] Robot not in E-stop.");
          if (mRobotState == State::START)
          {
            RCLCPP_INFO(this->get_logger(), "[Explore] Robot Started.");
            mCurrGoalPose = explore(); 
            RCLCPP_INFO(this->get_logger(), "[Explore] Robot state at 0: %d", static_cast<int>(mRobotState));

            if (mCurrGoalPose.first == -100 && mCurrGoalPose.second == -100)
            {
              RCLCPP_INFO(this->get_logger(), "[Explore] Robot started but goal not recieved.");
              mRobotState = State::START;              
            }
            else
            {
              RCLCPP_INFO(this->get_logger(), "[Explore] Robot started and goal received.");
              // publish_goal(mCurrGoalPose);        
              mRobotState = State::MOVING;
            }
            RCLCPP_INFO(this->get_logger(), "[Explore] Robot state at 1: %d", static_cast<int>(mRobotState));

          }
          float goalReachedHyp = std::hypot(mCurrGoalPose.first - mCurrPose_wrt_map[0], mCurrGoalPose.second - mCurrPose_wrt_map[1]);
          RCLCPP_INFO(this->get_logger(), "[Explore] distance to Goal: %f", goalReachedHyp);
          // mDiagonalTollerance = 0.1;

          if ( std::hypot(mCurrGoalPose.first - mCurrPose_wrt_map[0], mCurrGoalPose.second - mCurrPose_wrt_map[1]) < mDiagonalTollerance)
          {
            RCLCPP_INFO(this->get_logger(), "[Explore] ////////////////////////////////////////////////////.");
            RCLCPP_INFO(this->get_logger(), "[Explore] Goal reached.");
            mRobotState = State::REACHED_GOAL;
          }
          if (mRobotState == State::REACHED_GOAL)
          {
            RCLCPP_INFO(this->get_logger(), "[Explore] Robot state idle.");
            mCurrGoalPose = explore(); 
            // publish_goal(mCurrGoalPose); 
            mRobotState = State::MOVING;
          }
          else if (mRobotState == State::MOVING)
          {
            RCLCPP_INFO(this->get_logger(), "[Explore] Robot state moving.");
            // Handle the case when nav rejects the goal due to out of bounds, and robot does not actully move, after a timer.
            // Check if the robot has not moved for a certain duration
            static rclcpp::Time last_movement_time = this->now();
            static std::vector<float> last_position = mCurrPose_wrt_map;

            if (mCurrPose_wrt_map != last_position) {
              last_movement_time = this->now();
              last_position = mCurrPose_wrt_map;
            }

            if ((this->now() - last_movement_time).seconds() > 5.0) {
              RCLCPP_WARN(this->get_logger(), "[Explore] Robot has not moved for 5 seconds. Re-evaluating goal.");
              mCurrGoalPose = explore();
              last_movement_time = this->now();
            }
          }
          RCLCPP_INFO(this->get_logger(), "[Explore] Robot state: %d", static_cast<int>(mRobotState));
          publish_goal(mCurrGoalPose); 
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "[Explore] Robot is in Emergency Stop.");
        }
      }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Explore>());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inside main.");

  rclcpp::shutdown();
  return 0;
}