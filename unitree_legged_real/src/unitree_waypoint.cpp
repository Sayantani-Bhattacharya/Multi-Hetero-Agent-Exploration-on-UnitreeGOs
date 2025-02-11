/// \file
/// \brief The nusim node is a simulation and visualization tool for the turtlebot3 robots.
///        It uses rviz2 for visualization and provides a simulated environment. The package
///        creates stationary walls and obstacles and track the position of a red robot.
///
/// PARAMETERS:
///     \param rate (int): Timer callback frequency [Hz]
///     \param x0 (double): Initial x coordinate of the robot [m]
///     \param y0 (double): Initial y coordinate of the robot [m]
///     \param theta0 (double): Initial theta angle of the robot [radians]
///     \param obstacles.x (std::vector<double>): Vector of x coordinates for each obstacle [m]
///     \param obstacles.y (std::vector<double>): Vector of y coordinates for each obstacle [m]
///     \param obstacles.r (double): Radius of cylindrical obstacles [m]
///     \param arena_x_length (double): Inner length of arena in x direction [m]
///     \param arena_y_length (double): Inner length of arena in y direction [m]
///
/// PUBLISHES:
///     \param ~/timestep (std_msgs::msg::UInt64): Current simulation timestep
///     \param ~/obstacles (visualization_msgs::msg::MarkerArray): Marker obstacles that are
///                                                                displayed in Rviz
///     \param ~/walls (visualization_msgs::msg::MarkerArray): Marker walls that are
///                                                            displayed in Rviz
///
/// SUBSCRIBES:
///     None
///
/// SERVERS:
///     \param ~/reset (std_srvs::srv::Empty): Resets simulation to initial state
///     \param ~/teleport (nusim::srv::Teleport): Teleport robot to a specific pose
///
/// CLIENTS:
///     None
///
/// BROADCASTERS:
///     \param tf_broadcaster_ (tf2_ros::TransformBroadcaster): Broadcasts red turtle position

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "unitree_kinematics/geometry2d.hpp"
#include "unitree_kinematics/se2d.hpp"
#include "unitree_kinematics/kinematic_state.hpp"

#include "ros2_unitree_legged_msgs/srv/waypoint.hpp"

#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace std::chrono_literals;
using namespace UNITREE_LEGGED_SDK;

/// \brief This class publishes the current timestep of the simulation, obstacles and walls that
///        appear in Rviz as markers. The class has a timer_callback to continually update the
///        simulation at each timestep. The reset service resets the simulation to the initial
///        state thus restarting the simulation. A teleport service is available to teleport a
///        turtlebot to any pose. A broadcaster broadcasts the robots TF frames to a topic for
///        visualization in Rviz. The simulation operates in a loop, updating the state of the
///        world, publishing messages that provides state information simulating a real robot,
///        and processing service/subscriber callbacks for commands for the next time step. The
///        loop runs at a fixed frequency until termination.
///
///  \param rate (int): Timer callback frequency [Hz]
///  \param x0_ (double): Initial x coordinate of the robot [m]
///  \param y0_ (double): Initial y coordinate of the robot [m]
///  \param theta0_ (double): Initial theta angle of the robot [radians]
///  \param x_ (double): Current x coordinate of the robot [m]
///  \param y_ (double): Current y coordinate of the robot [m]
///  \param theta_ (double): Current theta angle of the robot [radians]

class Unitree_Waypoint : public rclcpp::Node
{
public:
  Unitree_Waypoint()
  : Node("unitree_waypoint"), timestep_(0)
  {
    // Parameter descirption
    auto rate_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto x0_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto y0_des = rcl_interfaces::msg::ParameterDescriptor{};
    auto theta0_des = rcl_interfaces::msg::ParameterDescriptor{};
    
    rate_des.description = "Timer callback frequency [Hz]";
    x0_des.description = "Initial x coordinate of the robot [m]";
    y0_des.description = "Initial y coordinate of the robot [m]";
    theta0_des.description = "Initial theta angle of the robot [radians]";

    // Declare default parameters values
    declare_parameter("rate", 200, rate_des);     // Hz for timer_callback
    declare_parameter("x0", 0.0, x0_des);         // Meters
    declare_parameter("y0", 0.0, y0_des);         // Meters
    declare_parameter("theta0", 0.0, theta0_des); // Radians

    // Get params - Read params from yaml file that is passed in the launch file
    int rate = get_parameter("rate").get_parameter_value().get<int>();
    x0_ = get_parameter("x0").get_parameter_value().get<double>();
    y0_ = get_parameter("y0").get_parameter_value().get<double>();
    theta0_ = get_parameter("theta0").get_parameter_value().get<double>();

    // Set current robot pose equal to initial pose
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;

    // Timer timestep [seconds]
    dt_ = 1.0 / static_cast<double>(rate);

    // Create ~/timestep publisher
    timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    // Create ~/waypoint service
    teleport_server_ = create_service<ros2_unitree_legged_msgs::srv::Waypoint>(
      "~/waypoint",
      std::bind(&Unitree_Waypoint::waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize and broadcast static transform
    // static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    // static_tf_broadcaster_.sendTransform(static_tf_zedd_unitree_);

    // this->
    highcmd_publisher_ = create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);

    // Create Timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      std::bind(&Unitree_Waypoint::timer_callback, this));

    // TODO: Subscriber
  }

private:
  // Variables
  size_t timestep_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  double x_, y_, theta_;     // Theta in radians, x & y in meters.
  double x0_ = 0.0;          // Meters
  double y0_ = 0.0;          // Meters
  double theta0_ = 0;        // Radians
  double dt_ = 0.0; // Timer in seconds

  // Set Pose of Robot Relative to Camera (for rviz)
  geometry_msgs::msg::TransformStamped static_tf_zedd_unitree_;

  // Create objects
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr highcmd_publisher_;
  rclcpp::Service<ros2_unitree_legged_msgs::srv::Waypoint>::SharedPtr teleport_server_;
  
  // TODO: subscriber

  /// \brief Move the robot to specific goal pose
  void waypoint_callback(
    ros2_unitree_legged_msgs::srv::Waypoint::Request::SharedPtr request,
    ros2_unitree_legged_msgs::srv::Waypoint::Response::SharedPtr)
  {
    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
  }

  /// \brief Broadcast the TF frames of the robot
  void broadcast_goalpose()
  {
    geometry_msgs::msg::TransformStamped tf_;

    tf_.header.stamp = this->get_clock()->now();
    tf_.header.frame_id = "map";
    tf_.child_frame_id = "goalpose_unitree";
    tf_.transform.translation.x = 0.0;
    tf_.transform.translation.y = 0.0;
    tf_.transform.translation.z = 0.0;     // Dog only exists in 2D

    tf2::Quaternion q_;
    q_.setRPY(0, 0, 0);     // Rotation around z-axis
    tf_.transform.rotation.x = q_.x();
    tf_.transform.rotation.y = q_.y();
    tf_.transform.rotation.z = q_.z();
    tf_.transform.rotation.w = q_.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(tf_);
  }

  /// \brief Main simulation time loop
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++;
    timestep_publisher_->publish(message);

    ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros;

    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    high_cmd_ros.level_flag = HIGHLEVEL;
    high_cmd_ros.mode = 0;
    high_cmd_ros.gait_type = 0;
    high_cmd_ros.speed_level = 0;
    high_cmd_ros.foot_raise_height = 0;
    high_cmd_ros.body_height = 0;
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.velocity[0] = 0.0f;
    high_cmd_ros.velocity[1] = 0.0f;
    high_cmd_ros.yaw_speed = 0.0f;
    high_cmd_ros.reserve = 0;

    std::cout << "Mode 1" << std::endl;
    high_cmd_ros.mode = 2;
    high_cmd_ros.gait_type = 1;
    // high_cmd_ros.velocity[0] = 0.2f; // -1  ~ +1
    // high_cmd_ros.body_height = -0.1;
    high_cmd_ros.yaw_speed = 4.0/1.0;

    highcmd_publisher_->publish(high_cmd_ros);

    broadcast_goalpose();

    static_tf_zedd_unitree_.header.stamp = this->get_clock()->now();
    static_tf_zedd_unitree_.header.frame_id = "zed_camera_link";
    static_tf_zedd_unitree_.child_frame_id = "base_link";
    static_tf_zedd_unitree_.transform.translation.x = 0.0;
    static_tf_zedd_unitree_.transform.translation.y = 0.0;
    static_tf_zedd_unitree_.transform.translation.z = 0.0;

    tf2::Quaternion q_zedd_unitree_;
    q_zedd_unitree_.setRPY(0, 0, 0);     // Rotation around z-axis
    static_tf_zedd_unitree_.transform.rotation.x = q_zedd_unitree_.x();
    static_tf_zedd_unitree_.transform.rotation.y = q_zedd_unitree_.y();
    static_tf_zedd_unitree_.transform.rotation.z = q_zedd_unitree_.z();
    static_tf_zedd_unitree_.transform.rotation.w = q_zedd_unitree_.w();

    tf_broadcaster_->sendTransform(static_tf_zedd_unitree_);
  }
};

/// \brief Main function for node create, error handling and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
  //             << "Make sure the robot is standing on the ground." << std::endl
  //             << "Press Enter to continue..." << std::endl;
  // std::cin.ignore();

  rclcpp::spin(std::make_shared<Unitree_Waypoint>());
  rclcpp::shutdown();
  return 0;
}