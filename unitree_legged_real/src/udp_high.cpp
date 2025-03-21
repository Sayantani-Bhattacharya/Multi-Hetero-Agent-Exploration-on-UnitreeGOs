#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using UNITREE_LEGGED_SDK::UDP;
using UNITREE_LEGGED_SDK::HighCmd;
using UNITREE_LEGGED_SDK::HighState;

class UDPHighBridge
{
public:
  UDP udp;

  HighCmd cmd = {0};
  HighState state = {0};

public:
  UDPHighBridge()
  : udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
  {
    udp.InitCmdData(cmd);
  }
};

class UDPHighNode : public rclcpp::Node
{
public:
  UDPHighNode()
  : Node("udp_high")
  {
    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};
    param.description = "The rate at which the node requests state information over UDP (Hz).";
    declare_parameter("rate", 500.0, param);
    rate_ = get_parameter("rate").get_parameter_value().get<double>();
    interval_ = 1.0 / rate_;

    //Timers
    timer_ = create_wall_timer(
      static_cast<std::chrono::microseconds>(static_cast<int>(interval_ * 1000000.0)),
      std::bind(&UDPHighNode::timer_callback, this)
    );

    //Publishers
    pub_state_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighState>("high_state", 10);
    
    //Subscribers
    sub_cmd_ = this->create_subscription<ros2_unitree_legged_msgs::msg::HighCmd>(
      "high_cmd",
      10,
      std::bind(&UDPHighNode::cmd_callback, this, std::placeholders::_1)
    );
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr pub_state_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr sub_cmd_;
  

  double rate_, interval_;
  UDPHighBridge bridge_;
  ros2_unitree_legged_msgs::msg::HighState state_ros_;

  void cmd_callback(const ros2_unitree_legged_msgs::msg::HighCmd::SharedPtr msg)
  {
    //Convert ROS message to UDP command
    bridge_.cmd = rosMsg2Cmd(msg);

    //Send UDP command
    bridge_.udp.SetSend(bridge_.cmd);
    bridge_.udp.Send();
  }

  void timer_callback()
  {
    //Get state over UDP
    bridge_.udp.Recv();
    bridge_.udp.GetRecv(bridge_.state);

    //Publish state as ROS message
    state_ros_ = state2rosMsg(bridge_.state);
    pub_state_->publish(state_ros_);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UDPHighNode>());
  rclcpp::shutdown();
  return 0;
}
