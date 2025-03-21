#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <string>
#include <vector>

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : low_udp(LOWLEVEL),
          high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }
};


class LL_UDP : public rclcpp::Node
{
  public:
    LL_UDP()
    : Node("low_udp")
    {
      timer_ = this->create_wall_timer(
        2ms,
        std::bind(&LL_UDP::timer_callback, this));
      pub_low_ = this->create_publisher<ros2_unitree_legged_msgs::msg::LowState>("low_state", 10);
      sub_low_ = this->create_subscription<ros2_unitree_legged_msgs::msg::LowCmd>("low_cmd", 10,
        std::bind(&LL_UDP::low_cmd_cb, this, std::placeholders::_1));
    }
  private:
    void low_cmd_cb(const ros2_unitree_legged_msgs::msg::LowCmd::SharedPtr msg)
    {
      // I had to change the argument to the above vs
      // (const ros2_unitree_legged_msgs::msg::LowCmd & msg) const
      // otherwise msg did not match the correct type to call rosMsg2Cmd :/
      custom.low_cmd = rosMsg2Cmd(msg);
      custom.low_udp.SetSend(custom.low_cmd);
      custom.low_udp.Send();
    }
    void timer_callback()
    {
      // do publish here
      custom.low_udp.Recv();
      custom.low_udp.GetRecv(custom.low_state);
      low_state_ros = state2rosMsg(custom.low_state);
      pub_low_->publish(low_state_ros);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::LowState>::SharedPtr pub_low_;
    rclcpp::Subscription<ros2_unitree_legged_msgs::msg::LowCmd>::SharedPtr sub_low_;
    Custom custom;
    ros2_unitree_legged_msgs::msg::LowState low_state_ros;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LL_UDP>());
  rclcpp::shutdown();
  return 0;
}
