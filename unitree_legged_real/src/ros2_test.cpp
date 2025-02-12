#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HelloWorldPublisher : public rclcpp::Node
{
public:
    HelloWorldPublisher() : Node("hello_world_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&HelloWorldPublisher::publishMessage, this));
    }

private:
    void publishMessage()
    {
        auto message = std_msgs::msg::String();
        message.data = "HELLO WORLD";
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloWorldPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
