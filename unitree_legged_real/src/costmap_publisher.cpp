#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap.hpp"

class CostmapPublisher : public rclcpp::Node
{
public:
    CostmapPublisher() : Node("costmap_publisher")
    {
        // Subscribe to occupancy grid topic
        occupancy_grid_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&CostmapPublisher::occupancyGridCallback, this, std::placeholders::_1));

        // Create publisher for costmap
        costmap_publisher_ = this->create_publisher<nav2_msgs::msg::Costmap>("/my_costmap", 10);
    }

private:
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_msg)
    {
        // Create a new costmap message
        auto costmap_msg = std::make_unique<nav2_msgs::msg::Costmap>();

        // Fill in the header
        costmap_msg->header = occupancy_grid_msg->header;

        // Set the width and height of the costmap
        costmap_msg->metadata.size_x = occupancy_grid_msg->info.width;
        costmap_msg->metadata.size_y = occupancy_grid_msg->info.height;

        // Set the resolution of the costmap
        costmap_msg->metadata.resolution = occupancy_grid_msg->info.resolution;

        // Convert the occupancy grid data to unsigned char
        costmap_msg->data.resize(occupancy_grid_msg->data.size());
        for (size_t i = 0; i < occupancy_grid_msg->data.size(); ++i) {
            costmap_msg->data[i] = occupancy_grid_msg->data[i];
        }

        // Publish the costmap message
        costmap_publisher_->publish(std::move(costmap_msg));
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscription_;
    rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
