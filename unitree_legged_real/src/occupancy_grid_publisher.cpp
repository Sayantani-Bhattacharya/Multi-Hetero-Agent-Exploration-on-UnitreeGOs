#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

class OccupancyGridPublisher : public rclcpp::Node
{
public:
  OccupancyGridPublisher()
  : Node("occupancy_grid_publisher")
  {
    // Initialize occupancy grid message as map
    // TODO: declare these as parameters
    occupancy_grid.header.stamp = now();
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.map_load_time = now();
    occupancy_grid.info.resolution = map_resolution; // meters/cell
    occupancy_grid.info.width = grid_width;  // cells
    occupancy_grid.info.height = grid_height;  // cells
    occupancy_grid.info.origin.position.x = map_origin_x; // meters
    occupancy_grid.info.origin.position.y = map_origin_y; // meters
    occupancy_grid.info.origin.position.z = default_map_level; // meters
    occupancy_grid.info.origin.orientation.w = 1.0;

    // Initialize as empty map (0 for free, 100 for occupied, -1 for unknown)
    occupancy_grid.data.resize(grid_width * grid_height, -1);

    // Set QoS settings for the /map topic to match /global_costmap/global_costmap node's subscriber
    rclcpp::QoS custom_qos(rclcpp::KeepLast(10));
    custom_qos.transient_local();

    // Publisher
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", custom_qos);

    // Subscriber
    // subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    //   "/zed/zed_node/mapping/fused_cloud", 10, std::bind(&OccupancyGridPublisher::pointCloudCallback, this, std::placeholders::_1));

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed/zed_node/point_cloud/cloud_registered", 10, std::bind(&OccupancyGridPublisher::pointCloudCallback, this, std::placeholders::_1));

    // Timer
    timer_ = create_wall_timer(std::chrono::milliseconds(1000),
      std::bind(&OccupancyGridPublisher::publishOccupancyGrid, this));
  }

private:

  // Occupancy Grid
  nav_msgs::msg::OccupancyGrid occupancy_grid;

  // PCL type Point Cloud
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  
  // MAP PARAMETERS

  const int map_precision = 10; // cells per meter. 
  const double map_resolution = 1.0 / static_cast<double>(map_precision); // meters per cell. Must be 1/N.
  
  const double map_width = 40.0; // meters. Must result in integer grid cells.
  const double map_height = 40.0; // meters. Must result in integer grid cells.

  const double map_origin_x = -map_width/4.0; // meters. Bottom left corner of the grid, in map frame. Must result in integer grid cells.
  const double map_origin_y = -map_height/2.0; // meters. Bottom left corner of the grid, in map frame. Must result in integer grid cells.

  const int grid_width = map_width * map_precision; // cells
  const int grid_height = map_height * map_precision; // cells

  const double unitree_height = 0.4; // meters
  const double zedmount_height = 0.1; // meters
  const double default_map_level = -(unitree_height + zedmount_height); // meters

  // FLAGS

  const bool adaptive_snapping = false; // If true, use the lowest point as the floor. Otherwise take the fixed height.

  void publishOccupancyGrid()
  {    

    // Simulate an obstacle in the center
    // int obstacle_index = (occupancy_grid->info.width / 2) + (occupancy_grid->info.height / 2) * occupancy_grid->info.width;
    // occupancy_grid->data[obstacle_index] = 100;

    // Publish the message
    publisher_->publish(occupancy_grid);

    RCLCPP_DEBUG(get_logger(), "OccupancyGrid published");
  }

  // Fill in the 2D map based on the 3D map
  void project_pointcloud_to_grid()
  {
    // Define the thickness of the slice
    const float slice_thickness = unitree_height + zedmount_height; // meters
    const float floor_clearance = 0.15; // meters
    const float sky_clearance = 2.5; // meters
    const float reflective_floor_clearance = -3.0; // meters

    // Values are tuned for specific mapping params

    const int num_points_threshold = 2; //1-100. minimum %age confidence in a grid cell being an obstacle.
    const int sensitivity = 1; // 1-100. Larger is more sensitive. %age of confidence in an obstacle's presence, provided by one point.

    const int erosion_threshold = 6; // Minimum no. of free cells around an obstacle to qualify for erosion. 0-8.
    const int erosion_passes = 10; // No. of erosion iterations. 0-inf.

    const float clear_width = 2.0; // meters
    const float clear_height = 2.0; // meters

    const double padding_thickness = 0.5; // meters

    // Iterate through each point in the point cloud
    for (const auto& point : pcl_cloud.points) 
    {
        // Calculate the grid cell indices corresponding to the point
        int grid_x = static_cast<int>((point.x - occupancy_grid.info.origin.position.x) / occupancy_grid.info.resolution);
        int grid_y = static_cast<int>((point.y - occupancy_grid.info.origin.position.y) / occupancy_grid.info.resolution);

        // Check if point is confined to the grid
        if((grid_x >= 0) && (grid_x < grid_width) && (grid_y >= 0) && (grid_y < grid_height))
        {
          // Convert grid cell indices to occupancy grid index
          int index = grid_x + grid_y * occupancy_grid.info.width;

          // Check if the point falls within the height slice
          if (point.z >= occupancy_grid.info.origin.position.z + floor_clearance &&
              point.z <= occupancy_grid.info.origin.position.z + floor_clearance + slice_thickness) {
              // Increment confidence in obstacle presence for a certain grid cell, for every point present in it
              if(occupancy_grid.data[index] == -1)
              {
                occupancy_grid.data[index] = sensitivity;
              }
              else if(occupancy_grid.data[index] >= 100 - sensitivity)
              {
                occupancy_grid.data[index] = 100;
              }
              else
              {
                occupancy_grid.data[index] += sensitivity;
              }
          }
          else // Check if point is part of the floor/ceiling
          if ((point.z >= occupancy_grid.info.origin.position.z + reflective_floor_clearance && // FLOOR
              point.z <= occupancy_grid.info.origin.position.z + floor_clearance) || // FLOOR
              (point.z >= occupancy_grid.info.origin.position.z + sky_clearance)) { // CEILING

              occupancy_grid.data[index] = 0;
          }
        }
    }
    RCLCPP_DEBUG(get_logger(), "Obstacles detected");

    // Post Processing

    // Convert confidence values in occupancy grid based on a threshold.
    threshold_map(num_points_threshold);

    // Erode away outliers
    erode_map(erosion_threshold, erosion_passes);

    // Pad the map's borders with obstacles.
    pad_map(padding_thickness);

    // Assume that the robot starts off heading towards a clear spot.
    clear_initial_heading(clear_width, clear_height);

  }

  // Adjust floor level of map
  void snap_grid_to_floor()
  {
    // Initialize variables to hold the minimum z coordinate and its corresponding point
    float min_z = std::numeric_limits<float>::max();
    pcl::PointXYZ min_point;

    // Iterate through each point in the point cloud
    for (const auto& point : pcl_cloud.points) {
        // Check if the z coordinate of the current point is smaller than the current minimum
        if (point.z < min_z) {
            // Update the minimum z coordinate and its corresponding point
            min_z = point.z;
            min_point = point;
        }
    }

    // Print the minimum z coordinate and its corresponding point
    RCLCPP_DEBUG(get_logger(), "Minimum z coordinate: %f", min_z);

    // Set grid height
    occupancy_grid.info.origin.position.z = min_z; // meters
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
  {
    // Process the PointCloud2 data and update the occupancy grid map

    // Example: Convert PointCloud2 to PointCloud data
    pcl::fromROSMsg(*point_cloud_msg, pcl_cloud);

    // Check if the point cloud is empty
    if (pcl_cloud.empty()) {
        RCLCPP_ERROR(get_logger(), "Received empty point cloud");
        return;
    }
    else
    {
      RCLCPP_DEBUG(get_logger(), "Received PointCloud message");
    }

    if(adaptive_snapping)
    {
      snap_grid_to_floor();
    }
    else
    {
      occupancy_grid.info.origin.position.z = default_map_level;
    }

    project_pointcloud_to_grid();
  }

  void threshold_map(const int num_points_threshold)
  {
    for (int index = 0; index < grid_width * grid_height; index++)
    {
      // Hard local thereshold
      if(occupancy_grid.data[index] == -1)
      {
        occupancy_grid.data[index] = -1;
      }
      else if(occupancy_grid.data[index] < num_points_threshold)
      {
        occupancy_grid.data[index] = 0;
      }
      else
      {
        occupancy_grid.data[index] = 100;
      }
    }

    RCLCPP_DEBUG(get_logger(), "Generated map");
  }

  void pad_map(const double padding_thickness)
  {
    draw_rectangle(map_origin_x+padding_thickness/2.0, map_origin_y+map_height/2.0, padding_thickness,map_height,"obstacle"); // Left Wall
    draw_rectangle(map_width+map_origin_x-padding_thickness/2.0, map_origin_y+map_height/2.0, padding_thickness,map_height,"obstacle"); // Right Wall
    draw_rectangle(map_origin_x+map_width/2.0, map_origin_y+padding_thickness/2.0, map_width,padding_thickness,"obstacle"); // Lower Wall
    draw_rectangle(map_origin_x+map_width/2.0, map_origin_y+map_height-padding_thickness/2.0, map_width,padding_thickness,"obstacle"); // Upper Wall
  
    RCLCPP_DEBUG(get_logger(), "Padded map");
  }

  void erode_map(const int erosion_threshold, const int erosion_passes)
  {
    int free_ctr = 0;

    for (int pass = 0; pass <= erosion_passes; pass++)
    {
      for (int index = 0; index < grid_width * grid_height; index++)
      {     
        // Eliminate small encalves
        if(occupancy_grid.data[index] == 100)
        {
          // Check North
          if(index >= grid_width)
          {
            if(occupancy_grid.data[index - grid_width] == 0)
            {
              free_ctr += 1;
            }
          }
          // Check Northwest
          if((index >= grid_width) && (index%grid_width != 0))
          {
            if(occupancy_grid.data[index - grid_width - 1] == 0)
            {
              free_ctr += 1;
            }
          }
          // Check West
          if(index%grid_width != 0)
          {
            if(occupancy_grid.data[index - 1] == 0)
            {
              free_ctr += 1;
            }
          }
          // Check Southwest
          if((index < grid_width * (grid_height-1)) && (index%grid_width != 0))
          {
            if(occupancy_grid.data[index + grid_width - 1] == 0)
            {
              free_ctr += 1;
            }
          }
          // Check South
          if(index < grid_width * (grid_height-1))
          {
            if(occupancy_grid.data[index + grid_width] == 0)
            {
              free_ctr += 1;
            }
          }
          // Check Southeast
          if((index < grid_width * (grid_height-1)) && (index%grid_width != (grid_width-1)))
          {
            if(occupancy_grid.data[index + grid_width + 1] == 0)
            {
              free_ctr += 1;
            }
          }
          // Check East
          if(index%grid_width != (grid_width-1))
          {
            if(occupancy_grid.data[index + 1] == 0)
            {
              free_ctr += 1;
            }
          }
          // Check Northeast
          if((index >= grid_width) && (index%grid_width != (grid_width-1)))
          {
            if(occupancy_grid.data[index - grid_width + 1] == 0)
            {
              free_ctr += 1;
            }
          }
        }

        // Erosion threshold. 0-8.
        if(free_ctr >= erosion_threshold)
        {
          occupancy_grid.data[index] == 0;
          // RCLCPP_INFO(get_logger(), "Eroded");

        }
        // Reset counter
        free_ctr = 0;
      }
    }

    RCLCPP_DEBUG(get_logger(), "Eroded map");
  }

  void clear_initial_heading(const double clear_width_meters, const double clear_height_meters)
  {
    // Buffer around the cleared section
    double buffer = 0.5;

    // Free initial heading
    draw_rectangle(0.0, 0.0, clear_width_meters, clear_height_meters, "free");
    // Bunker behind initial position
    draw_rectangle(-(clear_width_meters/2.0 + buffer/2.0), 0.0, buffer, clear_height_meters, "obstacle");
    // Walls adjacent to initial heading
    draw_rectangle(-(clear_width_meters/2.0), (clear_height_meters/2.0 + buffer/2.0), clear_width_meters, buffer, "obstacle");
    draw_rectangle(-(clear_width_meters/2.0), -(clear_height_meters/2.0 + buffer/2.0), clear_width_meters, buffer, "obstacle");
  }

  /// \brief Draws a rectangle in th grid
  // \center_x : center in \map frame in meters
  // \center_y : center in \map frame in meters
  // \width : width of rectangle in meters
  // \height : height of rectangle in meters
  // \type : "obstacle", "free", or "unexplored"
  void draw_rectangle(const double center_x, const double center_y, const double width, const double height,const std::string type)
  {
    // Check if rectangle is in the map
    if (width <= 0.0 || height <= 0.0 || // Check if width or height is non-positive
        center_x + width/2.0 > map_width + map_origin_x || // Right Limit in /map frame
        center_x - width/2.0 < map_origin_x || // Left Limit in /map frame
        center_y + height/2.0 > map_height + map_origin_y || // Upper limit in /map frame
        center_y - height/2.0 < map_origin_y // Lower limit in /map frame
        ) 
    {
        RCLCPP_ERROR(get_logger(), "Rectangle is out of bounds! %f, %f, %f. %f", center_x, center_y, width, height);
        throw std::runtime_error("Rectangle is out of bounds!");
    }
    else
    {
      const int ncx = static_cast<int>((center_x - map_origin_x) / map_resolution);
      const int ncy = static_cast<int>((center_y - map_origin_y) / map_resolution);
      const int nwidth = static_cast<int>(width / map_resolution);
      const int nheight = static_cast<int>(height / map_resolution);

      int fill = -1;
      if(type == "obstacle")
      {
        fill = 100;
      }
      else if(type == "free")
      {
        fill = 0;
      }
      else if(type == "unexplored")
      {
        fill = -1;
      }

      for(int index = 0; index < (nwidth * nheight); index++)
      {
        occupancy_grid.data[ (ncx - nwidth/2) + (ncy - nheight/2) * grid_width // Bottom left corner. 
                              + static_cast<int>(index/nwidth)*grid_width // Row increment
                              + index%(nwidth)] = fill; // Column Increment
      }
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridPublisher>());
  rclcpp::shutdown();
  return 0;
}
