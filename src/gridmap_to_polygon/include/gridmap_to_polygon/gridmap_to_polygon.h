#ifndef __GRIDMAP_TO_POLYGONS__
#define __GRIDMAP_TO_POLYGONS__

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>

class GridMapToPolygonConverter : public rclcpp::Node
{
public:
    GridMapToPolygonConverter();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void loadMapFromFile();
    cv::Mat removeNoise(const cv::Mat &binary_image);
    void processGridMap(const nav_msgs::msg::OccupancyGrid &grid_map);

    // Biến thành viên
    std::string map_topic_;
    bool use_file_input_;
    std::string map_yaml_path_;
    std::string map_pgm_path_;
    int occupancy_threshold_;
    double min_polygon_area_;
    double simplify_tolerance_;
    int min_area_;
    std::string publish_frame_id_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr outer_polygon_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr inner_polygon_pub_;
};

#endif