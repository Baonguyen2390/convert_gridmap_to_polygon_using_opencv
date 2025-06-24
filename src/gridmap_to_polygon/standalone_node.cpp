#include <rclcpp/rclcpp.hpp>
#include <gridmap_to_polygon/gridmap_to_polygon.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GridMapToPolygonConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}