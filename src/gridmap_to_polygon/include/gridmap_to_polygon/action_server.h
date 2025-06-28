#ifndef __ACTION_SERVER__
#define __ACTION_SERVER__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <gridmap_to_polygon_msg/action/convert_gridmap_to_polygons.hpp>
#include <gridmap_to_polygon/gridmap_to_polygon.h>

class GridMapToPolygonsActionServer : public rclcpp::Node
{
public:
    using ConvertGridmapToPolygons = gridmap_to_polygon_msg::action::ConvertGridmapToPolygons;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ConvertGridmapToPolygons>;

    GridMapToPolygonsActionServer();

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ConvertGridmapToPolygons::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    rclcpp_action::Server<ConvertGridmapToPolygons>::SharedPtr action_server_;
    std::shared_ptr<GridMapToPolygonConverter> converter_;
    std::unique_ptr<std::thread> converter_thread_;
};

#endif