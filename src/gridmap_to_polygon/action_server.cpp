#include "gridmap_to_polygon/action_server.h"

GridMapToPolygonsActionServer::GridMapToPolygonsActionServer()
 : Node("convert_grid_map_to_polygon_server")
{
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<ConvertGridmapToPolygons>(
        this,
        "convert_grid_map_to_polygons",
        std::bind(&GridMapToPolygonsActionServer::handle_goal, this, _1, _2),
        std::bind(&GridMapToPolygonsActionServer::handle_cancel, this, _1),
        std::bind(&GridMapToPolygonsActionServer::handle_accepted, this, _1));

    auto converter_node = std::make_shared<rclcpp::Node>("grid_map_to_polygons_node");
    converter_ = std::make_shared<GridMapToPolygonConverter>(converter_node);
    converter_thread_ = std::make_unique<std::thread>(
        [converter_node]() {
            rclcpp::spin(converter_node);
        });
}

rclcpp_action::GoalResponse GridMapToPolygonsActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ConvertGridmapToPolygons::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GridMapToPolygonsActionServer::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GridMapToPolygonsActionServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&GridMapToPolygonsActionServer::execute, this, _1), goal_handle}.detach();
}

void GridMapToPolygonsActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    auto result = std::make_shared<ConvertGridmapToPolygons::Result>();
    result->polygons = converter_->getPolygons();
    goal_handle->succeed(result);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GridMapToPolygonsActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}