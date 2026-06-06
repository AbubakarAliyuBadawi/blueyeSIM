#include "blueye_stonefish_bt/behaviors/launch_docking_procedure.hpp"
#include <chrono>

using namespace std::chrono_literals;

LaunchDockingProcedure::LaunchDockingProcedure(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    // Stop waypoint controller so it does not conflict with the docking controller
    stop_waypoint_client_ = g_node->create_client<mundus_mir_msgs::srv::RunWaypointController>(
        "/blueye/run_waypoint_controller");

    // Trigger the Stonefish docking controller (must already be running via launch file)
    start_docking_client_ = g_node->create_client<std_srvs::srv::SetBool>(
        "/blueye/start_docking");
}

BT::NodeStatus LaunchDockingProcedure::tick()
{
    RCLCPP_INFO(g_node->get_logger(), "LaunchDockingProcedure: stopping waypoint controller");

    if (stop_waypoint_client_->wait_for_service(3s)) {
        auto req = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
        req->run = false;
        auto future = stop_waypoint_client_->async_send_request(req);
        rclcpp::spin_until_future_complete(g_node, future, 3s);
    } else {
        RCLCPP_WARN(g_node->get_logger(), "Waypoint controller service not available — continuing anyway");
    }

    RCLCPP_INFO(g_node->get_logger(), "LaunchDockingProcedure: calling /blueye/start_docking");

    if (!start_docking_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(g_node->get_logger(),
                     "Docking service /blueye/start_docking not available. "
                     "Is stonefish_docking.launch.py running?");
        return BT::NodeStatus::FAILURE;
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    auto future = start_docking_client_->async_send_request(req);

    if (rclcpp::spin_until_future_complete(g_node, future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(g_node->get_logger(), "Failed to call /blueye/start_docking");
        return BT::NodeStatus::FAILURE;
    }

    auto result = future.get();
    if (!result->success) {
        RCLCPP_ERROR(g_node->get_logger(), "Docking start rejected: %s", result->message.c_str());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(g_node->get_logger(), "Docking procedure started successfully");
    return BT::NodeStatus::SUCCESS;
}
