#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "mundus_mir_msgs/srv/clear_waypoints.hpp"
#include "mundus_mir_msgs/srv/run_waypoint_controller.hpp"

extern rclcpp::Node::SharedPtr g_node;

// Holds the vehicle where it is while the operator decides on a takeover (closed loop).
// Stops the docking controller, clears the waypoint list and keeps the waypoint controller running:
// with no waypoints it station-keeps on the current pose (position and heading). Stopping the
// waypoint controller instead would leave the velocity controller on its last reference.
class HoldPosition : public BT::SyncActionNode {
public:
    HoldPosition(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        docking_client_ = g_node->create_client<std_srvs::srv::SetBool>("/blueye/start_docking");
        clear_client_ = g_node->create_client<mundus_mir_msgs::srv::ClearWaypoints>("/blueye/clear_waypoints");
        run_client_ = g_node->create_client<mundus_mir_msgs::srv::RunWaypointController>(
            "/blueye/run_waypoint_controller");
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        if (docking_client_->service_is_ready()) {          // docking controller only runs with the docking launch
            auto stop = std::make_shared<std_srvs::srv::SetBool::Request>();
            stop->data = false;
            call(docking_client_, stop, "stop the docking controller");
        }
        if (!clear_client_->wait_for_service(std::chrono::seconds(2)) ||
            !run_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(g_node->get_logger(), "HoldPosition: waypoint controller services not available");
            return BT::NodeStatus::SUCCESS;                  // still ask the operator
        }
        call(clear_client_, std::make_shared<mundus_mir_msgs::srv::ClearWaypoints::Request>(), "clear waypoints");
        auto run = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
        run->run = true;
        call(run_client_, run, "run the waypoint controller");
        RCLCPP_INFO(g_node->get_logger(), "HoldPosition: holding position while the operator decides");
        return BT::NodeStatus::SUCCESS;
    }

private:
    template <typename ClientT, typename RequestT>
    void call(ClientT& client, RequestT request, const char* what) {
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(g_node, future, std::chrono::seconds(2))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(g_node->get_logger(), "HoldPosition: failed to %s", what);
        }
    }

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr docking_client_;
    rclcpp::Client<mundus_mir_msgs::srv::ClearWaypoints>::SharedPtr clear_client_;
    rclcpp::Client<mundus_mir_msgs::srv::RunWaypointController>::SharedPtr run_client_;
};
