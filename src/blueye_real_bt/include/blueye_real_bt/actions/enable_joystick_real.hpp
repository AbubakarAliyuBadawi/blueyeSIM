#pragma once
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

extern rclcpp::Node::SharedPtr g_node;
// Set true here (ACCEPT path) so main.cpp knows to restart the tree after handback.
// Never set in the REJECT path, so emergency dock exits cleanly without a restart.
extern bool g_mission_paused;

// Disables depth_hold and heading_hold so the operator has full joystick control.
class EnableJoystick : public BT::SyncActionNode {
public:
    EnableJoystick(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        depth_client_   = g_node->create_client<std_srvs::srv::SetBool>("/blueye/depth_hold");
        heading_client_ = g_node->create_client<std_srvs::srv::SetBool>("/blueye/heading_hold");
    }

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override {
        bool ok = _call(depth_client_,   false, "/blueye/depth_hold")
               && _call(heading_client_, false, "/blueye/heading_hold");
        if (ok) {
            g_mission_paused = true;
            RCLCPP_INFO(g_node->get_logger(),
                "EnableJoystick: auto modes disabled — joystick active, mission resume armed");
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_ERROR(g_node->get_logger(), "EnableJoystick: failed to disable one or more auto modes");
        return BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr depth_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr heading_client_;

    bool _call(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client,
               bool value, const std::string& name)
    {
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(g_node->get_logger(), "EnableJoystick: service not available: %s", name.c_str());
            return false;
        }
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = value;
        auto future = client->async_send_request(req);
        if (rclcpp::spin_until_future_complete(g_node, future, std::chrono::seconds(3))
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(g_node->get_logger(), "EnableJoystick: call timed out: %s", name.c_str());
            return false;
        }
        return true;
    }
};
