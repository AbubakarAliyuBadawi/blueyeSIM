#include "blueye_stonefish_bt/behaviors/navigate_to_waypoint.hpp"
#include "blueye_stonefish_bt/behaviors/safe_navigate_to_waypoint.hpp"
#include "blueye_stonefish_bt/behaviors/station_keeping.hpp"
#include "blueye_stonefish_bt/behaviors/launch_docking_procedure.hpp"
#include "blueye_stonefish_bt/behaviors/wait_node.hpp"
#include "blueye_stonefish_bt/conditions/battery_condition.hpp"
#include "blueye_stonefish_bt/conditions/camera_condition.hpp"
#include "blueye_stonefish_bt/conditions/sonar_condition.hpp"
#include "blueye_stonefish_bt/conditions/system_watchdog.hpp"
#include "blueye_stonefish_bt/conditions/blackboard_condition.hpp"
#include "blueye_stonefish_bt/conditions/check_takeover_request.hpp"
#include "blueye_stonefish_bt/actions/altitude_control_action.hpp"
#include "blueye_stonefish_bt/actions/publish_state.hpp"
#include "blueye_stonefish_bt/actions/wait_for_human_decision.hpp"
#include "blueye_stonefish_bt/actions/enable_joystick.hpp"
#include "blueye_stonefish_bt/actions/wait_for_handback.hpp"
#include "blueye_stonefish_bt/decorators/abort_on_condition_decorator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <std_msgs/msg/int32.hpp>
#include <memory>
#include <chrono>
#include <thread>
#include <unistd.h>

rclcpp::Node::SharedPtr g_node;
std::atomic<bool> g_program_running{true};
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr g_mission_state_pub;
int g_current_mission_state = 0;

void signalHandler(int signum) {
    if (g_node) RCLCPP_INFO(g_node->get_logger(), "Signal %d received — shutting down", signum);
    g_program_running = false;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);
    rclcpp::init(argc, argv);

    g_node = rclcpp::Node::make_shared("blueye_stonefish_bt");
    g_node->declare_parameter("behavior_tree_path", "");
    g_mission_state_pub = g_node->create_publisher<std_msgs::msg::Int32>("/mission_state", 10);

    // Continuous mission-state publishing thread
    std::thread([&]() {
        rclcpp::Rate rate(10);
        while (rclcpp::ok() && g_program_running) {
            auto msg = std::make_unique<std_msgs::msg::Int32>();
            msg->data = g_current_mission_state;
            g_mission_state_pub->publish(*msg);
            rate.sleep();
        }
    }).detach();

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<BT::RetryNode>("RetryNode");
    factory.registerNodeType<CheckBlackboard>("CheckBlackboard");
    factory.registerNodeType<AbortOnCondition>("AbortOnCondition");
    factory.registerNodeType<BT::AlwaysFailureNode>("AlwaysFail");

    factory.registerBuilder<NavigateToWaypoint>("NavigateToWaypoint",
        [](const std::string& n, const BT::NodeConfig& c) { return std::make_unique<NavigateToWaypoint>(n, c); });
    factory.registerBuilder<SafeNavigateToWaypoint>("SafeNavigateToWaypoint",
        [](const std::string& n, const BT::NodeConfiguration& c) { return std::make_unique<SafeNavigateToWaypoint>(n, c); });
    factory.registerBuilder<StationKeeping>("StationKeeping",
        [](const std::string& n, const BT::NodeConfig& c) { return std::make_unique<StationKeeping>(n, c); });
    factory.registerBuilder<LaunchDockingProcedure>("LaunchDockingProcedure",
        [](const std::string& n, const BT::NodeConfig& c) { return std::make_unique<LaunchDockingProcedure>(n, c); });
    factory.registerBuilder<Wait>("Wait",
        [](const std::string& n, const BT::NodeConfig& c) { return std::make_unique<Wait>(n, c); });
    factory.registerBuilder<CheckCameraStatus>("CheckCameraStatus",
        [](const std::string& n, const BT::NodeConfig& c) { return std::make_unique<CheckCameraStatus>(n, c); });
    factory.registerBuilder<CheckSonarStatus>("CheckSonarStatus",
        [](const std::string& n, const BT::NodeConfig& c) { return std::make_unique<CheckSonarStatus>(n, c); });
    factory.registerBuilder<SystemWatchdog>("SystemWatchdog",
        [](const std::string& n, const BT::NodeConfig& c) { return std::make_unique<SystemWatchdog>(n, c); });
    factory.registerBuilder<CheckBatteryLevel>("CheckBatteryLevel",
        [](const std::string& n, const BT::NodeConfig& c) { return std::make_unique<CheckBatteryLevel>(n, c); });
    factory.registerBuilder<AltitudeControlAction>("AltitudeControlAction",
        [](const std::string& n, const BT::NodeConfig& c) { return std::make_unique<AltitudeControlAction>(n, c); });
    factory.registerBuilder<PublishState>("PublishState",
        [](const std::string& n, const BT::NodeConfig& c) { return std::make_unique<PublishState>(n, c); });
    factory.registerNodeType<CheckTakeoverRequest>("CheckTakeoverRequest");
    factory.registerNodeType<WaitForHumanDecision>("WaitForHumanDecision");
    factory.registerNodeType<EnableJoystick>("EnableJoystick");
    factory.registerNodeType<WaitForHandback>("WaitForHandback");

    try {
        std::string mission_file;
        g_node->get_parameter("behavior_tree_path", mission_file);
        if (mission_file.empty())
            throw std::runtime_error("behavior_tree_path parameter not set");
        if (access(mission_file.c_str(), F_OK) == -1)
            throw std::runtime_error("Behavior tree file not found: " + mission_file);

        RCLCPP_INFO(g_node->get_logger(), "Loading BT from: %s", mission_file.c_str());
        auto tree = factory.createTreeFromFile(mission_file);

        BT::Groot2Publisher publisher(tree, 6677);
        RCLCPP_INFO(g_node->get_logger(), "Groot2 monitor on port 6677");

        while (rclcpp::ok() && g_program_running) {
            auto status = tree.tickOnce();
            rclcpp::spin_some(g_node);
            if (BT::isStatusCompleted(status)) {
                RCLCPP_INFO(g_node->get_logger(), "Mission finished: %s",
                            status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        tree.haltTree();
    } catch (const std::exception& e) {
        if (g_node) RCLCPP_ERROR(g_node->get_logger(), "Exception: %s", e.what());
        g_node.reset();
        rclcpp::shutdown();
        return 1;
    }

    g_program_running = false;
    g_node.reset();
    rclcpp::shutdown();
    return 0;
}
