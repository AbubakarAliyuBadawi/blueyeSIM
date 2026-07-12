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
#include "blueye_real_bt/behaviors/launch_mission_procedure.hpp"
#include "blueye_real_bt/behaviors/wait_node_real.hpp"
#include "blueye_real_bt/behaviors/launch_docking_real.hpp"
#include "blueye_real_bt/conditions/BatteryLevelCondition.hpp"
#include "blueye_real_bt/behaviors/goto_waypoint.hpp"
#include "blueye_real_bt/behaviors/goto_waypoint_cc.hpp"
#include "blueye_real_bt/behaviors/publish_state.hpp"
#include "blueye_real_bt/behaviors/activate_auto_modes.hpp"
#include "blueye_real_bt/conditions/CheckBlackboardBool.hpp"
#include "blueye_real_bt/behaviors/launch_undocking_real.hpp"
#include "blueye_real_bt/conditions/check_takeover_request.hpp"
#include "blueye_real_bt/actions/wait_for_human_decision.hpp"
#include "blueye_real_bt/actions/enable_joystick_real.hpp"
#include "blueye_real_bt/actions/wait_for_handback_real.hpp"

// Global node for service clients
rclcpp::Node::SharedPtr g_node;
std::atomic<bool> g_program_running{true};
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr g_mission_state_pub;
int g_current_mission_state = 0;
// Set by LaunchMissionProcedure::onHalted when mission is paused for takeover;
// cleared by the main loop after it resets the tree so the mission can resume.
bool g_mission_paused = false;

void signalHandler(int signum) {
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Interrupt signal (%d) received.", signum);
    }
    g_program_running = false;
}



int main(int argc, char **argv) {
    signal(SIGINT, signalHandler);
    
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("blueye_real_bt");
    g_node->declare_parameter("behavior_tree_path", "");
    g_mission_state_pub = g_node->create_publisher<std_msgs::msg::Int32>("/mission_state", 10);
    BT::BehaviorTreeFactory factory;

    // Register basic BT nodes
    factory.registerNodeType<BT::RetryNode>("RetryNode");
    factory.registerNodeType<BT::SequenceNode>("SequenceNode");
    factory.registerNodeType<BT::AlwaysFailureNode>("AlwaysFail");
    factory.registerNodeType<BT::ParallelNode>("ParallelNode");
    
    // Register LaunchDockingProcedure
    factory.registerBuilder<LaunchMissionProcedure>(
        "LaunchMissionProcedure",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<LaunchMissionProcedure>(name, config);
        });

    // Register Wait
    factory.registerBuilder<Wait>(
        "Wait",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<Wait>(name, config);
        });
        
    // Register the node in main()
    factory.registerBuilder<LaunchDockingProcedure>(
        "LaunchDockingProcedure",
        [](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<LaunchDockingProcedure>(name, config);
        });


    // Register BatteryLevelCondition
    factory.registerBuilder<BatteryLevelCondition>(
        "BatteryLevelCondition",
        [](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<BatteryLevelCondition>(name, config);
        });

    // Register GoToWaypoint - Add this block
    factory.registerBuilder<GoToWaypoint>(
        "GoToWaypoint",
        [](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<GoToWaypoint>(name, config);
        });

    // Register GoToWaypoint - Add this block
    factory.registerBuilder<GoToWaypointCC>(
        "GoToWaypointCC",
        [](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<GoToWaypointCC>(name, config);
        });

    // Register ActivateAutoModes
    factory.registerBuilder<ActivateAutoModes>(
        "ActivateAutoModes",
        [](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ActivateAutoModes>(name, config);
        });

    factory.registerBuilder<blueye_real_bt::CheckBlackboardBool>(
        "CheckBlackboardBool",
        [](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<blueye_real_bt::CheckBlackboardBool>(name, config);
        });



    // Register the node in main()
    factory.registerBuilder<PublishState>(
        "PublishState",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<PublishState>(name, config);
        });

    // Register LaunchUndockingProcedure
    factory.registerBuilder<LaunchUndockingProcedure>(
        "LaunchUndockingProcedure",
        [](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<LaunchUndockingProcedure>(name, config);
        });

    // Register takeover nodes
    factory.registerBuilder<CheckTakeoverRequest>(
        "CheckTakeoverRequest",
        [](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<CheckTakeoverRequest>(name, config);
        });

    factory.registerBuilder<WaitForHumanDecision>(
        "WaitForHumanDecision",
        [](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<WaitForHumanDecision>(name, config);
        });

    factory.registerBuilder<EnableJoystick>(
        "EnableJoystick",
        [](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<EnableJoystick>(name, config);
        });

    factory.registerBuilder<WaitForHandback>(
        "WaitForHandback",
        [](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<WaitForHandback>(name, config);
        });

try {
        std::string mission_file;
        if (!g_node->get_parameter("behavior_tree_path", mission_file)) {
            throw std::runtime_error("Failed to get behavior_tree_path parameter");
        }
        
        RCLCPP_INFO(g_node->get_logger(), "Loading behavior tree from: %s", mission_file.c_str());
        if (access(mission_file.c_str(), F_OK) == -1) {
            throw std::runtime_error("Behavior tree file does not exist: " + mission_file);
        }

        auto tree = factory.createTreeFromFile(mission_file);
        RCLCPP_INFO(g_node->get_logger(), "Behavior tree created successfully");

        BT::Groot2Publisher publisher(tree, 6677);
        RCLCPP_INFO(g_node->get_logger(), "Groot2 publisher created on port 6677. You can monitor the tree using Groot2");

        const auto sleep_ms = std::chrono::milliseconds(1);
        auto status = BT::NodeStatus::RUNNING;

        while (rclcpp::ok() && g_program_running) {
            status = tree.tickOnce();
            rclcpp::spin_some(g_node);
            std::this_thread::sleep_for(sleep_ms);

            if (BT::isStatusCompleted(status)) {
                if (g_mission_paused && g_program_running) {
                    // Takeover was handled (operator accepted + handed back).
                    // Reset all BT nodes to IDLE so the tree re-enters from the top.
                    // LaunchUndockingProcedure will skip itself (already_done_=true).
                    // LaunchMissionProcedure will detect the paused script and resume it.
                    RCLCPP_INFO(g_node->get_logger(),
                        "Takeover handled — resetting tree to resume paused mission");
                    g_mission_paused = false;
                    tree.haltTree();
                    status = BT::NodeStatus::RUNNING;
                    continue;
                }
                RCLCPP_INFO(g_node->get_logger(), "Tree finished with status: %s",
                           status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
                break;
            }
        }

        RCLCPP_INFO(g_node->get_logger(), "Starting clean shutdown...");
        tree.haltTree();
        LaunchMissionProcedure::cleanup_if_needed();
        RCLCPP_INFO(g_node->get_logger(), "Tree halted successfully");
        g_mission_state_pub.reset();  // Explicitly reset the publisher
        g_program_running = false;    // Make sure thread knows to stop
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(g_node->get_logger(), "Shutdown complete");

    } catch (const std::exception& e) {
        if (g_node) {
            RCLCPP_ERROR(g_node->get_logger(), "Exception caught: %s", e.what());
        }
        g_node.reset();
        rclcpp::shutdown();
        return 1;
    }

    g_node.reset();
    rclcpp::shutdown();
    return 0;
}