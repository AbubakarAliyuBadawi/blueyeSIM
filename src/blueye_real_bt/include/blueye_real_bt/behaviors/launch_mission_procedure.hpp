#ifndef LAUNCH_MISSION_PROCEDURE_HPP
#define LAUNCH_MISSION_PROCEDURE_HPP

#include <string>
#include <sys/types.h>
#include <atomic>
#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

extern rclcpp::Node::SharedPtr g_node;
extern std::atomic<bool> g_program_running;
extern bool g_mission_paused;

class LaunchMissionProcedure : public BT::StatefulActionNode
{
public:
    LaunchMissionProcedure(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("script_path", "Path to the mission script"),
            BT::InputPort<std::string>("drone_ip", "IP address of the drone"),
            BT::InputPort<int>("max_retries", "Maximum number of retry attempts for aborted missions")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

    // Called at BT shutdown to kill any script that was left alive (e.g. after REJECT path)
    static void cleanup_if_needed();

private:
    // Static so state survives tree.haltTree() + re-tick after takeover handback
    inline static pid_t pid_ = -1;
    inline static bool script_paused_ = false;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;
};

#endif // LAUNCH_MISSION_PROCEDURE_HPP
