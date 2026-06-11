#ifndef LAUNCH_MISSION_PROCEDURE_HPP
#define LAUNCH_MISSION_PROCEDURE_HPP

#include <string>
#include <sys/types.h>
#include "behaviortree_cpp/action_node.h"

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

private:
    pid_t pid_ = -1;
};

#endif // LAUNCH_MISSION_PROCEDURE_HPP
