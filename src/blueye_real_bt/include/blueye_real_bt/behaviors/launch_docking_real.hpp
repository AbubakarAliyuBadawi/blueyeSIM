// launch_docking_procedure.hpp
#ifndef LAUNCH_DOCKING_PROCEDURE_HPP
#define LAUNCH_DOCKING_PROCEDURE_HPP

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/types.h>
#include <atomic>

extern rclcpp::Node::SharedPtr g_node;
extern std::atomic<bool> g_program_running;

class LaunchDockingProcedure : public BT::StatefulActionNode {
public:
    LaunchDockingProcedure(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    pid_t pid_ = -1;
};

#endif // LAUNCH_DOCKING_PROCEDURE_HPP
