// launch_docking_procedure.hpp
#ifndef LAUNCH_DOCKING_PROCEDURE_HPP
#define LAUNCH_DOCKING_PROCEDURE_HPP

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
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
    // Stops what is left of the docking nodes in the script's process group
    void stopGroup(pid_t group);
    // The drone keeps the last motion command: zeroed whenever the docking controller stops
    void stopThrusters();
    // Depth and heading hold, then station keeping, on (the docking step switched them off): the drone holds while
    // the operator decides
    void holdPosition();

    pid_t pid_ = -1;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr cmd_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr depth_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr heading_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr stationkeep_client_;
};

#endif // LAUNCH_DOCKING_PROCEDURE_HPP
