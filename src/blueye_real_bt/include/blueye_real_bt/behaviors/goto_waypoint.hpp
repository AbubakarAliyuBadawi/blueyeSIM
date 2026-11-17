#ifndef GOTO_WAYPOINT_HPP
#define GOTO_WAYPOINT_HPP

#include <string>
#include "behaviortree_cpp/action_node.h"

class GoToWaypoint : public BT::SyncActionNode
{
public:
    GoToWaypoint(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("drone_ip", "192.168.1.101", "IP address of the drone"),
            BT::InputPort<double>("waypoint_lat", "Target latitude"),
            BT::InputPort<double>("waypoint_lon", "Target longitude"),
            BT::InputPort<double>("depth", 1.0, "Target depth in meters"),
            BT::InputPort<std::string>("site_point", "",
                                       "Waypoint of the site layout instead of waypoint_lat/lon/depth, e.g. S")
        };
    }
    
    BT::NodeStatus tick() override;

private:
    BT::NodeStatus run(const std::string& cmd);
};

#endif // GOTO_WAYPOINT_HPP