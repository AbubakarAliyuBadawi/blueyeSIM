#pragma once
#include "blueye_stonefish_bt/behaviors/navigate_to_waypoint.hpp"

// Simplified SafeNavigateToWaypoint for Stonefish.
// Accepts the same ports as the original (including obstacle-avoidance parameters)
// but delegates entirely to NavigateToWaypoint — obstacle avoidance can be wired in later.
class SafeNavigateToWaypoint : public NavigateToWaypoint {
public:
    SafeNavigateToWaypoint(const std::string& name, const BT::NodeConfiguration& config)
        : NavigateToWaypoint(name, config) {}

    static BT::PortsList providedPorts() {
        auto ports = NavigateToWaypoint::providedPorts();
        ports.insert(BT::InputPort<double>("safety_distance", 2.0, "Min distance to obstacles (unused)"));
        ports.insert(BT::InputPort<double>("deviation_distance", 10.0, "Deviation point distance (unused)"));
        ports.insert(BT::InputPort<double>("deviation_velocity", 0.3, "Avoidance velocity (unused)"));
        ports.insert(BT::InputPort<bool>("obstacle_avoidance_enabled", false, "Enable obstacle avoidance (unused)"));
        ports.insert(BT::InputPort<double>("waypoint_resume_delay", 0.1, "Resume delay (unused)"));
        return ports;
    }
};
