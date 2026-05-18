// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:action/WaypointManager.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__ACTION__DETAIL__WAYPOINT_MANAGER__BUILDER_HPP_
#define VORTEX_MSGS__ACTION__DETAIL__WAYPOINT_MANAGER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/action/detail/waypoint_manager__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_WaypointManager_Goal_switching_threshold
{
public:
  explicit Init_WaypointManager_Goal_switching_threshold(::vortex_msgs::action::WaypointManager_Goal & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::WaypointManager_Goal switching_threshold(::vortex_msgs::action::WaypointManager_Goal::_switching_threshold_type arg)
  {
    msg_.switching_threshold = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_Goal msg_;
};

class Init_WaypointManager_Goal_target_server
{
public:
  explicit Init_WaypointManager_Goal_target_server(::vortex_msgs::action::WaypointManager_Goal & msg)
  : msg_(msg)
  {}
  Init_WaypointManager_Goal_switching_threshold target_server(::vortex_msgs::action::WaypointManager_Goal::_target_server_type arg)
  {
    msg_.target_server = std::move(arg);
    return Init_WaypointManager_Goal_switching_threshold(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_Goal msg_;
};

class Init_WaypointManager_Goal_waypoints
{
public:
  Init_WaypointManager_Goal_waypoints()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointManager_Goal_target_server waypoints(::vortex_msgs::action::WaypointManager_Goal::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return Init_WaypointManager_Goal_target_server(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::WaypointManager_Goal>()
{
  return vortex_msgs::action::builder::Init_WaypointManager_Goal_waypoints();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_WaypointManager_Result_completed_waypoints
{
public:
  explicit Init_WaypointManager_Result_completed_waypoints(::vortex_msgs::action::WaypointManager_Result & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::WaypointManager_Result completed_waypoints(::vortex_msgs::action::WaypointManager_Result::_completed_waypoints_type arg)
  {
    msg_.completed_waypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_Result msg_;
};

class Init_WaypointManager_Result_success
{
public:
  Init_WaypointManager_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointManager_Result_completed_waypoints success(::vortex_msgs::action::WaypointManager_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_WaypointManager_Result_completed_waypoints(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::WaypointManager_Result>()
{
  return vortex_msgs::action::builder::Init_WaypointManager_Result_success();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_WaypointManager_Feedback_distance_to_waypoint
{
public:
  explicit Init_WaypointManager_Feedback_distance_to_waypoint(::vortex_msgs::action::WaypointManager_Feedback & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::WaypointManager_Feedback distance_to_waypoint(::vortex_msgs::action::WaypointManager_Feedback::_distance_to_waypoint_type arg)
  {
    msg_.distance_to_waypoint = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_Feedback msg_;
};

class Init_WaypointManager_Feedback_current_waypoint_index
{
public:
  explicit Init_WaypointManager_Feedback_current_waypoint_index(::vortex_msgs::action::WaypointManager_Feedback & msg)
  : msg_(msg)
  {}
  Init_WaypointManager_Feedback_distance_to_waypoint current_waypoint_index(::vortex_msgs::action::WaypointManager_Feedback::_current_waypoint_index_type arg)
  {
    msg_.current_waypoint_index = std::move(arg);
    return Init_WaypointManager_Feedback_distance_to_waypoint(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_Feedback msg_;
};

class Init_WaypointManager_Feedback_current_pose
{
public:
  Init_WaypointManager_Feedback_current_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointManager_Feedback_current_waypoint_index current_pose(::vortex_msgs::action::WaypointManager_Feedback::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_WaypointManager_Feedback_current_waypoint_index(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::WaypointManager_Feedback>()
{
  return vortex_msgs::action::builder::Init_WaypointManager_Feedback_current_pose();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_WaypointManager_SendGoal_Request_goal
{
public:
  explicit Init_WaypointManager_SendGoal_Request_goal(::vortex_msgs::action::WaypointManager_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::WaypointManager_SendGoal_Request goal(::vortex_msgs::action::WaypointManager_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_SendGoal_Request msg_;
};

class Init_WaypointManager_SendGoal_Request_goal_id
{
public:
  Init_WaypointManager_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointManager_SendGoal_Request_goal goal_id(::vortex_msgs::action::WaypointManager_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_WaypointManager_SendGoal_Request_goal(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::WaypointManager_SendGoal_Request>()
{
  return vortex_msgs::action::builder::Init_WaypointManager_SendGoal_Request_goal_id();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_WaypointManager_SendGoal_Response_stamp
{
public:
  explicit Init_WaypointManager_SendGoal_Response_stamp(::vortex_msgs::action::WaypointManager_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::WaypointManager_SendGoal_Response stamp(::vortex_msgs::action::WaypointManager_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_SendGoal_Response msg_;
};

class Init_WaypointManager_SendGoal_Response_accepted
{
public:
  Init_WaypointManager_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointManager_SendGoal_Response_stamp accepted(::vortex_msgs::action::WaypointManager_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_WaypointManager_SendGoal_Response_stamp(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::WaypointManager_SendGoal_Response>()
{
  return vortex_msgs::action::builder::Init_WaypointManager_SendGoal_Response_accepted();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_WaypointManager_GetResult_Request_goal_id
{
public:
  Init_WaypointManager_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::action::WaypointManager_GetResult_Request goal_id(::vortex_msgs::action::WaypointManager_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::WaypointManager_GetResult_Request>()
{
  return vortex_msgs::action::builder::Init_WaypointManager_GetResult_Request_goal_id();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_WaypointManager_GetResult_Response_result
{
public:
  explicit Init_WaypointManager_GetResult_Response_result(::vortex_msgs::action::WaypointManager_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::WaypointManager_GetResult_Response result(::vortex_msgs::action::WaypointManager_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_GetResult_Response msg_;
};

class Init_WaypointManager_GetResult_Response_status
{
public:
  Init_WaypointManager_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointManager_GetResult_Response_result status(::vortex_msgs::action::WaypointManager_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_WaypointManager_GetResult_Response_result(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::WaypointManager_GetResult_Response>()
{
  return vortex_msgs::action::builder::Init_WaypointManager_GetResult_Response_status();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_WaypointManager_FeedbackMessage_feedback
{
public:
  explicit Init_WaypointManager_FeedbackMessage_feedback(::vortex_msgs::action::WaypointManager_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::WaypointManager_FeedbackMessage feedback(::vortex_msgs::action::WaypointManager_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_FeedbackMessage msg_;
};

class Init_WaypointManager_FeedbackMessage_goal_id
{
public:
  Init_WaypointManager_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointManager_FeedbackMessage_feedback goal_id(::vortex_msgs::action::WaypointManager_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_WaypointManager_FeedbackMessage_feedback(msg_);
  }

private:
  ::vortex_msgs::action::WaypointManager_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::WaypointManager_FeedbackMessage>()
{
  return vortex_msgs::action::builder::Init_WaypointManager_FeedbackMessage_goal_id();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__ACTION__DETAIL__WAYPOINT_MANAGER__BUILDER_HPP_
