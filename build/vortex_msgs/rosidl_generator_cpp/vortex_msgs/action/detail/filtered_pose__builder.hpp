// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:action/FilteredPose.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__ACTION__DETAIL__FILTERED_POSE__BUILDER_HPP_
#define VORTEX_MSGS__ACTION__DETAIL__FILTERED_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/action/detail/filtered_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredPose_Goal_num_measurements
{
public:
  Init_FilteredPose_Goal_num_measurements()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::action::FilteredPose_Goal num_measurements(::vortex_msgs::action::FilteredPose_Goal::_num_measurements_type arg)
  {
    msg_.num_measurements = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredPose_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredPose_Goal>()
{
  return vortex_msgs::action::builder::Init_FilteredPose_Goal_num_measurements();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredPose_Result_filtered_pose
{
public:
  Init_FilteredPose_Result_filtered_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::action::FilteredPose_Result filtered_pose(::vortex_msgs::action::FilteredPose_Result::_filtered_pose_type arg)
  {
    msg_.filtered_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredPose_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredPose_Result>()
{
  return vortex_msgs::action::builder::Init_FilteredPose_Result_filtered_pose();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredPose_Feedback_current_pose
{
public:
  Init_FilteredPose_Feedback_current_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::action::FilteredPose_Feedback current_pose(::vortex_msgs::action::FilteredPose_Feedback::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredPose_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredPose_Feedback>()
{
  return vortex_msgs::action::builder::Init_FilteredPose_Feedback_current_pose();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredPose_SendGoal_Request_goal
{
public:
  explicit Init_FilteredPose_SendGoal_Request_goal(::vortex_msgs::action::FilteredPose_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::FilteredPose_SendGoal_Request goal(::vortex_msgs::action::FilteredPose_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredPose_SendGoal_Request msg_;
};

class Init_FilteredPose_SendGoal_Request_goal_id
{
public:
  Init_FilteredPose_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FilteredPose_SendGoal_Request_goal goal_id(::vortex_msgs::action::FilteredPose_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FilteredPose_SendGoal_Request_goal(msg_);
  }

private:
  ::vortex_msgs::action::FilteredPose_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredPose_SendGoal_Request>()
{
  return vortex_msgs::action::builder::Init_FilteredPose_SendGoal_Request_goal_id();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredPose_SendGoal_Response_stamp
{
public:
  explicit Init_FilteredPose_SendGoal_Response_stamp(::vortex_msgs::action::FilteredPose_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::FilteredPose_SendGoal_Response stamp(::vortex_msgs::action::FilteredPose_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredPose_SendGoal_Response msg_;
};

class Init_FilteredPose_SendGoal_Response_accepted
{
public:
  Init_FilteredPose_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FilteredPose_SendGoal_Response_stamp accepted(::vortex_msgs::action::FilteredPose_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_FilteredPose_SendGoal_Response_stamp(msg_);
  }

private:
  ::vortex_msgs::action::FilteredPose_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredPose_SendGoal_Response>()
{
  return vortex_msgs::action::builder::Init_FilteredPose_SendGoal_Response_accepted();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredPose_GetResult_Request_goal_id
{
public:
  Init_FilteredPose_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::action::FilteredPose_GetResult_Request goal_id(::vortex_msgs::action::FilteredPose_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredPose_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredPose_GetResult_Request>()
{
  return vortex_msgs::action::builder::Init_FilteredPose_GetResult_Request_goal_id();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredPose_GetResult_Response_result
{
public:
  explicit Init_FilteredPose_GetResult_Response_result(::vortex_msgs::action::FilteredPose_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::FilteredPose_GetResult_Response result(::vortex_msgs::action::FilteredPose_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredPose_GetResult_Response msg_;
};

class Init_FilteredPose_GetResult_Response_status
{
public:
  Init_FilteredPose_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FilteredPose_GetResult_Response_result status(::vortex_msgs::action::FilteredPose_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_FilteredPose_GetResult_Response_result(msg_);
  }

private:
  ::vortex_msgs::action::FilteredPose_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredPose_GetResult_Response>()
{
  return vortex_msgs::action::builder::Init_FilteredPose_GetResult_Response_status();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredPose_FeedbackMessage_feedback
{
public:
  explicit Init_FilteredPose_FeedbackMessage_feedback(::vortex_msgs::action::FilteredPose_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::FilteredPose_FeedbackMessage feedback(::vortex_msgs::action::FilteredPose_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredPose_FeedbackMessage msg_;
};

class Init_FilteredPose_FeedbackMessage_goal_id
{
public:
  Init_FilteredPose_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FilteredPose_FeedbackMessage_feedback goal_id(::vortex_msgs::action::FilteredPose_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FilteredPose_FeedbackMessage_feedback(msg_);
  }

private:
  ::vortex_msgs::action::FilteredPose_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredPose_FeedbackMessage>()
{
  return vortex_msgs::action::builder::Init_FilteredPose_FeedbackMessage_goal_id();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__ACTION__DETAIL__FILTERED_POSE__BUILDER_HPP_
