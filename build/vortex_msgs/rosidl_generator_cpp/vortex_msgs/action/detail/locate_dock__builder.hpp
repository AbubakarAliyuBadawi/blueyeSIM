// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:action/LocateDock.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__ACTION__DETAIL__LOCATE_DOCK__BUILDER_HPP_
#define VORTEX_MSGS__ACTION__DETAIL__LOCATE_DOCK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/action/detail/locate_dock__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_LocateDock_Goal_start_search
{
public:
  Init_LocateDock_Goal_start_search()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::action::LocateDock_Goal start_search(::vortex_msgs::action::LocateDock_Goal::_start_search_type arg)
  {
    msg_.start_search = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::LocateDock_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::LocateDock_Goal>()
{
  return vortex_msgs::action::builder::Init_LocateDock_Goal_start_search();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_LocateDock_Result_board_pose
{
public:
  Init_LocateDock_Result_board_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::action::LocateDock_Result board_pose(::vortex_msgs::action::LocateDock_Result::_board_pose_type arg)
  {
    msg_.board_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::LocateDock_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::LocateDock_Result>()
{
  return vortex_msgs::action::builder::Init_LocateDock_Result_board_pose();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_LocateDock_Feedback_confirmed
{
public:
  Init_LocateDock_Feedback_confirmed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::action::LocateDock_Feedback confirmed(::vortex_msgs::action::LocateDock_Feedback::_confirmed_type arg)
  {
    msg_.confirmed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::LocateDock_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::LocateDock_Feedback>()
{
  return vortex_msgs::action::builder::Init_LocateDock_Feedback_confirmed();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_LocateDock_SendGoal_Request_goal
{
public:
  explicit Init_LocateDock_SendGoal_Request_goal(::vortex_msgs::action::LocateDock_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::LocateDock_SendGoal_Request goal(::vortex_msgs::action::LocateDock_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::LocateDock_SendGoal_Request msg_;
};

class Init_LocateDock_SendGoal_Request_goal_id
{
public:
  Init_LocateDock_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocateDock_SendGoal_Request_goal goal_id(::vortex_msgs::action::LocateDock_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_LocateDock_SendGoal_Request_goal(msg_);
  }

private:
  ::vortex_msgs::action::LocateDock_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::LocateDock_SendGoal_Request>()
{
  return vortex_msgs::action::builder::Init_LocateDock_SendGoal_Request_goal_id();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_LocateDock_SendGoal_Response_stamp
{
public:
  explicit Init_LocateDock_SendGoal_Response_stamp(::vortex_msgs::action::LocateDock_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::LocateDock_SendGoal_Response stamp(::vortex_msgs::action::LocateDock_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::LocateDock_SendGoal_Response msg_;
};

class Init_LocateDock_SendGoal_Response_accepted
{
public:
  Init_LocateDock_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocateDock_SendGoal_Response_stamp accepted(::vortex_msgs::action::LocateDock_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_LocateDock_SendGoal_Response_stamp(msg_);
  }

private:
  ::vortex_msgs::action::LocateDock_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::LocateDock_SendGoal_Response>()
{
  return vortex_msgs::action::builder::Init_LocateDock_SendGoal_Response_accepted();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_LocateDock_GetResult_Request_goal_id
{
public:
  Init_LocateDock_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::action::LocateDock_GetResult_Request goal_id(::vortex_msgs::action::LocateDock_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::LocateDock_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::LocateDock_GetResult_Request>()
{
  return vortex_msgs::action::builder::Init_LocateDock_GetResult_Request_goal_id();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_LocateDock_GetResult_Response_result
{
public:
  explicit Init_LocateDock_GetResult_Response_result(::vortex_msgs::action::LocateDock_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::LocateDock_GetResult_Response result(::vortex_msgs::action::LocateDock_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::LocateDock_GetResult_Response msg_;
};

class Init_LocateDock_GetResult_Response_status
{
public:
  Init_LocateDock_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocateDock_GetResult_Response_result status(::vortex_msgs::action::LocateDock_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_LocateDock_GetResult_Response_result(msg_);
  }

private:
  ::vortex_msgs::action::LocateDock_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::LocateDock_GetResult_Response>()
{
  return vortex_msgs::action::builder::Init_LocateDock_GetResult_Response_status();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_LocateDock_FeedbackMessage_feedback
{
public:
  explicit Init_LocateDock_FeedbackMessage_feedback(::vortex_msgs::action::LocateDock_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::LocateDock_FeedbackMessage feedback(::vortex_msgs::action::LocateDock_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::LocateDock_FeedbackMessage msg_;
};

class Init_LocateDock_FeedbackMessage_goal_id
{
public:
  Init_LocateDock_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocateDock_FeedbackMessage_feedback goal_id(::vortex_msgs::action::LocateDock_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_LocateDock_FeedbackMessage_feedback(msg_);
  }

private:
  ::vortex_msgs::action::LocateDock_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::LocateDock_FeedbackMessage>()
{
  return vortex_msgs::action::builder::Init_LocateDock_FeedbackMessage_goal_id();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__ACTION__DETAIL__LOCATE_DOCK__BUILDER_HPP_
