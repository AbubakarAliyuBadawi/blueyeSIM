// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vortex_msgs:action/FilteredLandmarks.idl
// generated code does not contain a copyright notice

#ifndef VORTEX_MSGS__ACTION__DETAIL__FILTERED_LANDMARKS__BUILDER_HPP_
#define VORTEX_MSGS__ACTION__DETAIL__FILTERED_LANDMARKS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vortex_msgs/action/detail/filtered_landmarks__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredLandmarks_Goal_distance
{
public:
  explicit Init_FilteredLandmarks_Goal_distance(::vortex_msgs::action::FilteredLandmarks_Goal & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::FilteredLandmarks_Goal distance(::vortex_msgs::action::FilteredLandmarks_Goal::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_Goal msg_;
};

class Init_FilteredLandmarks_Goal_landmark_types
{
public:
  Init_FilteredLandmarks_Goal_landmark_types()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FilteredLandmarks_Goal_distance landmark_types(::vortex_msgs::action::FilteredLandmarks_Goal::_landmark_types_type arg)
  {
    msg_.landmark_types = std::move(arg);
    return Init_FilteredLandmarks_Goal_distance(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredLandmarks_Goal>()
{
  return vortex_msgs::action::builder::Init_FilteredLandmarks_Goal_landmark_types();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredLandmarks_Result_result
{
public:
  Init_FilteredLandmarks_Result_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::action::FilteredLandmarks_Result result(::vortex_msgs::action::FilteredLandmarks_Result::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredLandmarks_Result>()
{
  return vortex_msgs::action::builder::Init_FilteredLandmarks_Result_result();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredLandmarks_Feedback_feedback
{
public:
  Init_FilteredLandmarks_Feedback_feedback()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::action::FilteredLandmarks_Feedback feedback(::vortex_msgs::action::FilteredLandmarks_Feedback::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredLandmarks_Feedback>()
{
  return vortex_msgs::action::builder::Init_FilteredLandmarks_Feedback_feedback();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredLandmarks_SendGoal_Request_goal
{
public:
  explicit Init_FilteredLandmarks_SendGoal_Request_goal(::vortex_msgs::action::FilteredLandmarks_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::FilteredLandmarks_SendGoal_Request goal(::vortex_msgs::action::FilteredLandmarks_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_SendGoal_Request msg_;
};

class Init_FilteredLandmarks_SendGoal_Request_goal_id
{
public:
  Init_FilteredLandmarks_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FilteredLandmarks_SendGoal_Request_goal goal_id(::vortex_msgs::action::FilteredLandmarks_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FilteredLandmarks_SendGoal_Request_goal(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredLandmarks_SendGoal_Request>()
{
  return vortex_msgs::action::builder::Init_FilteredLandmarks_SendGoal_Request_goal_id();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredLandmarks_SendGoal_Response_stamp
{
public:
  explicit Init_FilteredLandmarks_SendGoal_Response_stamp(::vortex_msgs::action::FilteredLandmarks_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::FilteredLandmarks_SendGoal_Response stamp(::vortex_msgs::action::FilteredLandmarks_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_SendGoal_Response msg_;
};

class Init_FilteredLandmarks_SendGoal_Response_accepted
{
public:
  Init_FilteredLandmarks_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FilteredLandmarks_SendGoal_Response_stamp accepted(::vortex_msgs::action::FilteredLandmarks_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_FilteredLandmarks_SendGoal_Response_stamp(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredLandmarks_SendGoal_Response>()
{
  return vortex_msgs::action::builder::Init_FilteredLandmarks_SendGoal_Response_accepted();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredLandmarks_GetResult_Request_goal_id
{
public:
  Init_FilteredLandmarks_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::vortex_msgs::action::FilteredLandmarks_GetResult_Request goal_id(::vortex_msgs::action::FilteredLandmarks_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredLandmarks_GetResult_Request>()
{
  return vortex_msgs::action::builder::Init_FilteredLandmarks_GetResult_Request_goal_id();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredLandmarks_GetResult_Response_result
{
public:
  explicit Init_FilteredLandmarks_GetResult_Response_result(::vortex_msgs::action::FilteredLandmarks_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::FilteredLandmarks_GetResult_Response result(::vortex_msgs::action::FilteredLandmarks_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_GetResult_Response msg_;
};

class Init_FilteredLandmarks_GetResult_Response_status
{
public:
  Init_FilteredLandmarks_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FilteredLandmarks_GetResult_Response_result status(::vortex_msgs::action::FilteredLandmarks_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_FilteredLandmarks_GetResult_Response_result(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredLandmarks_GetResult_Response>()
{
  return vortex_msgs::action::builder::Init_FilteredLandmarks_GetResult_Response_status();
}

}  // namespace vortex_msgs


namespace vortex_msgs
{

namespace action
{

namespace builder
{

class Init_FilteredLandmarks_FeedbackMessage_feedback
{
public:
  explicit Init_FilteredLandmarks_FeedbackMessage_feedback(::vortex_msgs::action::FilteredLandmarks_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::vortex_msgs::action::FilteredLandmarks_FeedbackMessage feedback(::vortex_msgs::action::FilteredLandmarks_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_FeedbackMessage msg_;
};

class Init_FilteredLandmarks_FeedbackMessage_goal_id
{
public:
  Init_FilteredLandmarks_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FilteredLandmarks_FeedbackMessage_feedback goal_id(::vortex_msgs::action::FilteredLandmarks_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FilteredLandmarks_FeedbackMessage_feedback(msg_);
  }

private:
  ::vortex_msgs::action::FilteredLandmarks_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::vortex_msgs::action::FilteredLandmarks_FeedbackMessage>()
{
  return vortex_msgs::action::builder::Init_FilteredLandmarks_FeedbackMessage_goal_id();
}

}  // namespace vortex_msgs

#endif  // VORTEX_MSGS__ACTION__DETAIL__FILTERED_LANDMARKS__BUILDER_HPP_
