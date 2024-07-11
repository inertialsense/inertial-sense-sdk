// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:srv/RefLLAUpdate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/srv/ref_lla_update.hpp"


#ifndef INERTIAL_SENSE_ROS2__SRV__DETAIL__REF_LLA_UPDATE__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__SRV__DETAIL__REF_LLA_UPDATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/srv/detail/ref_lla_update__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace srv
{

namespace builder
{

class Init_RefLLAUpdate_Request_lla
{
public:
  Init_RefLLAUpdate_Request_lla()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::inertial_sense_ros2::srv::RefLLAUpdate_Request lla(::inertial_sense_ros2::srv::RefLLAUpdate_Request::_lla_type arg)
  {
    msg_.lla = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::srv::RefLLAUpdate_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::srv::RefLLAUpdate_Request>()
{
  return inertial_sense_ros2::srv::builder::Init_RefLLAUpdate_Request_lla();
}

}  // namespace inertial_sense_ros2


namespace inertial_sense_ros2
{

namespace srv
{

namespace builder
{

class Init_RefLLAUpdate_Response_message
{
public:
  explicit Init_RefLLAUpdate_Response_message(::inertial_sense_ros2::srv::RefLLAUpdate_Response & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::srv::RefLLAUpdate_Response message(::inertial_sense_ros2::srv::RefLLAUpdate_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::srv::RefLLAUpdate_Response msg_;
};

class Init_RefLLAUpdate_Response_success
{
public:
  Init_RefLLAUpdate_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RefLLAUpdate_Response_message success(::inertial_sense_ros2::srv::RefLLAUpdate_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_RefLLAUpdate_Response_message(msg_);
  }

private:
  ::inertial_sense_ros2::srv::RefLLAUpdate_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::srv::RefLLAUpdate_Response>()
{
  return inertial_sense_ros2::srv::builder::Init_RefLLAUpdate_Response_success();
}

}  // namespace inertial_sense_ros2


namespace inertial_sense_ros2
{

namespace srv
{

namespace builder
{

class Init_RefLLAUpdate_Event_response
{
public:
  explicit Init_RefLLAUpdate_Event_response(::inertial_sense_ros2::srv::RefLLAUpdate_Event & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::srv::RefLLAUpdate_Event response(::inertial_sense_ros2::srv::RefLLAUpdate_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::srv::RefLLAUpdate_Event msg_;
};

class Init_RefLLAUpdate_Event_request
{
public:
  explicit Init_RefLLAUpdate_Event_request(::inertial_sense_ros2::srv::RefLLAUpdate_Event & msg)
  : msg_(msg)
  {}
  Init_RefLLAUpdate_Event_response request(::inertial_sense_ros2::srv::RefLLAUpdate_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_RefLLAUpdate_Event_response(msg_);
  }

private:
  ::inertial_sense_ros2::srv::RefLLAUpdate_Event msg_;
};

class Init_RefLLAUpdate_Event_info
{
public:
  Init_RefLLAUpdate_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RefLLAUpdate_Event_request info(::inertial_sense_ros2::srv::RefLLAUpdate_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RefLLAUpdate_Event_request(msg_);
  }

private:
  ::inertial_sense_ros2::srv::RefLLAUpdate_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::srv::RefLLAUpdate_Event>()
{
  return inertial_sense_ros2::srv::builder::Init_RefLLAUpdate_Event_info();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__SRV__DETAIL__REF_LLA_UPDATE__BUILDER_HPP_
