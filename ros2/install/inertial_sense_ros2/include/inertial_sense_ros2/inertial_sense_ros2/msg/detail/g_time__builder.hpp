// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/GTime.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/g_time.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__G_TIME__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__G_TIME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/g_time__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_GTime_sec
{
public:
  explicit Init_GTime_sec(::inertial_sense_ros2::msg::GTime & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::GTime sec(::inertial_sense_ros2::msg::GTime::_sec_type arg)
  {
    msg_.sec = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GTime msg_;
};

class Init_GTime_time
{
public:
  Init_GTime_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GTime_sec time(::inertial_sense_ros2::msg::GTime::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_GTime_sec(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GTime msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::GTime>()
{
  return inertial_sense_ros2::msg::builder::Init_GTime_time();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__G_TIME__BUILDER_HPP_
