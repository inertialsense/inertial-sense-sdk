// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/DIDINS1.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/didins1.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS1__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS1__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/didins1__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_DIDINS1_ned
{
public:
  explicit Init_DIDINS1_ned(::inertial_sense_ros2::msg::DIDINS1 & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::DIDINS1 ned(::inertial_sense_ros2::msg::DIDINS1::_ned_type arg)
  {
    msg_.ned = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS1 msg_;
};

class Init_DIDINS1_lla
{
public:
  explicit Init_DIDINS1_lla(::inertial_sense_ros2::msg::DIDINS1 & msg)
  : msg_(msg)
  {}
  Init_DIDINS1_ned lla(::inertial_sense_ros2::msg::DIDINS1::_lla_type arg)
  {
    msg_.lla = std::move(arg);
    return Init_DIDINS1_ned(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS1 msg_;
};

class Init_DIDINS1_uvw
{
public:
  explicit Init_DIDINS1_uvw(::inertial_sense_ros2::msg::DIDINS1 & msg)
  : msg_(msg)
  {}
  Init_DIDINS1_lla uvw(::inertial_sense_ros2::msg::DIDINS1::_uvw_type arg)
  {
    msg_.uvw = std::move(arg);
    return Init_DIDINS1_lla(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS1 msg_;
};

class Init_DIDINS1_theta
{
public:
  explicit Init_DIDINS1_theta(::inertial_sense_ros2::msg::DIDINS1 & msg)
  : msg_(msg)
  {}
  Init_DIDINS1_uvw theta(::inertial_sense_ros2::msg::DIDINS1::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_DIDINS1_uvw(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS1 msg_;
};

class Init_DIDINS1_hdw_status
{
public:
  explicit Init_DIDINS1_hdw_status(::inertial_sense_ros2::msg::DIDINS1 & msg)
  : msg_(msg)
  {}
  Init_DIDINS1_theta hdw_status(::inertial_sense_ros2::msg::DIDINS1::_hdw_status_type arg)
  {
    msg_.hdw_status = std::move(arg);
    return Init_DIDINS1_theta(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS1 msg_;
};

class Init_DIDINS1_ins_status
{
public:
  explicit Init_DIDINS1_ins_status(::inertial_sense_ros2::msg::DIDINS1 & msg)
  : msg_(msg)
  {}
  Init_DIDINS1_hdw_status ins_status(::inertial_sense_ros2::msg::DIDINS1::_ins_status_type arg)
  {
    msg_.ins_status = std::move(arg);
    return Init_DIDINS1_hdw_status(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS1 msg_;
};

class Init_DIDINS1_time_of_week
{
public:
  explicit Init_DIDINS1_time_of_week(::inertial_sense_ros2::msg::DIDINS1 & msg)
  : msg_(msg)
  {}
  Init_DIDINS1_ins_status time_of_week(::inertial_sense_ros2::msg::DIDINS1::_time_of_week_type arg)
  {
    msg_.time_of_week = std::move(arg);
    return Init_DIDINS1_ins_status(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS1 msg_;
};

class Init_DIDINS1_week
{
public:
  explicit Init_DIDINS1_week(::inertial_sense_ros2::msg::DIDINS1 & msg)
  : msg_(msg)
  {}
  Init_DIDINS1_time_of_week week(::inertial_sense_ros2::msg::DIDINS1::_week_type arg)
  {
    msg_.week = std::move(arg);
    return Init_DIDINS1_time_of_week(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS1 msg_;
};

class Init_DIDINS1_header
{
public:
  Init_DIDINS1_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DIDINS1_week header(::inertial_sense_ros2::msg::DIDINS1::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DIDINS1_week(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS1 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::DIDINS1>()
{
  return inertial_sense_ros2::msg::builder::Init_DIDINS1_header();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS1__BUILDER_HPP_
