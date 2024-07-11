// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/DIDINS2.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/didins2.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS2__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS2__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/didins2__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_DIDINS2_lla
{
public:
  explicit Init_DIDINS2_lla(::inertial_sense_ros2::msg::DIDINS2 & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::DIDINS2 lla(::inertial_sense_ros2::msg::DIDINS2::_lla_type arg)
  {
    msg_.lla = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS2 msg_;
};

class Init_DIDINS2_uvw
{
public:
  explicit Init_DIDINS2_uvw(::inertial_sense_ros2::msg::DIDINS2 & msg)
  : msg_(msg)
  {}
  Init_DIDINS2_lla uvw(::inertial_sense_ros2::msg::DIDINS2::_uvw_type arg)
  {
    msg_.uvw = std::move(arg);
    return Init_DIDINS2_lla(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS2 msg_;
};

class Init_DIDINS2_qn2b
{
public:
  explicit Init_DIDINS2_qn2b(::inertial_sense_ros2::msg::DIDINS2 & msg)
  : msg_(msg)
  {}
  Init_DIDINS2_uvw qn2b(::inertial_sense_ros2::msg::DIDINS2::_qn2b_type arg)
  {
    msg_.qn2b = std::move(arg);
    return Init_DIDINS2_uvw(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS2 msg_;
};

class Init_DIDINS2_hdw_status
{
public:
  explicit Init_DIDINS2_hdw_status(::inertial_sense_ros2::msg::DIDINS2 & msg)
  : msg_(msg)
  {}
  Init_DIDINS2_qn2b hdw_status(::inertial_sense_ros2::msg::DIDINS2::_hdw_status_type arg)
  {
    msg_.hdw_status = std::move(arg);
    return Init_DIDINS2_qn2b(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS2 msg_;
};

class Init_DIDINS2_ins_status
{
public:
  explicit Init_DIDINS2_ins_status(::inertial_sense_ros2::msg::DIDINS2 & msg)
  : msg_(msg)
  {}
  Init_DIDINS2_hdw_status ins_status(::inertial_sense_ros2::msg::DIDINS2::_ins_status_type arg)
  {
    msg_.ins_status = std::move(arg);
    return Init_DIDINS2_hdw_status(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS2 msg_;
};

class Init_DIDINS2_time_of_week
{
public:
  explicit Init_DIDINS2_time_of_week(::inertial_sense_ros2::msg::DIDINS2 & msg)
  : msg_(msg)
  {}
  Init_DIDINS2_ins_status time_of_week(::inertial_sense_ros2::msg::DIDINS2::_time_of_week_type arg)
  {
    msg_.time_of_week = std::move(arg);
    return Init_DIDINS2_ins_status(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS2 msg_;
};

class Init_DIDINS2_week
{
public:
  explicit Init_DIDINS2_week(::inertial_sense_ros2::msg::DIDINS2 & msg)
  : msg_(msg)
  {}
  Init_DIDINS2_time_of_week week(::inertial_sense_ros2::msg::DIDINS2::_week_type arg)
  {
    msg_.week = std::move(arg);
    return Init_DIDINS2_time_of_week(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS2 msg_;
};

class Init_DIDINS2_header
{
public:
  Init_DIDINS2_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DIDINS2_week header(::inertial_sense_ros2::msg::DIDINS2::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DIDINS2_week(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS2 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::DIDINS2>()
{
  return inertial_sense_ros2::msg::builder::Init_DIDINS2_header();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS2__BUILDER_HPP_
