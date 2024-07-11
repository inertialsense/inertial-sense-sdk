// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/DIDINS4.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/didins4.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS4__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS4__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/didins4__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_DIDINS4_ecef
{
public:
  explicit Init_DIDINS4_ecef(::inertial_sense_ros2::msg::DIDINS4 & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::DIDINS4 ecef(::inertial_sense_ros2::msg::DIDINS4::_ecef_type arg)
  {
    msg_.ecef = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS4 msg_;
};

class Init_DIDINS4_ve
{
public:
  explicit Init_DIDINS4_ve(::inertial_sense_ros2::msg::DIDINS4 & msg)
  : msg_(msg)
  {}
  Init_DIDINS4_ecef ve(::inertial_sense_ros2::msg::DIDINS4::_ve_type arg)
  {
    msg_.ve = std::move(arg);
    return Init_DIDINS4_ecef(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS4 msg_;
};

class Init_DIDINS4_qe2b
{
public:
  explicit Init_DIDINS4_qe2b(::inertial_sense_ros2::msg::DIDINS4 & msg)
  : msg_(msg)
  {}
  Init_DIDINS4_ve qe2b(::inertial_sense_ros2::msg::DIDINS4::_qe2b_type arg)
  {
    msg_.qe2b = std::move(arg);
    return Init_DIDINS4_ve(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS4 msg_;
};

class Init_DIDINS4_hdw_status
{
public:
  explicit Init_DIDINS4_hdw_status(::inertial_sense_ros2::msg::DIDINS4 & msg)
  : msg_(msg)
  {}
  Init_DIDINS4_qe2b hdw_status(::inertial_sense_ros2::msg::DIDINS4::_hdw_status_type arg)
  {
    msg_.hdw_status = std::move(arg);
    return Init_DIDINS4_qe2b(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS4 msg_;
};

class Init_DIDINS4_ins_status
{
public:
  explicit Init_DIDINS4_ins_status(::inertial_sense_ros2::msg::DIDINS4 & msg)
  : msg_(msg)
  {}
  Init_DIDINS4_hdw_status ins_status(::inertial_sense_ros2::msg::DIDINS4::_ins_status_type arg)
  {
    msg_.ins_status = std::move(arg);
    return Init_DIDINS4_hdw_status(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS4 msg_;
};

class Init_DIDINS4_time_of_week
{
public:
  explicit Init_DIDINS4_time_of_week(::inertial_sense_ros2::msg::DIDINS4 & msg)
  : msg_(msg)
  {}
  Init_DIDINS4_ins_status time_of_week(::inertial_sense_ros2::msg::DIDINS4::_time_of_week_type arg)
  {
    msg_.time_of_week = std::move(arg);
    return Init_DIDINS4_ins_status(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS4 msg_;
};

class Init_DIDINS4_week
{
public:
  explicit Init_DIDINS4_week(::inertial_sense_ros2::msg::DIDINS4 & msg)
  : msg_(msg)
  {}
  Init_DIDINS4_time_of_week week(::inertial_sense_ros2::msg::DIDINS4::_week_type arg)
  {
    msg_.week = std::move(arg);
    return Init_DIDINS4_time_of_week(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS4 msg_;
};

class Init_DIDINS4_header
{
public:
  Init_DIDINS4_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DIDINS4_week header(::inertial_sense_ros2::msg::DIDINS4::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DIDINS4_week(msg_);
  }

private:
  ::inertial_sense_ros2::msg::DIDINS4 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::DIDINS4>()
{
  return inertial_sense_ros2::msg::builder::Init_DIDINS4_header();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS4__BUILDER_HPP_
