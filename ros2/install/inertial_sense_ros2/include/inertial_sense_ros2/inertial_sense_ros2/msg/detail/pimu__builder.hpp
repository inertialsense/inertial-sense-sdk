// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/PIMU.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/pimu.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__PIMU__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__PIMU__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/pimu__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_PIMU_dt
{
public:
  explicit Init_PIMU_dt(::inertial_sense_ros2::msg::PIMU & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::PIMU dt(::inertial_sense_ros2::msg::PIMU::_dt_type arg)
  {
    msg_.dt = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::PIMU msg_;
};

class Init_PIMU_dvel
{
public:
  explicit Init_PIMU_dvel(::inertial_sense_ros2::msg::PIMU & msg)
  : msg_(msg)
  {}
  Init_PIMU_dt dvel(::inertial_sense_ros2::msg::PIMU::_dvel_type arg)
  {
    msg_.dvel = std::move(arg);
    return Init_PIMU_dt(msg_);
  }

private:
  ::inertial_sense_ros2::msg::PIMU msg_;
};

class Init_PIMU_dtheta
{
public:
  explicit Init_PIMU_dtheta(::inertial_sense_ros2::msg::PIMU & msg)
  : msg_(msg)
  {}
  Init_PIMU_dvel dtheta(::inertial_sense_ros2::msg::PIMU::_dtheta_type arg)
  {
    msg_.dtheta = std::move(arg);
    return Init_PIMU_dvel(msg_);
  }

private:
  ::inertial_sense_ros2::msg::PIMU msg_;
};

class Init_PIMU_header
{
public:
  Init_PIMU_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PIMU_dtheta header(::inertial_sense_ros2::msg::PIMU::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PIMU_dtheta(msg_);
  }

private:
  ::inertial_sense_ros2::msg::PIMU msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::PIMU>()
{
  return inertial_sense_ros2::msg::builder::Init_PIMU_header();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__PIMU__BUILDER_HPP_
