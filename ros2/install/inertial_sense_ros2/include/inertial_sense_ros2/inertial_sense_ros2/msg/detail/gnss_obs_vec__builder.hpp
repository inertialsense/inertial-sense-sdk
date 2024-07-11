// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/GNSSObsVec.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_obs_vec.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/gnss_obs_vec__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_GNSSObsVec_obs
{
public:
  explicit Init_GNSSObsVec_obs(::inertial_sense_ros2::msg::GNSSObsVec & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::GNSSObsVec obs(::inertial_sense_ros2::msg::GNSSObsVec::_obs_type arg)
  {
    msg_.obs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObsVec msg_;
};

class Init_GNSSObsVec_time
{
public:
  explicit Init_GNSSObsVec_time(::inertial_sense_ros2::msg::GNSSObsVec & msg)
  : msg_(msg)
  {}
  Init_GNSSObsVec_obs time(::inertial_sense_ros2::msg::GNSSObsVec::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_GNSSObsVec_obs(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObsVec msg_;
};

class Init_GNSSObsVec_header
{
public:
  Init_GNSSObsVec_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GNSSObsVec_time header(::inertial_sense_ros2::msg::GNSSObsVec::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GNSSObsVec_time(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObsVec msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::GNSSObsVec>()
{
  return inertial_sense_ros2::msg::builder::Init_GNSSObsVec_header();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__BUILDER_HPP_
