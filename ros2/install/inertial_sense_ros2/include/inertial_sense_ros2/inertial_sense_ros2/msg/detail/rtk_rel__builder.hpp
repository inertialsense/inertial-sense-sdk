// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/RTKRel.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/rtk_rel.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_REL__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_REL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/rtk_rel__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_RTKRel_heading_base_to_rover
{
public:
  explicit Init_RTKRel_heading_base_to_rover(::inertial_sense_ros2::msg::RTKRel & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::RTKRel heading_base_to_rover(::inertial_sense_ros2::msg::RTKRel::_heading_base_to_rover_type arg)
  {
    msg_.heading_base_to_rover = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKRel msg_;
};

class Init_RTKRel_distance_base_to_rover
{
public:
  explicit Init_RTKRel_distance_base_to_rover(::inertial_sense_ros2::msg::RTKRel & msg)
  : msg_(msg)
  {}
  Init_RTKRel_heading_base_to_rover distance_base_to_rover(::inertial_sense_ros2::msg::RTKRel::_distance_base_to_rover_type arg)
  {
    msg_.distance_base_to_rover = std::move(arg);
    return Init_RTKRel_heading_base_to_rover(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKRel msg_;
};

class Init_RTKRel_vector_base_to_rover
{
public:
  explicit Init_RTKRel_vector_base_to_rover(::inertial_sense_ros2::msg::RTKRel & msg)
  : msg_(msg)
  {}
  Init_RTKRel_distance_base_to_rover vector_base_to_rover(::inertial_sense_ros2::msg::RTKRel::_vector_base_to_rover_type arg)
  {
    msg_.vector_base_to_rover = std::move(arg);
    return Init_RTKRel_distance_base_to_rover(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKRel msg_;
};

class Init_RTKRel_e_gps_status_fix
{
public:
  explicit Init_RTKRel_e_gps_status_fix(::inertial_sense_ros2::msg::RTKRel & msg)
  : msg_(msg)
  {}
  Init_RTKRel_vector_base_to_rover e_gps_status_fix(::inertial_sense_ros2::msg::RTKRel::_e_gps_status_fix_type arg)
  {
    msg_.e_gps_status_fix = std::move(arg);
    return Init_RTKRel_vector_base_to_rover(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKRel msg_;
};

class Init_RTKRel_ar_ratio
{
public:
  explicit Init_RTKRel_ar_ratio(::inertial_sense_ros2::msg::RTKRel & msg)
  : msg_(msg)
  {}
  Init_RTKRel_e_gps_status_fix ar_ratio(::inertial_sense_ros2::msg::RTKRel::_ar_ratio_type arg)
  {
    msg_.ar_ratio = std::move(arg);
    return Init_RTKRel_e_gps_status_fix(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKRel msg_;
};

class Init_RTKRel_differential_age
{
public:
  explicit Init_RTKRel_differential_age(::inertial_sense_ros2::msg::RTKRel & msg)
  : msg_(msg)
  {}
  Init_RTKRel_ar_ratio differential_age(::inertial_sense_ros2::msg::RTKRel::_differential_age_type arg)
  {
    msg_.differential_age = std::move(arg);
    return Init_RTKRel_ar_ratio(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKRel msg_;
};

class Init_RTKRel_header
{
public:
  Init_RTKRel_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RTKRel_differential_age header(::inertial_sense_ros2::msg::RTKRel::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RTKRel_differential_age(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKRel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::RTKRel>()
{
  return inertial_sense_ros2::msg::builder::Init_RTKRel_header();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_REL__BUILDER_HPP_
