// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/SatInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/sat_info.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__SAT_INFO__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__SAT_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/sat_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_SatInfo_cno
{
public:
  explicit Init_SatInfo_cno(::inertial_sense_ros2::msg::SatInfo & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::SatInfo cno(::inertial_sense_ros2::msg::SatInfo::_cno_type arg)
  {
    msg_.cno = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::SatInfo msg_;
};

class Init_SatInfo_sat_id
{
public:
  Init_SatInfo_sat_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SatInfo_cno sat_id(::inertial_sense_ros2::msg::SatInfo::_sat_id_type arg)
  {
    msg_.sat_id = std::move(arg);
    return Init_SatInfo_cno(msg_);
  }

private:
  ::inertial_sense_ros2::msg::SatInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::SatInfo>()
{
  return inertial_sense_ros2::msg::builder::Init_SatInfo_sat_id();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__SAT_INFO__BUILDER_HPP_
