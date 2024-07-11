// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/GPSInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gps_info.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS_INFO__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/gps_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_GPSInfo_sattelite_info
{
public:
  explicit Init_GPSInfo_sattelite_info(::inertial_sense_ros2::msg::GPSInfo & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::GPSInfo sattelite_info(::inertial_sense_ros2::msg::GPSInfo::_sattelite_info_type arg)
  {
    msg_.sattelite_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPSInfo msg_;
};

class Init_GPSInfo_num_sats
{
public:
  explicit Init_GPSInfo_num_sats(::inertial_sense_ros2::msg::GPSInfo & msg)
  : msg_(msg)
  {}
  Init_GPSInfo_sattelite_info num_sats(::inertial_sense_ros2::msg::GPSInfo::_num_sats_type arg)
  {
    msg_.num_sats = std::move(arg);
    return Init_GPSInfo_sattelite_info(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPSInfo msg_;
};

class Init_GPSInfo_header
{
public:
  Init_GPSInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GPSInfo_num_sats header(::inertial_sense_ros2::msg::GPSInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GPSInfo_num_sats(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPSInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::GPSInfo>()
{
  return inertial_sense_ros2::msg::builder::Init_GPSInfo_header();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS_INFO__BUILDER_HPP_
