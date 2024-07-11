// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/RTKInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/rtk_info.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_INFO__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/rtk_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_RTKInfo_base_ant_count
{
public:
  explicit Init_RTKInfo_base_ant_count(::inertial_sense_ros2::msg::RTKInfo & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::RTKInfo base_ant_count(::inertial_sense_ros2::msg::RTKInfo::_base_ant_count_type arg)
  {
    msg_.base_ant_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKInfo msg_;
};

class Init_RTKInfo_base_eph
{
public:
  explicit Init_RTKInfo_base_eph(::inertial_sense_ros2::msg::RTKInfo & msg)
  : msg_(msg)
  {}
  Init_RTKInfo_base_ant_count base_eph(::inertial_sense_ros2::msg::RTKInfo::_base_eph_type arg)
  {
    msg_.base_eph = std::move(arg);
    return Init_RTKInfo_base_ant_count(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKInfo msg_;
};

class Init_RTKInfo_rover_eph
{
public:
  explicit Init_RTKInfo_rover_eph(::inertial_sense_ros2::msg::RTKInfo & msg)
  : msg_(msg)
  {}
  Init_RTKInfo_base_eph rover_eph(::inertial_sense_ros2::msg::RTKInfo::_rover_eph_type arg)
  {
    msg_.rover_eph = std::move(arg);
    return Init_RTKInfo_base_eph(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKInfo msg_;
};

class Init_RTKInfo_base_obs
{
public:
  explicit Init_RTKInfo_base_obs(::inertial_sense_ros2::msg::RTKInfo & msg)
  : msg_(msg)
  {}
  Init_RTKInfo_rover_eph base_obs(::inertial_sense_ros2::msg::RTKInfo::_base_obs_type arg)
  {
    msg_.base_obs = std::move(arg);
    return Init_RTKInfo_rover_eph(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKInfo msg_;
};

class Init_RTKInfo_rover_obs
{
public:
  explicit Init_RTKInfo_rover_obs(::inertial_sense_ros2::msg::RTKInfo & msg)
  : msg_(msg)
  {}
  Init_RTKInfo_base_obs rover_obs(::inertial_sense_ros2::msg::RTKInfo::_rover_obs_type arg)
  {
    msg_.rover_obs = std::move(arg);
    return Init_RTKInfo_base_obs(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKInfo msg_;
};

class Init_RTKInfo_cycle_slip_count
{
public:
  explicit Init_RTKInfo_cycle_slip_count(::inertial_sense_ros2::msg::RTKInfo & msg)
  : msg_(msg)
  {}
  Init_RTKInfo_rover_obs cycle_slip_count(::inertial_sense_ros2::msg::RTKInfo::_cycle_slip_count_type arg)
  {
    msg_.cycle_slip_count = std::move(arg);
    return Init_RTKInfo_rover_obs(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKInfo msg_;
};

class Init_RTKInfo_base_lla
{
public:
  explicit Init_RTKInfo_base_lla(::inertial_sense_ros2::msg::RTKInfo & msg)
  : msg_(msg)
  {}
  Init_RTKInfo_cycle_slip_count base_lla(::inertial_sense_ros2::msg::RTKInfo::_base_lla_type arg)
  {
    msg_.base_lla = std::move(arg);
    return Init_RTKInfo_cycle_slip_count(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKInfo msg_;
};

class Init_RTKInfo_header
{
public:
  Init_RTKInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RTKInfo_base_lla header(::inertial_sense_ros2::msg::RTKInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RTKInfo_base_lla(msg_);
  }

private:
  ::inertial_sense_ros2::msg::RTKInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::RTKInfo>()
{
  return inertial_sense_ros2::msg::builder::Init_RTKInfo_header();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_INFO__BUILDER_HPP_
