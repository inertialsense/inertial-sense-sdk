// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/INL2States.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/inl2_states.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__INL2_STATES__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__INL2_STATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/inl2_states__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_INL2States_mag_inc
{
public:
  explicit Init_INL2States_mag_inc(::inertial_sense_ros2::msg::INL2States & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::INL2States mag_inc(::inertial_sense_ros2::msg::INL2States::_mag_inc_type arg)
  {
    msg_.mag_inc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::INL2States msg_;
};

class Init_INL2States_mag_dec
{
public:
  explicit Init_INL2States_mag_dec(::inertial_sense_ros2::msg::INL2States & msg)
  : msg_(msg)
  {}
  Init_INL2States_mag_inc mag_dec(::inertial_sense_ros2::msg::INL2States::_mag_dec_type arg)
  {
    msg_.mag_dec = std::move(arg);
    return Init_INL2States_mag_inc(msg_);
  }

private:
  ::inertial_sense_ros2::msg::INL2States msg_;
};

class Init_INL2States_baro_bias
{
public:
  explicit Init_INL2States_baro_bias(::inertial_sense_ros2::msg::INL2States & msg)
  : msg_(msg)
  {}
  Init_INL2States_mag_dec baro_bias(::inertial_sense_ros2::msg::INL2States::_baro_bias_type arg)
  {
    msg_.baro_bias = std::move(arg);
    return Init_INL2States_mag_dec(msg_);
  }

private:
  ::inertial_sense_ros2::msg::INL2States msg_;
};

class Init_INL2States_accel_bias
{
public:
  explicit Init_INL2States_accel_bias(::inertial_sense_ros2::msg::INL2States & msg)
  : msg_(msg)
  {}
  Init_INL2States_baro_bias accel_bias(::inertial_sense_ros2::msg::INL2States::_accel_bias_type arg)
  {
    msg_.accel_bias = std::move(arg);
    return Init_INL2States_baro_bias(msg_);
  }

private:
  ::inertial_sense_ros2::msg::INL2States msg_;
};

class Init_INL2States_gyro_bias
{
public:
  explicit Init_INL2States_gyro_bias(::inertial_sense_ros2::msg::INL2States & msg)
  : msg_(msg)
  {}
  Init_INL2States_accel_bias gyro_bias(::inertial_sense_ros2::msg::INL2States::_gyro_bias_type arg)
  {
    msg_.gyro_bias = std::move(arg);
    return Init_INL2States_accel_bias(msg_);
  }

private:
  ::inertial_sense_ros2::msg::INL2States msg_;
};

class Init_INL2States_pos_ecef
{
public:
  explicit Init_INL2States_pos_ecef(::inertial_sense_ros2::msg::INL2States & msg)
  : msg_(msg)
  {}
  Init_INL2States_gyro_bias pos_ecef(::inertial_sense_ros2::msg::INL2States::_pos_ecef_type arg)
  {
    msg_.pos_ecef = std::move(arg);
    return Init_INL2States_gyro_bias(msg_);
  }

private:
  ::inertial_sense_ros2::msg::INL2States msg_;
};

class Init_INL2States_vel_ecef
{
public:
  explicit Init_INL2States_vel_ecef(::inertial_sense_ros2::msg::INL2States & msg)
  : msg_(msg)
  {}
  Init_INL2States_pos_ecef vel_ecef(::inertial_sense_ros2::msg::INL2States::_vel_ecef_type arg)
  {
    msg_.vel_ecef = std::move(arg);
    return Init_INL2States_pos_ecef(msg_);
  }

private:
  ::inertial_sense_ros2::msg::INL2States msg_;
};

class Init_INL2States_quat_ecef
{
public:
  explicit Init_INL2States_quat_ecef(::inertial_sense_ros2::msg::INL2States & msg)
  : msg_(msg)
  {}
  Init_INL2States_vel_ecef quat_ecef(::inertial_sense_ros2::msg::INL2States::_quat_ecef_type arg)
  {
    msg_.quat_ecef = std::move(arg);
    return Init_INL2States_vel_ecef(msg_);
  }

private:
  ::inertial_sense_ros2::msg::INL2States msg_;
};

class Init_INL2States_header
{
public:
  Init_INL2States_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_INL2States_quat_ecef header(::inertial_sense_ros2::msg::INL2States::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_INL2States_quat_ecef(msg_);
  }

private:
  ::inertial_sense_ros2::msg::INL2States msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::INL2States>()
{
  return inertial_sense_ros2::msg::builder::Init_INL2States_header();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__INL2_STATES__BUILDER_HPP_
