// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/GlonassEphemeris.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/glonass_ephemeris.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GLONASS_EPHEMERIS__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GLONASS_EPHEMERIS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/glonass_ephemeris__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_GlonassEphemeris_dtaun
{
public:
  explicit Init_GlonassEphemeris_dtaun(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::GlonassEphemeris dtaun(::inertial_sense_ros2::msg::GlonassEphemeris::_dtaun_type arg)
  {
    msg_.dtaun = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_gamn
{
public:
  explicit Init_GlonassEphemeris_gamn(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  Init_GlonassEphemeris_dtaun gamn(::inertial_sense_ros2::msg::GlonassEphemeris::_gamn_type arg)
  {
    msg_.gamn = std::move(arg);
    return Init_GlonassEphemeris_dtaun(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_taun
{
public:
  explicit Init_GlonassEphemeris_taun(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  Init_GlonassEphemeris_gamn taun(::inertial_sense_ros2::msg::GlonassEphemeris::_taun_type arg)
  {
    msg_.taun = std::move(arg);
    return Init_GlonassEphemeris_gamn(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_acc
{
public:
  explicit Init_GlonassEphemeris_acc(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  Init_GlonassEphemeris_taun acc(::inertial_sense_ros2::msg::GlonassEphemeris::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return Init_GlonassEphemeris_taun(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_vel
{
public:
  explicit Init_GlonassEphemeris_vel(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  Init_GlonassEphemeris_acc vel(::inertial_sense_ros2::msg::GlonassEphemeris::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_GlonassEphemeris_acc(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_pos
{
public:
  explicit Init_GlonassEphemeris_pos(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  Init_GlonassEphemeris_vel pos(::inertial_sense_ros2::msg::GlonassEphemeris::_pos_type arg)
  {
    msg_.pos = std::move(arg);
    return Init_GlonassEphemeris_vel(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_tof
{
public:
  explicit Init_GlonassEphemeris_tof(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  Init_GlonassEphemeris_pos tof(::inertial_sense_ros2::msg::GlonassEphemeris::_tof_type arg)
  {
    msg_.tof = std::move(arg);
    return Init_GlonassEphemeris_pos(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_toe
{
public:
  explicit Init_GlonassEphemeris_toe(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  Init_GlonassEphemeris_tof toe(::inertial_sense_ros2::msg::GlonassEphemeris::_toe_type arg)
  {
    msg_.toe = std::move(arg);
    return Init_GlonassEphemeris_tof(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_age
{
public:
  explicit Init_GlonassEphemeris_age(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  Init_GlonassEphemeris_toe age(::inertial_sense_ros2::msg::GlonassEphemeris::_age_type arg)
  {
    msg_.age = std::move(arg);
    return Init_GlonassEphemeris_toe(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_sva
{
public:
  explicit Init_GlonassEphemeris_sva(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  Init_GlonassEphemeris_age sva(::inertial_sense_ros2::msg::GlonassEphemeris::_sva_type arg)
  {
    msg_.sva = std::move(arg);
    return Init_GlonassEphemeris_age(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_svh
{
public:
  explicit Init_GlonassEphemeris_svh(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  Init_GlonassEphemeris_sva svh(::inertial_sense_ros2::msg::GlonassEphemeris::_svh_type arg)
  {
    msg_.svh = std::move(arg);
    return Init_GlonassEphemeris_sva(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_frq
{
public:
  explicit Init_GlonassEphemeris_frq(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  Init_GlonassEphemeris_svh frq(::inertial_sense_ros2::msg::GlonassEphemeris::_frq_type arg)
  {
    msg_.frq = std::move(arg);
    return Init_GlonassEphemeris_svh(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_iode
{
public:
  explicit Init_GlonassEphemeris_iode(::inertial_sense_ros2::msg::GlonassEphemeris & msg)
  : msg_(msg)
  {}
  Init_GlonassEphemeris_frq iode(::inertial_sense_ros2::msg::GlonassEphemeris::_iode_type arg)
  {
    msg_.iode = std::move(arg);
    return Init_GlonassEphemeris_frq(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

class Init_GlonassEphemeris_sat
{
public:
  Init_GlonassEphemeris_sat()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GlonassEphemeris_iode sat(::inertial_sense_ros2::msg::GlonassEphemeris::_sat_type arg)
  {
    msg_.sat = std::move(arg);
    return Init_GlonassEphemeris_iode(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GlonassEphemeris msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::GlonassEphemeris>()
{
  return inertial_sense_ros2::msg::builder::Init_GlonassEphemeris_sat();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GLONASS_EPHEMERIS__BUILDER_HPP_
