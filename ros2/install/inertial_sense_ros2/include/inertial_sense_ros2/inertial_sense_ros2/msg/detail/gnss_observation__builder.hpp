// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/GNSSObservation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_observation.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/gnss_observation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_GNSSObservation_d
{
public:
  explicit Init_GNSSObservation_d(::inertial_sense_ros2::msg::GNSSObservation & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::GNSSObservation d(::inertial_sense_ros2::msg::GNSSObservation::_d_type arg)
  {
    msg_.d = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObservation msg_;
};

class Init_GNSSObservation_p
{
public:
  explicit Init_GNSSObservation_p(::inertial_sense_ros2::msg::GNSSObservation & msg)
  : msg_(msg)
  {}
  Init_GNSSObservation_d p(::inertial_sense_ros2::msg::GNSSObservation::_p_type arg)
  {
    msg_.p = std::move(arg);
    return Init_GNSSObservation_d(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObservation msg_;
};

class Init_GNSSObservation_l
{
public:
  explicit Init_GNSSObservation_l(::inertial_sense_ros2::msg::GNSSObservation & msg)
  : msg_(msg)
  {}
  Init_GNSSObservation_p l(::inertial_sense_ros2::msg::GNSSObservation::_l_type arg)
  {
    msg_.l = std::move(arg);
    return Init_GNSSObservation_p(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObservation msg_;
};

class Init_GNSSObservation_qual_p
{
public:
  explicit Init_GNSSObservation_qual_p(::inertial_sense_ros2::msg::GNSSObservation & msg)
  : msg_(msg)
  {}
  Init_GNSSObservation_l qual_p(::inertial_sense_ros2::msg::GNSSObservation::_qual_p_type arg)
  {
    msg_.qual_p = std::move(arg);
    return Init_GNSSObservation_l(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObservation msg_;
};

class Init_GNSSObservation_qual_l
{
public:
  explicit Init_GNSSObservation_qual_l(::inertial_sense_ros2::msg::GNSSObservation & msg)
  : msg_(msg)
  {}
  Init_GNSSObservation_qual_p qual_l(::inertial_sense_ros2::msg::GNSSObservation::_qual_l_type arg)
  {
    msg_.qual_l = std::move(arg);
    return Init_GNSSObservation_qual_p(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObservation msg_;
};

class Init_GNSSObservation_code
{
public:
  explicit Init_GNSSObservation_code(::inertial_sense_ros2::msg::GNSSObservation & msg)
  : msg_(msg)
  {}
  Init_GNSSObservation_qual_l code(::inertial_sense_ros2::msg::GNSSObservation::_code_type arg)
  {
    msg_.code = std::move(arg);
    return Init_GNSSObservation_qual_l(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObservation msg_;
};

class Init_GNSSObservation_lli
{
public:
  explicit Init_GNSSObservation_lli(::inertial_sense_ros2::msg::GNSSObservation & msg)
  : msg_(msg)
  {}
  Init_GNSSObservation_code lli(::inertial_sense_ros2::msg::GNSSObservation::_lli_type arg)
  {
    msg_.lli = std::move(arg);
    return Init_GNSSObservation_code(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObservation msg_;
};

class Init_GNSSObservation_snrr
{
public:
  explicit Init_GNSSObservation_snrr(::inertial_sense_ros2::msg::GNSSObservation & msg)
  : msg_(msg)
  {}
  Init_GNSSObservation_lli snrr(::inertial_sense_ros2::msg::GNSSObservation::_snrr_type arg)
  {
    msg_.snrr = std::move(arg);
    return Init_GNSSObservation_lli(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObservation msg_;
};

class Init_GNSSObservation_rcv
{
public:
  explicit Init_GNSSObservation_rcv(::inertial_sense_ros2::msg::GNSSObservation & msg)
  : msg_(msg)
  {}
  Init_GNSSObservation_snrr rcv(::inertial_sense_ros2::msg::GNSSObservation::_rcv_type arg)
  {
    msg_.rcv = std::move(arg);
    return Init_GNSSObservation_snrr(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObservation msg_;
};

class Init_GNSSObservation_sat
{
public:
  explicit Init_GNSSObservation_sat(::inertial_sense_ros2::msg::GNSSObservation & msg)
  : msg_(msg)
  {}
  Init_GNSSObservation_rcv sat(::inertial_sense_ros2::msg::GNSSObservation::_sat_type arg)
  {
    msg_.sat = std::move(arg);
    return Init_GNSSObservation_rcv(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObservation msg_;
};

class Init_GNSSObservation_time
{
public:
  explicit Init_GNSSObservation_time(::inertial_sense_ros2::msg::GNSSObservation & msg)
  : msg_(msg)
  {}
  Init_GNSSObservation_sat time(::inertial_sense_ros2::msg::GNSSObservation::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_GNSSObservation_sat(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObservation msg_;
};

class Init_GNSSObservation_header
{
public:
  Init_GNSSObservation_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GNSSObservation_time header(::inertial_sense_ros2::msg::GNSSObservation::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GNSSObservation_time(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSObservation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::GNSSObservation>()
{
  return inertial_sense_ros2::msg::builder::Init_GNSSObservation_header();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__BUILDER_HPP_
