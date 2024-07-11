// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/GPS.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gps.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/gps__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_GPS_pdop
{
public:
  explicit Init_GPS_pdop(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::GPS pdop(::inertial_sense_ros2::msg::GPS::_pdop_type arg)
  {
    msg_.pdop = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_sacc
{
public:
  explicit Init_GPS_sacc(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_pdop sacc(::inertial_sense_ros2::msg::GPS::_sacc_type arg)
  {
    msg_.sacc = std::move(arg);
    return Init_GPS_pdop(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_vacc
{
public:
  explicit Init_GPS_vacc(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_sacc vacc(::inertial_sense_ros2::msg::GPS::_vacc_type arg)
  {
    msg_.vacc = std::move(arg);
    return Init_GPS_sacc(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_hacc
{
public:
  explicit Init_GPS_hacc(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_vacc hacc(::inertial_sense_ros2::msg::GPS::_hacc_type arg)
  {
    msg_.hacc = std::move(arg);
    return Init_GPS_vacc(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_hmsl
{
public:
  explicit Init_GPS_hmsl(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_hacc hmsl(::inertial_sense_ros2::msg::GPS::_hmsl_type arg)
  {
    msg_.hmsl = std::move(arg);
    return Init_GPS_hacc(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_vel_ecef
{
public:
  explicit Init_GPS_vel_ecef(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_hmsl vel_ecef(::inertial_sense_ros2::msg::GPS::_vel_ecef_type arg)
  {
    msg_.vel_ecef = std::move(arg);
    return Init_GPS_hmsl(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_pos_ecef
{
public:
  explicit Init_GPS_pos_ecef(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_vel_ecef pos_ecef(::inertial_sense_ros2::msg::GPS::_pos_ecef_type arg)
  {
    msg_.pos_ecef = std::move(arg);
    return Init_GPS_vel_ecef(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_altitude
{
public:
  explicit Init_GPS_altitude(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_pos_ecef altitude(::inertial_sense_ros2::msg::GPS::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_GPS_pos_ecef(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_longitude
{
public:
  explicit Init_GPS_longitude(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_altitude longitude(::inertial_sense_ros2::msg::GPS::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GPS_altitude(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_latitude
{
public:
  explicit Init_GPS_latitude(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_longitude latitude(::inertial_sense_ros2::msg::GPS::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GPS_longitude(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_cno
{
public:
  explicit Init_GPS_cno(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_latitude cno(::inertial_sense_ros2::msg::GPS::_cno_type arg)
  {
    msg_.cno = std::move(arg);
    return Init_GPS_latitude(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_status
{
public:
  explicit Init_GPS_status(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_cno status(::inertial_sense_ros2::msg::GPS::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_GPS_cno(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_num_sat
{
public:
  explicit Init_GPS_num_sat(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_status num_sat(::inertial_sense_ros2::msg::GPS::_num_sat_type arg)
  {
    msg_.num_sat = std::move(arg);
    return Init_GPS_status(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_week
{
public:
  explicit Init_GPS_week(::inertial_sense_ros2::msg::GPS & msg)
  : msg_(msg)
  {}
  Init_GPS_num_sat week(::inertial_sense_ros2::msg::GPS::_week_type arg)
  {
    msg_.week = std::move(arg);
    return Init_GPS_num_sat(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

class Init_GPS_header
{
public:
  Init_GPS_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GPS_week header(::inertial_sense_ros2::msg::GPS::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GPS_week(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GPS msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::GPS>()
{
  return inertial_sense_ros2::msg::builder::Init_GPS_header();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS__BUILDER_HPP_
