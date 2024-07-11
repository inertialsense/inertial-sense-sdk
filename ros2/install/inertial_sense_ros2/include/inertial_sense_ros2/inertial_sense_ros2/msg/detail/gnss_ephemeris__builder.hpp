// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inertial_sense_ros2:msg/GNSSEphemeris.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_ephemeris.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_EPHEMERIS__BUILDER_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_EPHEMERIS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inertial_sense_ros2/msg/detail/gnss_ephemeris__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inertial_sense_ros2
{

namespace msg
{

namespace builder
{

class Init_GNSSEphemeris_ndot
{
public:
  explicit Init_GNSSEphemeris_ndot(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  ::inertial_sense_ros2::msg::GNSSEphemeris ndot(::inertial_sense_ros2::msg::GNSSEphemeris::_ndot_type arg)
  {
    msg_.ndot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_a_dot
{
public:
  explicit Init_GNSSEphemeris_a_dot(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_ndot a_dot(::inertial_sense_ros2::msg::GNSSEphemeris::_a_dot_type arg)
  {
    msg_.a_dot = std::move(arg);
    return Init_GNSSEphemeris_ndot(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_tgd
{
public:
  explicit Init_GNSSEphemeris_tgd(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_a_dot tgd(::inertial_sense_ros2::msg::GNSSEphemeris::_tgd_type arg)
  {
    msg_.tgd = std::move(arg);
    return Init_GNSSEphemeris_a_dot(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_f2
{
public:
  explicit Init_GNSSEphemeris_f2(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_tgd f2(::inertial_sense_ros2::msg::GNSSEphemeris::_f2_type arg)
  {
    msg_.f2 = std::move(arg);
    return Init_GNSSEphemeris_tgd(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_f1
{
public:
  explicit Init_GNSSEphemeris_f1(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_f2 f1(::inertial_sense_ros2::msg::GNSSEphemeris::_f1_type arg)
  {
    msg_.f1 = std::move(arg);
    return Init_GNSSEphemeris_f2(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_f0
{
public:
  explicit Init_GNSSEphemeris_f0(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_f1 f0(::inertial_sense_ros2::msg::GNSSEphemeris::_f0_type arg)
  {
    msg_.f0 = std::move(arg);
    return Init_GNSSEphemeris_f1(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_fit
{
public:
  explicit Init_GNSSEphemeris_fit(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_f0 fit(::inertial_sense_ros2::msg::GNSSEphemeris::_fit_type arg)
  {
    msg_.fit = std::move(arg);
    return Init_GNSSEphemeris_f0(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_toes
{
public:
  explicit Init_GNSSEphemeris_toes(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_fit toes(::inertial_sense_ros2::msg::GNSSEphemeris::_toes_type arg)
  {
    msg_.toes = std::move(arg);
    return Init_GNSSEphemeris_fit(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_cis
{
public:
  explicit Init_GNSSEphemeris_cis(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_toes cis(::inertial_sense_ros2::msg::GNSSEphemeris::_cis_type arg)
  {
    msg_.cis = std::move(arg);
    return Init_GNSSEphemeris_toes(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_cic
{
public:
  explicit Init_GNSSEphemeris_cic(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_cis cic(::inertial_sense_ros2::msg::GNSSEphemeris::_cic_type arg)
  {
    msg_.cic = std::move(arg);
    return Init_GNSSEphemeris_cis(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_cus
{
public:
  explicit Init_GNSSEphemeris_cus(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_cic cus(::inertial_sense_ros2::msg::GNSSEphemeris::_cus_type arg)
  {
    msg_.cus = std::move(arg);
    return Init_GNSSEphemeris_cic(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_cuc
{
public:
  explicit Init_GNSSEphemeris_cuc(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_cus cuc(::inertial_sense_ros2::msg::GNSSEphemeris::_cuc_type arg)
  {
    msg_.cuc = std::move(arg);
    return Init_GNSSEphemeris_cus(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_crs
{
public:
  explicit Init_GNSSEphemeris_crs(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_cuc crs(::inertial_sense_ros2::msg::GNSSEphemeris::_crs_type arg)
  {
    msg_.crs = std::move(arg);
    return Init_GNSSEphemeris_cuc(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_crc
{
public:
  explicit Init_GNSSEphemeris_crc(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_crs crc(::inertial_sense_ros2::msg::GNSSEphemeris::_crc_type arg)
  {
    msg_.crc = std::move(arg);
    return Init_GNSSEphemeris_crs(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_idot
{
public:
  explicit Init_GNSSEphemeris_idot(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_crc idot(::inertial_sense_ros2::msg::GNSSEphemeris::_idot_type arg)
  {
    msg_.idot = std::move(arg);
    return Init_GNSSEphemeris_crc(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_omg_d
{
public:
  explicit Init_GNSSEphemeris_omg_d(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_idot omg_d(::inertial_sense_ros2::msg::GNSSEphemeris::_omg_d_type arg)
  {
    msg_.omg_d = std::move(arg);
    return Init_GNSSEphemeris_idot(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_deln
{
public:
  explicit Init_GNSSEphemeris_deln(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_omg_d deln(::inertial_sense_ros2::msg::GNSSEphemeris::_deln_type arg)
  {
    msg_.deln = std::move(arg);
    return Init_GNSSEphemeris_omg_d(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_m_0
{
public:
  explicit Init_GNSSEphemeris_m_0(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_deln m_0(::inertial_sense_ros2::msg::GNSSEphemeris::_m_0_type arg)
  {
    msg_.m_0 = std::move(arg);
    return Init_GNSSEphemeris_deln(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_omg
{
public:
  explicit Init_GNSSEphemeris_omg(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_m_0 omg(::inertial_sense_ros2::msg::GNSSEphemeris::_omg_type arg)
  {
    msg_.omg = std::move(arg);
    return Init_GNSSEphemeris_m_0(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_omg_0
{
public:
  explicit Init_GNSSEphemeris_omg_0(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_omg omg_0(::inertial_sense_ros2::msg::GNSSEphemeris::_omg_0_type arg)
  {
    msg_.omg_0 = std::move(arg);
    return Init_GNSSEphemeris_omg(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_i0
{
public:
  explicit Init_GNSSEphemeris_i0(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_omg_0 i0(::inertial_sense_ros2::msg::GNSSEphemeris::_i0_type arg)
  {
    msg_.i0 = std::move(arg);
    return Init_GNSSEphemeris_omg_0(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_e
{
public:
  explicit Init_GNSSEphemeris_e(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_i0 e(::inertial_sense_ros2::msg::GNSSEphemeris::_e_type arg)
  {
    msg_.e = std::move(arg);
    return Init_GNSSEphemeris_i0(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_a
{
public:
  explicit Init_GNSSEphemeris_a(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_e a(::inertial_sense_ros2::msg::GNSSEphemeris::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_GNSSEphemeris_e(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_ttr
{
public:
  explicit Init_GNSSEphemeris_ttr(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_a ttr(::inertial_sense_ros2::msg::GNSSEphemeris::_ttr_type arg)
  {
    msg_.ttr = std::move(arg);
    return Init_GNSSEphemeris_a(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_toc
{
public:
  explicit Init_GNSSEphemeris_toc(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_ttr toc(::inertial_sense_ros2::msg::GNSSEphemeris::_toc_type arg)
  {
    msg_.toc = std::move(arg);
    return Init_GNSSEphemeris_ttr(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_toe
{
public:
  explicit Init_GNSSEphemeris_toe(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_toc toe(::inertial_sense_ros2::msg::GNSSEphemeris::_toe_type arg)
  {
    msg_.toe = std::move(arg);
    return Init_GNSSEphemeris_toc(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_flag
{
public:
  explicit Init_GNSSEphemeris_flag(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_toe flag(::inertial_sense_ros2::msg::GNSSEphemeris::_flag_type arg)
  {
    msg_.flag = std::move(arg);
    return Init_GNSSEphemeris_toe(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_code
{
public:
  explicit Init_GNSSEphemeris_code(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_flag code(::inertial_sense_ros2::msg::GNSSEphemeris::_code_type arg)
  {
    msg_.code = std::move(arg);
    return Init_GNSSEphemeris_flag(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_week
{
public:
  explicit Init_GNSSEphemeris_week(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_code week(::inertial_sense_ros2::msg::GNSSEphemeris::_week_type arg)
  {
    msg_.week = std::move(arg);
    return Init_GNSSEphemeris_code(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_svh
{
public:
  explicit Init_GNSSEphemeris_svh(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_week svh(::inertial_sense_ros2::msg::GNSSEphemeris::_svh_type arg)
  {
    msg_.svh = std::move(arg);
    return Init_GNSSEphemeris_week(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_sva
{
public:
  explicit Init_GNSSEphemeris_sva(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_svh sva(::inertial_sense_ros2::msg::GNSSEphemeris::_sva_type arg)
  {
    msg_.sva = std::move(arg);
    return Init_GNSSEphemeris_svh(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_iodc
{
public:
  explicit Init_GNSSEphemeris_iodc(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_sva iodc(::inertial_sense_ros2::msg::GNSSEphemeris::_iodc_type arg)
  {
    msg_.iodc = std::move(arg);
    return Init_GNSSEphemeris_sva(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_iode
{
public:
  explicit Init_GNSSEphemeris_iode(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_iodc iode(::inertial_sense_ros2::msg::GNSSEphemeris::_iode_type arg)
  {
    msg_.iode = std::move(arg);
    return Init_GNSSEphemeris_iodc(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_sat
{
public:
  explicit Init_GNSSEphemeris_sat(::inertial_sense_ros2::msg::GNSSEphemeris & msg)
  : msg_(msg)
  {}
  Init_GNSSEphemeris_iode sat(::inertial_sense_ros2::msg::GNSSEphemeris::_sat_type arg)
  {
    msg_.sat = std::move(arg);
    return Init_GNSSEphemeris_iode(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

class Init_GNSSEphemeris_header
{
public:
  Init_GNSSEphemeris_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GNSSEphemeris_sat header(::inertial_sense_ros2::msg::GNSSEphemeris::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GNSSEphemeris_sat(msg_);
  }

private:
  ::inertial_sense_ros2::msg::GNSSEphemeris msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inertial_sense_ros2::msg::GNSSEphemeris>()
{
  return inertial_sense_ros2::msg::builder::Init_GNSSEphemeris_header();
}

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_EPHEMERIS__BUILDER_HPP_
