// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/GNSSEphemeris.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_ephemeris.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_EPHEMERIS__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_EPHEMERIS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'toe'
// Member 'toc'
// Member 'ttr'
#include "inertial_sense_ros2/msg/detail/g_time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__msg__GNSSEphemeris __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__GNSSEphemeris __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GNSSEphemeris_
{
  using Type = GNSSEphemeris_<ContainerAllocator>;

  explicit GNSSEphemeris_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    toe(_init),
    toc(_init),
    ttr(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sat = 0l;
      this->iode = 0l;
      this->iodc = 0l;
      this->sva = 0l;
      this->svh = 0l;
      this->week = 0l;
      this->code = 0l;
      this->flag = 0l;
      this->a = 0.0;
      this->e = 0.0;
      this->i0 = 0.0;
      this->omg_0 = 0.0;
      this->omg = 0.0;
      this->m_0 = 0.0;
      this->deln = 0.0;
      this->omg_d = 0.0;
      this->idot = 0.0;
      this->crc = 0.0;
      this->crs = 0.0;
      this->cuc = 0.0;
      this->cus = 0.0;
      this->cic = 0.0;
      this->cis = 0.0;
      this->toes = 0.0;
      this->fit = 0.0;
      this->f0 = 0.0;
      this->f1 = 0.0;
      this->f2 = 0.0;
      std::fill<typename std::array<double, 4>::iterator, double>(this->tgd.begin(), this->tgd.end(), 0.0);
      this->a_dot = 0.0;
      this->ndot = 0.0;
    }
  }

  explicit GNSSEphemeris_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    toe(_alloc, _init),
    toc(_alloc, _init),
    ttr(_alloc, _init),
    tgd(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sat = 0l;
      this->iode = 0l;
      this->iodc = 0l;
      this->sva = 0l;
      this->svh = 0l;
      this->week = 0l;
      this->code = 0l;
      this->flag = 0l;
      this->a = 0.0;
      this->e = 0.0;
      this->i0 = 0.0;
      this->omg_0 = 0.0;
      this->omg = 0.0;
      this->m_0 = 0.0;
      this->deln = 0.0;
      this->omg_d = 0.0;
      this->idot = 0.0;
      this->crc = 0.0;
      this->crs = 0.0;
      this->cuc = 0.0;
      this->cus = 0.0;
      this->cic = 0.0;
      this->cis = 0.0;
      this->toes = 0.0;
      this->fit = 0.0;
      this->f0 = 0.0;
      this->f1 = 0.0;
      this->f2 = 0.0;
      std::fill<typename std::array<double, 4>::iterator, double>(this->tgd.begin(), this->tgd.end(), 0.0);
      this->a_dot = 0.0;
      this->ndot = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _sat_type =
    int32_t;
  _sat_type sat;
  using _iode_type =
    int32_t;
  _iode_type iode;
  using _iodc_type =
    int32_t;
  _iodc_type iodc;
  using _sva_type =
    int32_t;
  _sva_type sva;
  using _svh_type =
    int32_t;
  _svh_type svh;
  using _week_type =
    int32_t;
  _week_type week;
  using _code_type =
    int32_t;
  _code_type code;
  using _flag_type =
    int32_t;
  _flag_type flag;
  using _toe_type =
    inertial_sense_ros2::msg::GTime_<ContainerAllocator>;
  _toe_type toe;
  using _toc_type =
    inertial_sense_ros2::msg::GTime_<ContainerAllocator>;
  _toc_type toc;
  using _ttr_type =
    inertial_sense_ros2::msg::GTime_<ContainerAllocator>;
  _ttr_type ttr;
  using _a_type =
    double;
  _a_type a;
  using _e_type =
    double;
  _e_type e;
  using _i0_type =
    double;
  _i0_type i0;
  using _omg_0_type =
    double;
  _omg_0_type omg_0;
  using _omg_type =
    double;
  _omg_type omg;
  using _m_0_type =
    double;
  _m_0_type m_0;
  using _deln_type =
    double;
  _deln_type deln;
  using _omg_d_type =
    double;
  _omg_d_type omg_d;
  using _idot_type =
    double;
  _idot_type idot;
  using _crc_type =
    double;
  _crc_type crc;
  using _crs_type =
    double;
  _crs_type crs;
  using _cuc_type =
    double;
  _cuc_type cuc;
  using _cus_type =
    double;
  _cus_type cus;
  using _cic_type =
    double;
  _cic_type cic;
  using _cis_type =
    double;
  _cis_type cis;
  using _toes_type =
    double;
  _toes_type toes;
  using _fit_type =
    double;
  _fit_type fit;
  using _f0_type =
    double;
  _f0_type f0;
  using _f1_type =
    double;
  _f1_type f1;
  using _f2_type =
    double;
  _f2_type f2;
  using _tgd_type =
    std::array<double, 4>;
  _tgd_type tgd;
  using _a_dot_type =
    double;
  _a_dot_type a_dot;
  using _ndot_type =
    double;
  _ndot_type ndot;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__sat(
    const int32_t & _arg)
  {
    this->sat = _arg;
    return *this;
  }
  Type & set__iode(
    const int32_t & _arg)
  {
    this->iode = _arg;
    return *this;
  }
  Type & set__iodc(
    const int32_t & _arg)
  {
    this->iodc = _arg;
    return *this;
  }
  Type & set__sva(
    const int32_t & _arg)
  {
    this->sva = _arg;
    return *this;
  }
  Type & set__svh(
    const int32_t & _arg)
  {
    this->svh = _arg;
    return *this;
  }
  Type & set__week(
    const int32_t & _arg)
  {
    this->week = _arg;
    return *this;
  }
  Type & set__code(
    const int32_t & _arg)
  {
    this->code = _arg;
    return *this;
  }
  Type & set__flag(
    const int32_t & _arg)
  {
    this->flag = _arg;
    return *this;
  }
  Type & set__toe(
    const inertial_sense_ros2::msg::GTime_<ContainerAllocator> & _arg)
  {
    this->toe = _arg;
    return *this;
  }
  Type & set__toc(
    const inertial_sense_ros2::msg::GTime_<ContainerAllocator> & _arg)
  {
    this->toc = _arg;
    return *this;
  }
  Type & set__ttr(
    const inertial_sense_ros2::msg::GTime_<ContainerAllocator> & _arg)
  {
    this->ttr = _arg;
    return *this;
  }
  Type & set__a(
    const double & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__e(
    const double & _arg)
  {
    this->e = _arg;
    return *this;
  }
  Type & set__i0(
    const double & _arg)
  {
    this->i0 = _arg;
    return *this;
  }
  Type & set__omg_0(
    const double & _arg)
  {
    this->omg_0 = _arg;
    return *this;
  }
  Type & set__omg(
    const double & _arg)
  {
    this->omg = _arg;
    return *this;
  }
  Type & set__m_0(
    const double & _arg)
  {
    this->m_0 = _arg;
    return *this;
  }
  Type & set__deln(
    const double & _arg)
  {
    this->deln = _arg;
    return *this;
  }
  Type & set__omg_d(
    const double & _arg)
  {
    this->omg_d = _arg;
    return *this;
  }
  Type & set__idot(
    const double & _arg)
  {
    this->idot = _arg;
    return *this;
  }
  Type & set__crc(
    const double & _arg)
  {
    this->crc = _arg;
    return *this;
  }
  Type & set__crs(
    const double & _arg)
  {
    this->crs = _arg;
    return *this;
  }
  Type & set__cuc(
    const double & _arg)
  {
    this->cuc = _arg;
    return *this;
  }
  Type & set__cus(
    const double & _arg)
  {
    this->cus = _arg;
    return *this;
  }
  Type & set__cic(
    const double & _arg)
  {
    this->cic = _arg;
    return *this;
  }
  Type & set__cis(
    const double & _arg)
  {
    this->cis = _arg;
    return *this;
  }
  Type & set__toes(
    const double & _arg)
  {
    this->toes = _arg;
    return *this;
  }
  Type & set__fit(
    const double & _arg)
  {
    this->fit = _arg;
    return *this;
  }
  Type & set__f0(
    const double & _arg)
  {
    this->f0 = _arg;
    return *this;
  }
  Type & set__f1(
    const double & _arg)
  {
    this->f1 = _arg;
    return *this;
  }
  Type & set__f2(
    const double & _arg)
  {
    this->f2 = _arg;
    return *this;
  }
  Type & set__tgd(
    const std::array<double, 4> & _arg)
  {
    this->tgd = _arg;
    return *this;
  }
  Type & set__a_dot(
    const double & _arg)
  {
    this->a_dot = _arg;
    return *this;
  }
  Type & set__ndot(
    const double & _arg)
  {
    this->ndot = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::GNSSEphemeris_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::GNSSEphemeris_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GNSSEphemeris_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GNSSEphemeris_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GNSSEphemeris_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GNSSEphemeris_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GNSSEphemeris_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GNSSEphemeris_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GNSSEphemeris_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GNSSEphemeris_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__GNSSEphemeris
    std::shared_ptr<inertial_sense_ros2::msg::GNSSEphemeris_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__GNSSEphemeris
    std::shared_ptr<inertial_sense_ros2::msg::GNSSEphemeris_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GNSSEphemeris_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->sat != other.sat) {
      return false;
    }
    if (this->iode != other.iode) {
      return false;
    }
    if (this->iodc != other.iodc) {
      return false;
    }
    if (this->sva != other.sva) {
      return false;
    }
    if (this->svh != other.svh) {
      return false;
    }
    if (this->week != other.week) {
      return false;
    }
    if (this->code != other.code) {
      return false;
    }
    if (this->flag != other.flag) {
      return false;
    }
    if (this->toe != other.toe) {
      return false;
    }
    if (this->toc != other.toc) {
      return false;
    }
    if (this->ttr != other.ttr) {
      return false;
    }
    if (this->a != other.a) {
      return false;
    }
    if (this->e != other.e) {
      return false;
    }
    if (this->i0 != other.i0) {
      return false;
    }
    if (this->omg_0 != other.omg_0) {
      return false;
    }
    if (this->omg != other.omg) {
      return false;
    }
    if (this->m_0 != other.m_0) {
      return false;
    }
    if (this->deln != other.deln) {
      return false;
    }
    if (this->omg_d != other.omg_d) {
      return false;
    }
    if (this->idot != other.idot) {
      return false;
    }
    if (this->crc != other.crc) {
      return false;
    }
    if (this->crs != other.crs) {
      return false;
    }
    if (this->cuc != other.cuc) {
      return false;
    }
    if (this->cus != other.cus) {
      return false;
    }
    if (this->cic != other.cic) {
      return false;
    }
    if (this->cis != other.cis) {
      return false;
    }
    if (this->toes != other.toes) {
      return false;
    }
    if (this->fit != other.fit) {
      return false;
    }
    if (this->f0 != other.f0) {
      return false;
    }
    if (this->f1 != other.f1) {
      return false;
    }
    if (this->f2 != other.f2) {
      return false;
    }
    if (this->tgd != other.tgd) {
      return false;
    }
    if (this->a_dot != other.a_dot) {
      return false;
    }
    if (this->ndot != other.ndot) {
      return false;
    }
    return true;
  }
  bool operator!=(const GNSSEphemeris_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GNSSEphemeris_

// alias to use template instance with default allocator
using GNSSEphemeris =
  inertial_sense_ros2::msg::GNSSEphemeris_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_EPHEMERIS__STRUCT_HPP_
