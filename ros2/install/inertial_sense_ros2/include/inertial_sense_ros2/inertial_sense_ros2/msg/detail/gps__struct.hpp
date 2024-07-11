// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/GPS.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gps.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS__STRUCT_HPP_

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
// Member 'pos_ecef'
// Member 'vel_ecef'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__msg__GPS __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__GPS __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GPS_
{
  using Type = GPS_<ContainerAllocator>;

  explicit GPS_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pos_ecef(_init),
    vel_ecef(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->week = 0ul;
      this->num_sat = 0;
      this->status = 0ul;
      this->cno = 0l;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->hmsl = 0.0f;
      this->hacc = 0.0f;
      this->vacc = 0.0f;
      this->sacc = 0.0f;
      this->pdop = 0.0f;
    }
  }

  explicit GPS_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pos_ecef(_alloc, _init),
    vel_ecef(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->week = 0ul;
      this->num_sat = 0;
      this->status = 0ul;
      this->cno = 0l;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->hmsl = 0.0f;
      this->hacc = 0.0f;
      this->vacc = 0.0f;
      this->sacc = 0.0f;
      this->pdop = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _week_type =
    uint32_t;
  _week_type week;
  using _num_sat_type =
    int8_t;
  _num_sat_type num_sat;
  using _status_type =
    uint32_t;
  _status_type status;
  using _cno_type =
    int32_t;
  _cno_type cno;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _pos_ecef_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _pos_ecef_type pos_ecef;
  using _vel_ecef_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _vel_ecef_type vel_ecef;
  using _hmsl_type =
    float;
  _hmsl_type hmsl;
  using _hacc_type =
    float;
  _hacc_type hacc;
  using _vacc_type =
    float;
  _vacc_type vacc;
  using _sacc_type =
    float;
  _sacc_type sacc;
  using _pdop_type =
    float;
  _pdop_type pdop;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__week(
    const uint32_t & _arg)
  {
    this->week = _arg;
    return *this;
  }
  Type & set__num_sat(
    const int8_t & _arg)
  {
    this->num_sat = _arg;
    return *this;
  }
  Type & set__status(
    const uint32_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__cno(
    const int32_t & _arg)
  {
    this->cno = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__pos_ecef(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->pos_ecef = _arg;
    return *this;
  }
  Type & set__vel_ecef(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->vel_ecef = _arg;
    return *this;
  }
  Type & set__hmsl(
    const float & _arg)
  {
    this->hmsl = _arg;
    return *this;
  }
  Type & set__hacc(
    const float & _arg)
  {
    this->hacc = _arg;
    return *this;
  }
  Type & set__vacc(
    const float & _arg)
  {
    this->vacc = _arg;
    return *this;
  }
  Type & set__sacc(
    const float & _arg)
  {
    this->sacc = _arg;
    return *this;
  }
  Type & set__pdop(
    const float & _arg)
  {
    this->pdop = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::GPS_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::GPS_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GPS_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GPS_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GPS_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GPS_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GPS_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GPS_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GPS_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GPS_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__GPS
    std::shared_ptr<inertial_sense_ros2::msg::GPS_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__GPS
    std::shared_ptr<inertial_sense_ros2::msg::GPS_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GPS_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->week != other.week) {
      return false;
    }
    if (this->num_sat != other.num_sat) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->cno != other.cno) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->pos_ecef != other.pos_ecef) {
      return false;
    }
    if (this->vel_ecef != other.vel_ecef) {
      return false;
    }
    if (this->hmsl != other.hmsl) {
      return false;
    }
    if (this->hacc != other.hacc) {
      return false;
    }
    if (this->vacc != other.vacc) {
      return false;
    }
    if (this->sacc != other.sacc) {
      return false;
    }
    if (this->pdop != other.pdop) {
      return false;
    }
    return true;
  }
  bool operator!=(const GPS_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GPS_

// alias to use template instance with default allocator
using GPS =
  inertial_sense_ros2::msg::GPS_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS__STRUCT_HPP_
