// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/GNSSObservation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_observation.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__STRUCT_HPP_

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
// Member 'time'
#include "inertial_sense_ros2/msg/detail/g_time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__msg__GNSSObservation __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__GNSSObservation __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GNSSObservation_
{
  using Type = GNSSObservation_<ContainerAllocator>;

  explicit GNSSObservation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sat = 0;
      this->rcv = 0;
      this->snrr = 0;
      this->lli = 0;
      this->code = 0;
      this->qual_l = 0;
      this->qual_p = 0;
      this->l = 0.0;
      this->p = 0.0;
      this->d = 0.0f;
    }
  }

  explicit GNSSObservation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sat = 0;
      this->rcv = 0;
      this->snrr = 0;
      this->lli = 0;
      this->code = 0;
      this->qual_l = 0;
      this->qual_p = 0;
      this->l = 0.0;
      this->p = 0.0;
      this->d = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _time_type =
    inertial_sense_ros2::msg::GTime_<ContainerAllocator>;
  _time_type time;
  using _sat_type =
    uint8_t;
  _sat_type sat;
  using _rcv_type =
    uint8_t;
  _rcv_type rcv;
  using _snrr_type =
    uint8_t;
  _snrr_type snrr;
  using _lli_type =
    uint8_t;
  _lli_type lli;
  using _code_type =
    uint8_t;
  _code_type code;
  using _qual_l_type =
    uint8_t;
  _qual_l_type qual_l;
  using _qual_p_type =
    uint8_t;
  _qual_p_type qual_p;
  using _l_type =
    double;
  _l_type l;
  using _p_type =
    double;
  _p_type p;
  using _d_type =
    float;
  _d_type d;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__time(
    const inertial_sense_ros2::msg::GTime_<ContainerAllocator> & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__sat(
    const uint8_t & _arg)
  {
    this->sat = _arg;
    return *this;
  }
  Type & set__rcv(
    const uint8_t & _arg)
  {
    this->rcv = _arg;
    return *this;
  }
  Type & set__snrr(
    const uint8_t & _arg)
  {
    this->snrr = _arg;
    return *this;
  }
  Type & set__lli(
    const uint8_t & _arg)
  {
    this->lli = _arg;
    return *this;
  }
  Type & set__code(
    const uint8_t & _arg)
  {
    this->code = _arg;
    return *this;
  }
  Type & set__qual_l(
    const uint8_t & _arg)
  {
    this->qual_l = _arg;
    return *this;
  }
  Type & set__qual_p(
    const uint8_t & _arg)
  {
    this->qual_p = _arg;
    return *this;
  }
  Type & set__l(
    const double & _arg)
  {
    this->l = _arg;
    return *this;
  }
  Type & set__p(
    const double & _arg)
  {
    this->p = _arg;
    return *this;
  }
  Type & set__d(
    const float & _arg)
  {
    this->d = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__GNSSObservation
    std::shared_ptr<inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__GNSSObservation
    std::shared_ptr<inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GNSSObservation_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->time != other.time) {
      return false;
    }
    if (this->sat != other.sat) {
      return false;
    }
    if (this->rcv != other.rcv) {
      return false;
    }
    if (this->snrr != other.snrr) {
      return false;
    }
    if (this->lli != other.lli) {
      return false;
    }
    if (this->code != other.code) {
      return false;
    }
    if (this->qual_l != other.qual_l) {
      return false;
    }
    if (this->qual_p != other.qual_p) {
      return false;
    }
    if (this->l != other.l) {
      return false;
    }
    if (this->p != other.p) {
      return false;
    }
    if (this->d != other.d) {
      return false;
    }
    return true;
  }
  bool operator!=(const GNSSObservation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GNSSObservation_

// alias to use template instance with default allocator
using GNSSObservation =
  inertial_sense_ros2::msg::GNSSObservation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBSERVATION__STRUCT_HPP_
