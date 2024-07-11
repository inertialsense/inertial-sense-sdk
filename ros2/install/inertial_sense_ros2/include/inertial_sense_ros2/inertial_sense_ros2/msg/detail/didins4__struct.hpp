// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/DIDINS4.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/didins4.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS4__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS4__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__msg__DIDINS4 __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__DIDINS4 __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DIDINS4_
{
  using Type = DIDINS4_<ContainerAllocator>;

  explicit DIDINS4_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->week = 0ul;
      this->time_of_week = 0.0;
      this->ins_status = 0ul;
      this->hdw_status = 0ul;
      std::fill<typename std::array<float, 4>::iterator, float>(this->qe2b.begin(), this->qe2b.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->ve.begin(), this->ve.end(), 0.0f);
      std::fill<typename std::array<double, 3>::iterator, double>(this->ecef.begin(), this->ecef.end(), 0.0);
    }
  }

  explicit DIDINS4_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    qe2b(_alloc),
    ve(_alloc),
    ecef(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->week = 0ul;
      this->time_of_week = 0.0;
      this->ins_status = 0ul;
      this->hdw_status = 0ul;
      std::fill<typename std::array<float, 4>::iterator, float>(this->qe2b.begin(), this->qe2b.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->ve.begin(), this->ve.end(), 0.0f);
      std::fill<typename std::array<double, 3>::iterator, double>(this->ecef.begin(), this->ecef.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _week_type =
    uint32_t;
  _week_type week;
  using _time_of_week_type =
    double;
  _time_of_week_type time_of_week;
  using _ins_status_type =
    uint32_t;
  _ins_status_type ins_status;
  using _hdw_status_type =
    uint32_t;
  _hdw_status_type hdw_status;
  using _qe2b_type =
    std::array<float, 4>;
  _qe2b_type qe2b;
  using _ve_type =
    std::array<float, 3>;
  _ve_type ve;
  using _ecef_type =
    std::array<double, 3>;
  _ecef_type ecef;

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
  Type & set__time_of_week(
    const double & _arg)
  {
    this->time_of_week = _arg;
    return *this;
  }
  Type & set__ins_status(
    const uint32_t & _arg)
  {
    this->ins_status = _arg;
    return *this;
  }
  Type & set__hdw_status(
    const uint32_t & _arg)
  {
    this->hdw_status = _arg;
    return *this;
  }
  Type & set__qe2b(
    const std::array<float, 4> & _arg)
  {
    this->qe2b = _arg;
    return *this;
  }
  Type & set__ve(
    const std::array<float, 3> & _arg)
  {
    this->ve = _arg;
    return *this;
  }
  Type & set__ecef(
    const std::array<double, 3> & _arg)
  {
    this->ecef = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::DIDINS4_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::DIDINS4_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::DIDINS4_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::DIDINS4_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::DIDINS4_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::DIDINS4_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::DIDINS4_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::DIDINS4_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::DIDINS4_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::DIDINS4_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__DIDINS4
    std::shared_ptr<inertial_sense_ros2::msg::DIDINS4_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__DIDINS4
    std::shared_ptr<inertial_sense_ros2::msg::DIDINS4_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DIDINS4_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->week != other.week) {
      return false;
    }
    if (this->time_of_week != other.time_of_week) {
      return false;
    }
    if (this->ins_status != other.ins_status) {
      return false;
    }
    if (this->hdw_status != other.hdw_status) {
      return false;
    }
    if (this->qe2b != other.qe2b) {
      return false;
    }
    if (this->ve != other.ve) {
      return false;
    }
    if (this->ecef != other.ecef) {
      return false;
    }
    return true;
  }
  bool operator!=(const DIDINS4_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DIDINS4_

// alias to use template instance with default allocator
using DIDINS4 =
  inertial_sense_ros2::msg::DIDINS4_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__DIDINS4__STRUCT_HPP_
