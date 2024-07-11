// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/GTime.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/g_time.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__G_TIME__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__G_TIME__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__msg__GTime __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__GTime __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GTime_
{
  using Type = GTime_<ContainerAllocator>;

  explicit GTime_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->sec = 0.0;
    }
  }

  explicit GTime_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->time = 0ll;
      this->sec = 0.0;
    }
  }

  // field types and members
  using _time_type =
    int64_t;
  _time_type time;
  using _sec_type =
    double;
  _sec_type sec;

  // setters for named parameter idiom
  Type & set__time(
    const int64_t & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__sec(
    const double & _arg)
  {
    this->sec = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::GTime_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::GTime_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GTime_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GTime_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GTime_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GTime_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GTime_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GTime_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GTime_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GTime_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__GTime
    std::shared_ptr<inertial_sense_ros2::msg::GTime_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__GTime
    std::shared_ptr<inertial_sense_ros2::msg::GTime_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GTime_ & other) const
  {
    if (this->time != other.time) {
      return false;
    }
    if (this->sec != other.sec) {
      return false;
    }
    return true;
  }
  bool operator!=(const GTime_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GTime_

// alias to use template instance with default allocator
using GTime =
  inertial_sense_ros2::msg::GTime_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__G_TIME__STRUCT_HPP_
