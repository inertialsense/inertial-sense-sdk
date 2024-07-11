// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/SatInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/sat_info.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__SAT_INFO__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__SAT_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__msg__SatInfo __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__SatInfo __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SatInfo_
{
  using Type = SatInfo_<ContainerAllocator>;

  explicit SatInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sat_id = 0ul;
      this->cno = 0ul;
    }
  }

  explicit SatInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sat_id = 0ul;
      this->cno = 0ul;
    }
  }

  // field types and members
  using _sat_id_type =
    uint32_t;
  _sat_id_type sat_id;
  using _cno_type =
    uint32_t;
  _cno_type cno;

  // setters for named parameter idiom
  Type & set__sat_id(
    const uint32_t & _arg)
  {
    this->sat_id = _arg;
    return *this;
  }
  Type & set__cno(
    const uint32_t & _arg)
  {
    this->cno = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::SatInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::SatInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::SatInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::SatInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::SatInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::SatInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::SatInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::SatInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::SatInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::SatInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__SatInfo
    std::shared_ptr<inertial_sense_ros2::msg::SatInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__SatInfo
    std::shared_ptr<inertial_sense_ros2::msg::SatInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SatInfo_ & other) const
  {
    if (this->sat_id != other.sat_id) {
      return false;
    }
    if (this->cno != other.cno) {
      return false;
    }
    return true;
  }
  bool operator!=(const SatInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SatInfo_

// alias to use template instance with default allocator
using SatInfo =
  inertial_sense_ros2::msg::SatInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__SAT_INFO__STRUCT_HPP_
