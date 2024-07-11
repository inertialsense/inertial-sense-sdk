// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/PIMU.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/pimu.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__PIMU__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__PIMU__STRUCT_HPP_

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
// Member 'dtheta'
// Member 'dvel'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__msg__PIMU __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__PIMU __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PIMU_
{
  using Type = PIMU_<ContainerAllocator>;

  explicit PIMU_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    dtheta(_init),
    dvel(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dt = 0.0;
    }
  }

  explicit PIMU_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    dtheta(_alloc, _init),
    dvel(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dt = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _dtheta_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _dtheta_type dtheta;
  using _dvel_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _dvel_type dvel;
  using _dt_type =
    double;
  _dt_type dt;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__dtheta(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->dtheta = _arg;
    return *this;
  }
  Type & set__dvel(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->dvel = _arg;
    return *this;
  }
  Type & set__dt(
    const double & _arg)
  {
    this->dt = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::PIMU_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::PIMU_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::PIMU_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::PIMU_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::PIMU_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::PIMU_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::PIMU_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::PIMU_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::PIMU_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::PIMU_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__PIMU
    std::shared_ptr<inertial_sense_ros2::msg::PIMU_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__PIMU
    std::shared_ptr<inertial_sense_ros2::msg::PIMU_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PIMU_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->dtheta != other.dtheta) {
      return false;
    }
    if (this->dvel != other.dvel) {
      return false;
    }
    if (this->dt != other.dt) {
      return false;
    }
    return true;
  }
  bool operator!=(const PIMU_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PIMU_

// alias to use template instance with default allocator
using PIMU =
  inertial_sense_ros2::msg::PIMU_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__PIMU__STRUCT_HPP_
