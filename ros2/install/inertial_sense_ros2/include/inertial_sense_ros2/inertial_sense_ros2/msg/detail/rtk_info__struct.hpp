// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/RTKInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/rtk_info.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_INFO__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_INFO__STRUCT_HPP_

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
# define DEPRECATED__inertial_sense_ros2__msg__RTKInfo __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__RTKInfo __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RTKInfo_
{
  using Type = RTKInfo_<ContainerAllocator>;

  explicit RTKInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->base_lla.begin(), this->base_lla.end(), 0.0f);
      this->cycle_slip_count = 0ul;
      this->rover_obs = 0ul;
      this->base_obs = 0ul;
      this->rover_eph = 0ul;
      this->base_eph = 0ul;
      this->base_ant_count = 0ul;
    }
  }

  explicit RTKInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    base_lla(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->base_lla.begin(), this->base_lla.end(), 0.0f);
      this->cycle_slip_count = 0ul;
      this->rover_obs = 0ul;
      this->base_obs = 0ul;
      this->rover_eph = 0ul;
      this->base_eph = 0ul;
      this->base_ant_count = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _base_lla_type =
    std::array<float, 3>;
  _base_lla_type base_lla;
  using _cycle_slip_count_type =
    uint32_t;
  _cycle_slip_count_type cycle_slip_count;
  using _rover_obs_type =
    uint32_t;
  _rover_obs_type rover_obs;
  using _base_obs_type =
    uint32_t;
  _base_obs_type base_obs;
  using _rover_eph_type =
    uint32_t;
  _rover_eph_type rover_eph;
  using _base_eph_type =
    uint32_t;
  _base_eph_type base_eph;
  using _base_ant_count_type =
    uint32_t;
  _base_ant_count_type base_ant_count;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__base_lla(
    const std::array<float, 3> & _arg)
  {
    this->base_lla = _arg;
    return *this;
  }
  Type & set__cycle_slip_count(
    const uint32_t & _arg)
  {
    this->cycle_slip_count = _arg;
    return *this;
  }
  Type & set__rover_obs(
    const uint32_t & _arg)
  {
    this->rover_obs = _arg;
    return *this;
  }
  Type & set__base_obs(
    const uint32_t & _arg)
  {
    this->base_obs = _arg;
    return *this;
  }
  Type & set__rover_eph(
    const uint32_t & _arg)
  {
    this->rover_eph = _arg;
    return *this;
  }
  Type & set__base_eph(
    const uint32_t & _arg)
  {
    this->base_eph = _arg;
    return *this;
  }
  Type & set__base_ant_count(
    const uint32_t & _arg)
  {
    this->base_ant_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::RTKInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::RTKInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::RTKInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::RTKInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::RTKInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::RTKInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::RTKInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::RTKInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::RTKInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::RTKInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__RTKInfo
    std::shared_ptr<inertial_sense_ros2::msg::RTKInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__RTKInfo
    std::shared_ptr<inertial_sense_ros2::msg::RTKInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RTKInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->base_lla != other.base_lla) {
      return false;
    }
    if (this->cycle_slip_count != other.cycle_slip_count) {
      return false;
    }
    if (this->rover_obs != other.rover_obs) {
      return false;
    }
    if (this->base_obs != other.base_obs) {
      return false;
    }
    if (this->rover_eph != other.rover_eph) {
      return false;
    }
    if (this->base_eph != other.base_eph) {
      return false;
    }
    if (this->base_ant_count != other.base_ant_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const RTKInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RTKInfo_

// alias to use template instance with default allocator
using RTKInfo =
  inertial_sense_ros2::msg::RTKInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_INFO__STRUCT_HPP_
