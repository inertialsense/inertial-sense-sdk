// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/RTKRel.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/rtk_rel.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_REL__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_REL__STRUCT_HPP_

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
// Member 'vector_base_to_rover'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__msg__RTKRel __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__RTKRel __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RTKRel_
{
  using Type = RTKRel_<ContainerAllocator>;

  explicit RTKRel_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    vector_base_to_rover(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->differential_age = 0.0f;
      this->ar_ratio = 0.0f;
      this->e_gps_status_fix = 0;
      this->distance_base_to_rover = 0.0f;
      this->heading_base_to_rover = 0.0f;
    }
  }

  explicit RTKRel_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    vector_base_to_rover(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->differential_age = 0.0f;
      this->ar_ratio = 0.0f;
      this->e_gps_status_fix = 0;
      this->distance_base_to_rover = 0.0f;
      this->heading_base_to_rover = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _differential_age_type =
    float;
  _differential_age_type differential_age;
  using _ar_ratio_type =
    float;
  _ar_ratio_type ar_ratio;
  using _e_gps_status_fix_type =
    uint8_t;
  _e_gps_status_fix_type e_gps_status_fix;
  using _vector_base_to_rover_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _vector_base_to_rover_type vector_base_to_rover;
  using _distance_base_to_rover_type =
    float;
  _distance_base_to_rover_type distance_base_to_rover;
  using _heading_base_to_rover_type =
    float;
  _heading_base_to_rover_type heading_base_to_rover;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__differential_age(
    const float & _arg)
  {
    this->differential_age = _arg;
    return *this;
  }
  Type & set__ar_ratio(
    const float & _arg)
  {
    this->ar_ratio = _arg;
    return *this;
  }
  Type & set__e_gps_status_fix(
    const uint8_t & _arg)
  {
    this->e_gps_status_fix = _arg;
    return *this;
  }
  Type & set__vector_base_to_rover(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->vector_base_to_rover = _arg;
    return *this;
  }
  Type & set__distance_base_to_rover(
    const float & _arg)
  {
    this->distance_base_to_rover = _arg;
    return *this;
  }
  Type & set__heading_base_to_rover(
    const float & _arg)
  {
    this->heading_base_to_rover = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t GPS_STATUS_FIX_3D =
    1u;
  static constexpr uint8_t GPS_STATUS_FIX_RTK_SINGLE =
    2u;
  static constexpr uint8_t GPS_STATUS_FIX_RTK_FLOAT =
    3u;
  static constexpr uint8_t GPS_STATUS_FIX_RTK_FIX =
    4u;
  static constexpr uint8_t GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD =
    5u;

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::RTKRel_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::RTKRel_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::RTKRel_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::RTKRel_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::RTKRel_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::RTKRel_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::RTKRel_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::RTKRel_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::RTKRel_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::RTKRel_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__RTKRel
    std::shared_ptr<inertial_sense_ros2::msg::RTKRel_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__RTKRel
    std::shared_ptr<inertial_sense_ros2::msg::RTKRel_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RTKRel_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->differential_age != other.differential_age) {
      return false;
    }
    if (this->ar_ratio != other.ar_ratio) {
      return false;
    }
    if (this->e_gps_status_fix != other.e_gps_status_fix) {
      return false;
    }
    if (this->vector_base_to_rover != other.vector_base_to_rover) {
      return false;
    }
    if (this->distance_base_to_rover != other.distance_base_to_rover) {
      return false;
    }
    if (this->heading_base_to_rover != other.heading_base_to_rover) {
      return false;
    }
    return true;
  }
  bool operator!=(const RTKRel_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RTKRel_

// alias to use template instance with default allocator
using RTKRel =
  inertial_sense_ros2::msg::RTKRel_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKRel_<ContainerAllocator>::GPS_STATUS_FIX_3D;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKRel_<ContainerAllocator>::GPS_STATUS_FIX_RTK_SINGLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKRel_<ContainerAllocator>::GPS_STATUS_FIX_RTK_FLOAT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKRel_<ContainerAllocator>::GPS_STATUS_FIX_RTK_FIX;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RTKRel_<ContainerAllocator>::GPS_STATUS_FLAGS_RTK_FIX_AND_HOLD;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__RTK_REL__STRUCT_HPP_
