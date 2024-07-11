// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/INL2States.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/inl2_states.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__INL2_STATES__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__INL2_STATES__STRUCT_HPP_

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
// Member 'quat_ecef'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
// Member 'vel_ecef'
// Member 'pos_ecef'
// Member 'gyro_bias'
// Member 'accel_bias'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__msg__INL2States __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__INL2States __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct INL2States_
{
  using Type = INL2States_<ContainerAllocator>;

  explicit INL2States_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    quat_ecef(_init),
    vel_ecef(_init),
    pos_ecef(_init),
    gyro_bias(_init),
    accel_bias(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->baro_bias = 0.0f;
      this->mag_dec = 0.0f;
      this->mag_inc = 0.0f;
    }
  }

  explicit INL2States_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    quat_ecef(_alloc, _init),
    vel_ecef(_alloc, _init),
    pos_ecef(_alloc, _init),
    gyro_bias(_alloc, _init),
    accel_bias(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->baro_bias = 0.0f;
      this->mag_dec = 0.0f;
      this->mag_inc = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _quat_ecef_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _quat_ecef_type quat_ecef;
  using _vel_ecef_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _vel_ecef_type vel_ecef;
  using _pos_ecef_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _pos_ecef_type pos_ecef;
  using _gyro_bias_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _gyro_bias_type gyro_bias;
  using _accel_bias_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _accel_bias_type accel_bias;
  using _baro_bias_type =
    float;
  _baro_bias_type baro_bias;
  using _mag_dec_type =
    float;
  _mag_dec_type mag_dec;
  using _mag_inc_type =
    float;
  _mag_inc_type mag_inc;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__quat_ecef(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->quat_ecef = _arg;
    return *this;
  }
  Type & set__vel_ecef(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->vel_ecef = _arg;
    return *this;
  }
  Type & set__pos_ecef(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->pos_ecef = _arg;
    return *this;
  }
  Type & set__gyro_bias(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->gyro_bias = _arg;
    return *this;
  }
  Type & set__accel_bias(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->accel_bias = _arg;
    return *this;
  }
  Type & set__baro_bias(
    const float & _arg)
  {
    this->baro_bias = _arg;
    return *this;
  }
  Type & set__mag_dec(
    const float & _arg)
  {
    this->mag_dec = _arg;
    return *this;
  }
  Type & set__mag_inc(
    const float & _arg)
  {
    this->mag_inc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::INL2States_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::INL2States_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::INL2States_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::INL2States_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::INL2States_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::INL2States_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::INL2States_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::INL2States_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::INL2States_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::INL2States_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__INL2States
    std::shared_ptr<inertial_sense_ros2::msg::INL2States_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__INL2States
    std::shared_ptr<inertial_sense_ros2::msg::INL2States_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const INL2States_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->quat_ecef != other.quat_ecef) {
      return false;
    }
    if (this->vel_ecef != other.vel_ecef) {
      return false;
    }
    if (this->pos_ecef != other.pos_ecef) {
      return false;
    }
    if (this->gyro_bias != other.gyro_bias) {
      return false;
    }
    if (this->accel_bias != other.accel_bias) {
      return false;
    }
    if (this->baro_bias != other.baro_bias) {
      return false;
    }
    if (this->mag_dec != other.mag_dec) {
      return false;
    }
    if (this->mag_inc != other.mag_inc) {
      return false;
    }
    return true;
  }
  bool operator!=(const INL2States_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct INL2States_

// alias to use template instance with default allocator
using INL2States =
  inertial_sense_ros2::msg::INL2States_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__INL2_STATES__STRUCT_HPP_
