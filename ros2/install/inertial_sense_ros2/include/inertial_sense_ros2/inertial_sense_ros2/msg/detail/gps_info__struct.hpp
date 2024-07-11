// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/GPSInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gps_info.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS_INFO__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS_INFO__STRUCT_HPP_

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
// Member 'sattelite_info'
#include "inertial_sense_ros2/msg/detail/sat_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__msg__GPSInfo __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__GPSInfo __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GPSInfo_
{
  using Type = GPSInfo_<ContainerAllocator>;

  explicit GPSInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_sats = 0ul;
      this->sattelite_info.fill(inertial_sense_ros2::msg::SatInfo_<ContainerAllocator>{_init});
    }
  }

  explicit GPSInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    sattelite_info(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_sats = 0ul;
      this->sattelite_info.fill(inertial_sense_ros2::msg::SatInfo_<ContainerAllocator>{_alloc, _init});
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _num_sats_type =
    uint32_t;
  _num_sats_type num_sats;
  using _sattelite_info_type =
    std::array<inertial_sense_ros2::msg::SatInfo_<ContainerAllocator>, 50>;
  _sattelite_info_type sattelite_info;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__num_sats(
    const uint32_t & _arg)
  {
    this->num_sats = _arg;
    return *this;
  }
  Type & set__sattelite_info(
    const std::array<inertial_sense_ros2::msg::SatInfo_<ContainerAllocator>, 50> & _arg)
  {
    this->sattelite_info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::GPSInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::GPSInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GPSInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GPSInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GPSInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GPSInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GPSInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GPSInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GPSInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GPSInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__GPSInfo
    std::shared_ptr<inertial_sense_ros2::msg::GPSInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__GPSInfo
    std::shared_ptr<inertial_sense_ros2::msg::GPSInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GPSInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->num_sats != other.num_sats) {
      return false;
    }
    if (this->sattelite_info != other.sattelite_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const GPSInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GPSInfo_

// alias to use template instance with default allocator
using GPSInfo =
  inertial_sense_ros2::msg::GPSInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GPS_INFO__STRUCT_HPP_
