// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/GNSSObsVec.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/gnss_obs_vec.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__STRUCT_HPP_

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
// Member 'obs'
#include "inertial_sense_ros2/msg/detail/gnss_observation__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__msg__GNSSObsVec __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__GNSSObsVec __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GNSSObsVec_
{
  using Type = GNSSObsVec_<ContainerAllocator>;

  explicit GNSSObsVec_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    time(_init)
  {
    (void)_init;
  }

  explicit GNSSObsVec_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    time(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _time_type =
    inertial_sense_ros2::msg::GTime_<ContainerAllocator>;
  _time_type time;
  using _obs_type =
    std::vector<inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator>>>;
  _obs_type obs;

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
  Type & set__obs(
    const std::vector<inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<inertial_sense_ros2::msg::GNSSObservation_<ContainerAllocator>>> & _arg)
  {
    this->obs = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::GNSSObsVec_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::GNSSObsVec_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GNSSObsVec_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GNSSObsVec_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GNSSObsVec_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GNSSObsVec_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GNSSObsVec_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GNSSObsVec_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GNSSObsVec_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GNSSObsVec_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__GNSSObsVec
    std::shared_ptr<inertial_sense_ros2::msg::GNSSObsVec_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__GNSSObsVec
    std::shared_ptr<inertial_sense_ros2::msg::GNSSObsVec_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GNSSObsVec_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->time != other.time) {
      return false;
    }
    if (this->obs != other.obs) {
      return false;
    }
    return true;
  }
  bool operator!=(const GNSSObsVec_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GNSSObsVec_

// alias to use template instance with default allocator
using GNSSObsVec =
  inertial_sense_ros2::msg::GNSSObsVec_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GNSS_OBS_VEC__STRUCT_HPP_
