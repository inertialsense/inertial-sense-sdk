// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:msg/GlonassEphemeris.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/msg/glonass_ephemeris.hpp"


#ifndef INERTIAL_SENSE_ROS2__MSG__DETAIL__GLONASS_EPHEMERIS__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__MSG__DETAIL__GLONASS_EPHEMERIS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'toe'
// Member 'tof'
#include "inertial_sense_ros2/msg/detail/g_time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__msg__GlonassEphemeris __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__msg__GlonassEphemeris __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GlonassEphemeris_
{
  using Type = GlonassEphemeris_<ContainerAllocator>;

  explicit GlonassEphemeris_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : toe(_init),
    tof(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sat = 0l;
      this->iode = 0l;
      this->frq = 0l;
      this->svh = 0l;
      this->sva = 0l;
      this->age = 0l;
      std::fill<typename std::array<double, 3>::iterator, double>(this->pos.begin(), this->pos.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->vel.begin(), this->vel.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->acc.begin(), this->acc.end(), 0.0);
      this->taun = 0.0;
      this->gamn = 0.0;
      this->dtaun = 0.0;
    }
  }

  explicit GlonassEphemeris_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : toe(_alloc, _init),
    tof(_alloc, _init),
    pos(_alloc),
    vel(_alloc),
    acc(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sat = 0l;
      this->iode = 0l;
      this->frq = 0l;
      this->svh = 0l;
      this->sva = 0l;
      this->age = 0l;
      std::fill<typename std::array<double, 3>::iterator, double>(this->pos.begin(), this->pos.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->vel.begin(), this->vel.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->acc.begin(), this->acc.end(), 0.0);
      this->taun = 0.0;
      this->gamn = 0.0;
      this->dtaun = 0.0;
    }
  }

  // field types and members
  using _sat_type =
    int32_t;
  _sat_type sat;
  using _iode_type =
    int32_t;
  _iode_type iode;
  using _frq_type =
    int32_t;
  _frq_type frq;
  using _svh_type =
    int32_t;
  _svh_type svh;
  using _sva_type =
    int32_t;
  _sva_type sva;
  using _age_type =
    int32_t;
  _age_type age;
  using _toe_type =
    inertial_sense_ros2::msg::GTime_<ContainerAllocator>;
  _toe_type toe;
  using _tof_type =
    inertial_sense_ros2::msg::GTime_<ContainerAllocator>;
  _tof_type tof;
  using _pos_type =
    std::array<double, 3>;
  _pos_type pos;
  using _vel_type =
    std::array<double, 3>;
  _vel_type vel;
  using _acc_type =
    std::array<double, 3>;
  _acc_type acc;
  using _taun_type =
    double;
  _taun_type taun;
  using _gamn_type =
    double;
  _gamn_type gamn;
  using _dtaun_type =
    double;
  _dtaun_type dtaun;

  // setters for named parameter idiom
  Type & set__sat(
    const int32_t & _arg)
  {
    this->sat = _arg;
    return *this;
  }
  Type & set__iode(
    const int32_t & _arg)
  {
    this->iode = _arg;
    return *this;
  }
  Type & set__frq(
    const int32_t & _arg)
  {
    this->frq = _arg;
    return *this;
  }
  Type & set__svh(
    const int32_t & _arg)
  {
    this->svh = _arg;
    return *this;
  }
  Type & set__sva(
    const int32_t & _arg)
  {
    this->sva = _arg;
    return *this;
  }
  Type & set__age(
    const int32_t & _arg)
  {
    this->age = _arg;
    return *this;
  }
  Type & set__toe(
    const inertial_sense_ros2::msg::GTime_<ContainerAllocator> & _arg)
  {
    this->toe = _arg;
    return *this;
  }
  Type & set__tof(
    const inertial_sense_ros2::msg::GTime_<ContainerAllocator> & _arg)
  {
    this->tof = _arg;
    return *this;
  }
  Type & set__pos(
    const std::array<double, 3> & _arg)
  {
    this->pos = _arg;
    return *this;
  }
  Type & set__vel(
    const std::array<double, 3> & _arg)
  {
    this->vel = _arg;
    return *this;
  }
  Type & set__acc(
    const std::array<double, 3> & _arg)
  {
    this->acc = _arg;
    return *this;
  }
  Type & set__taun(
    const double & _arg)
  {
    this->taun = _arg;
    return *this;
  }
  Type & set__gamn(
    const double & _arg)
  {
    this->gamn = _arg;
    return *this;
  }
  Type & set__dtaun(
    const double & _arg)
  {
    this->dtaun = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::msg::GlonassEphemeris_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::msg::GlonassEphemeris_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GlonassEphemeris_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::msg::GlonassEphemeris_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GlonassEphemeris_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GlonassEphemeris_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::msg::GlonassEphemeris_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::msg::GlonassEphemeris_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GlonassEphemeris_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::msg::GlonassEphemeris_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__msg__GlonassEphemeris
    std::shared_ptr<inertial_sense_ros2::msg::GlonassEphemeris_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__msg__GlonassEphemeris
    std::shared_ptr<inertial_sense_ros2::msg::GlonassEphemeris_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GlonassEphemeris_ & other) const
  {
    if (this->sat != other.sat) {
      return false;
    }
    if (this->iode != other.iode) {
      return false;
    }
    if (this->frq != other.frq) {
      return false;
    }
    if (this->svh != other.svh) {
      return false;
    }
    if (this->sva != other.sva) {
      return false;
    }
    if (this->age != other.age) {
      return false;
    }
    if (this->toe != other.toe) {
      return false;
    }
    if (this->tof != other.tof) {
      return false;
    }
    if (this->pos != other.pos) {
      return false;
    }
    if (this->vel != other.vel) {
      return false;
    }
    if (this->acc != other.acc) {
      return false;
    }
    if (this->taun != other.taun) {
      return false;
    }
    if (this->gamn != other.gamn) {
      return false;
    }
    if (this->dtaun != other.dtaun) {
      return false;
    }
    return true;
  }
  bool operator!=(const GlonassEphemeris_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GlonassEphemeris_

// alias to use template instance with default allocator
using GlonassEphemeris =
  inertial_sense_ros2::msg::GlonassEphemeris_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__MSG__DETAIL__GLONASS_EPHEMERIS__STRUCT_HPP_
