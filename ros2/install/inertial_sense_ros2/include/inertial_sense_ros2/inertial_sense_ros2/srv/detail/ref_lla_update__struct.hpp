// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inertial_sense_ros2:srv/RefLLAUpdate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "inertial_sense_ros2/srv/ref_lla_update.hpp"


#ifndef INERTIAL_SENSE_ROS2__SRV__DETAIL__REF_LLA_UPDATE__STRUCT_HPP_
#define INERTIAL_SENSE_ROS2__SRV__DETAIL__REF_LLA_UPDATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__srv__RefLLAUpdate_Request __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__srv__RefLLAUpdate_Request __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RefLLAUpdate_Request_
{
  using Type = RefLLAUpdate_Request_<ContainerAllocator>;

  explicit RefLLAUpdate_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 3>::iterator, double>(this->lla.begin(), this->lla.end(), 0.0);
    }
  }

  explicit RefLLAUpdate_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : lla(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 3>::iterator, double>(this->lla.begin(), this->lla.end(), 0.0);
    }
  }

  // field types and members
  using _lla_type =
    std::array<double, 3>;
  _lla_type lla;

  // setters for named parameter idiom
  Type & set__lla(
    const std::array<double, 3> & _arg)
  {
    this->lla = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__srv__RefLLAUpdate_Request
    std::shared_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__srv__RefLLAUpdate_Request
    std::shared_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RefLLAUpdate_Request_ & other) const
  {
    if (this->lla != other.lla) {
      return false;
    }
    return true;
  }
  bool operator!=(const RefLLAUpdate_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RefLLAUpdate_Request_

// alias to use template instance with default allocator
using RefLLAUpdate_Request =
  inertial_sense_ros2::srv::RefLLAUpdate_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace inertial_sense_ros2


#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__srv__RefLLAUpdate_Response __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__srv__RefLLAUpdate_Response __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RefLLAUpdate_Response_
{
  using Type = RefLLAUpdate_Response_<ContainerAllocator>;

  explicit RefLLAUpdate_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit RefLLAUpdate_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__srv__RefLLAUpdate_Response
    std::shared_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__srv__RefLLAUpdate_Response
    std::shared_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RefLLAUpdate_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const RefLLAUpdate_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RefLLAUpdate_Response_

// alias to use template instance with default allocator
using RefLLAUpdate_Response =
  inertial_sense_ros2::srv::RefLLAUpdate_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace inertial_sense_ros2


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__inertial_sense_ros2__srv__RefLLAUpdate_Event __attribute__((deprecated))
#else
# define DEPRECATED__inertial_sense_ros2__srv__RefLLAUpdate_Event __declspec(deprecated)
#endif

namespace inertial_sense_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RefLLAUpdate_Event_
{
  using Type = RefLLAUpdate_Event_<ContainerAllocator>;

  explicit RefLLAUpdate_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit RefLLAUpdate_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<inertial_sense_ros2::srv::RefLLAUpdate_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<inertial_sense_ros2::srv::RefLLAUpdate_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inertial_sense_ros2::srv::RefLLAUpdate_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const inertial_sense_ros2::srv::RefLLAUpdate_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::srv::RefLLAUpdate_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inertial_sense_ros2::srv::RefLLAUpdate_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inertial_sense_ros2__srv__RefLLAUpdate_Event
    std::shared_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inertial_sense_ros2__srv__RefLLAUpdate_Event
    std::shared_ptr<inertial_sense_ros2::srv::RefLLAUpdate_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RefLLAUpdate_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const RefLLAUpdate_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RefLLAUpdate_Event_

// alias to use template instance with default allocator
using RefLLAUpdate_Event =
  inertial_sense_ros2::srv::RefLLAUpdate_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace inertial_sense_ros2

namespace inertial_sense_ros2
{

namespace srv
{

struct RefLLAUpdate
{
  using Request = inertial_sense_ros2::srv::RefLLAUpdate_Request;
  using Response = inertial_sense_ros2::srv::RefLLAUpdate_Response;
  using Event = inertial_sense_ros2::srv::RefLLAUpdate_Event;
};

}  // namespace srv

}  // namespace inertial_sense_ros2

#endif  // INERTIAL_SENSE_ROS2__SRV__DETAIL__REF_LLA_UPDATE__STRUCT_HPP_
