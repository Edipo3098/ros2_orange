// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/Mpu.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__MPU__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__MPU__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__Mpu __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__Mpu __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Mpu_
{
  using Type = Mpu_<ContainerAllocator>;

  explicit Mpu_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message = "";
      this->acx = 0ll;
      this->acy = 0ll;
      this->acz = 0ll;
      this->gx = 0ll;
      this->gy = 0ll;
      this->gz = 0ll;
    }
  }

  explicit Mpu_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message = "";
      this->acx = 0ll;
      this->acy = 0ll;
      this->acz = 0ll;
      this->gx = 0ll;
      this->gy = 0ll;
      this->gz = 0ll;
    }
  }

  // field types and members
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _acx_type =
    int64_t;
  _acx_type acx;
  using _acy_type =
    int64_t;
  _acy_type acy;
  using _acz_type =
    int64_t;
  _acz_type acz;
  using _gx_type =
    int64_t;
  _gx_type gx;
  using _gy_type =
    int64_t;
  _gy_type gy;
  using _gz_type =
    int64_t;
  _gz_type gz;

  // setters for named parameter idiom
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__acx(
    const int64_t & _arg)
  {
    this->acx = _arg;
    return *this;
  }
  Type & set__acy(
    const int64_t & _arg)
  {
    this->acy = _arg;
    return *this;
  }
  Type & set__acz(
    const int64_t & _arg)
  {
    this->acz = _arg;
    return *this;
  }
  Type & set__gx(
    const int64_t & _arg)
  {
    this->gx = _arg;
    return *this;
  }
  Type & set__gy(
    const int64_t & _arg)
  {
    this->gy = _arg;
    return *this;
  }
  Type & set__gz(
    const int64_t & _arg)
  {
    this->gz = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::msg::Mpu_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::Mpu_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::Mpu_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::Mpu_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::Mpu_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::Mpu_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::Mpu_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::Mpu_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::Mpu_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::Mpu_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__Mpu
    std::shared_ptr<robot_interfaces::msg::Mpu_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__Mpu
    std::shared_ptr<robot_interfaces::msg::Mpu_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Mpu_ & other) const
  {
    if (this->message != other.message) {
      return false;
    }
    if (this->acx != other.acx) {
      return false;
    }
    if (this->acy != other.acy) {
      return false;
    }
    if (this->acz != other.acz) {
      return false;
    }
    if (this->gx != other.gx) {
      return false;
    }
    if (this->gy != other.gy) {
      return false;
    }
    if (this->gz != other.gz) {
      return false;
    }
    return true;
  }
  bool operator!=(const Mpu_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Mpu_

// alias to use template instance with default allocator
using Mpu =
  robot_interfaces::msg::Mpu_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__MPU__STRUCT_HPP_
