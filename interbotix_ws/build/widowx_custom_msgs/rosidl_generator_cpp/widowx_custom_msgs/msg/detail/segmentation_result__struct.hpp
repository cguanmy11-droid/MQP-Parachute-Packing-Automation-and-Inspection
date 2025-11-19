// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from widowx_custom_msgs:msg/SegmentationResult.idl
// generated code does not contain a copyright notice

#ifndef WIDOWX_CUSTOM_MSGS__MSG__DETAIL__SEGMENTATION_RESULT__STRUCT_HPP_
#define WIDOWX_CUSTOM_MSGS__MSG__DETAIL__SEGMENTATION_RESULT__STRUCT_HPP_

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
// Member 'centroid'
// Member 'corner_average'
// Member 'corners'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'camera_info'
#include "sensor_msgs/msg/detail/camera_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__widowx_custom_msgs__msg__SegmentationResult __attribute__((deprecated))
#else
# define DEPRECATED__widowx_custom_msgs__msg__SegmentationResult __declspec(deprecated)
#endif

namespace widowx_custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SegmentationResult_
{
  using Type = SegmentationResult_<ContainerAllocator>;

  explicit SegmentationResult_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    centroid(_init),
    corner_average(_init),
    camera_info(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->color_name = "";
      this->corners.fill(geometry_msgs::msg::Point_<ContainerAllocator>{_init});
      this->area = 0.0f;
      this->confidence = 0.0f;
    }
  }

  explicit SegmentationResult_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    color_name(_alloc),
    centroid(_alloc, _init),
    corner_average(_alloc, _init),
    corners(_alloc),
    camera_info(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->color_name = "";
      this->corners.fill(geometry_msgs::msg::Point_<ContainerAllocator>{_alloc, _init});
      this->area = 0.0f;
      this->confidence = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _color_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _color_name_type color_name;
  using _centroid_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _centroid_type centroid;
  using _corner_average_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _corner_average_type corner_average;
  using _corners_type =
    std::array<geometry_msgs::msg::Point_<ContainerAllocator>, 4>;
  _corners_type corners;
  using _area_type =
    float;
  _area_type area;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _camera_info_type =
    sensor_msgs::msg::CameraInfo_<ContainerAllocator>;
  _camera_info_type camera_info;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__color_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->color_name = _arg;
    return *this;
  }
  Type & set__centroid(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->centroid = _arg;
    return *this;
  }
  Type & set__corner_average(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->corner_average = _arg;
    return *this;
  }
  Type & set__corners(
    const std::array<geometry_msgs::msg::Point_<ContainerAllocator>, 4> & _arg)
  {
    this->corners = _arg;
    return *this;
  }
  Type & set__area(
    const float & _arg)
  {
    this->area = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__camera_info(
    const sensor_msgs::msg::CameraInfo_<ContainerAllocator> & _arg)
  {
    this->camera_info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    widowx_custom_msgs::msg::SegmentationResult_<ContainerAllocator> *;
  using ConstRawPtr =
    const widowx_custom_msgs::msg::SegmentationResult_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<widowx_custom_msgs::msg::SegmentationResult_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<widowx_custom_msgs::msg::SegmentationResult_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      widowx_custom_msgs::msg::SegmentationResult_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<widowx_custom_msgs::msg::SegmentationResult_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      widowx_custom_msgs::msg::SegmentationResult_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<widowx_custom_msgs::msg::SegmentationResult_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<widowx_custom_msgs::msg::SegmentationResult_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<widowx_custom_msgs::msg::SegmentationResult_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__widowx_custom_msgs__msg__SegmentationResult
    std::shared_ptr<widowx_custom_msgs::msg::SegmentationResult_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__widowx_custom_msgs__msg__SegmentationResult
    std::shared_ptr<widowx_custom_msgs::msg::SegmentationResult_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SegmentationResult_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->color_name != other.color_name) {
      return false;
    }
    if (this->centroid != other.centroid) {
      return false;
    }
    if (this->corner_average != other.corner_average) {
      return false;
    }
    if (this->corners != other.corners) {
      return false;
    }
    if (this->area != other.area) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->camera_info != other.camera_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const SegmentationResult_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SegmentationResult_

// alias to use template instance with default allocator
using SegmentationResult =
  widowx_custom_msgs::msg::SegmentationResult_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace widowx_custom_msgs

#endif  // WIDOWX_CUSTOM_MSGS__MSG__DETAIL__SEGMENTATION_RESULT__STRUCT_HPP_
