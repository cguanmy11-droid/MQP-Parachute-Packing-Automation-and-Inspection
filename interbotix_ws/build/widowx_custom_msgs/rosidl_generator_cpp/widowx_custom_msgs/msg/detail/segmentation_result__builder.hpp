// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from widowx_custom_msgs:msg/SegmentationResult.idl
// generated code does not contain a copyright notice

#ifndef WIDOWX_CUSTOM_MSGS__MSG__DETAIL__SEGMENTATION_RESULT__BUILDER_HPP_
#define WIDOWX_CUSTOM_MSGS__MSG__DETAIL__SEGMENTATION_RESULT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "widowx_custom_msgs/msg/detail/segmentation_result__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace widowx_custom_msgs
{

namespace msg
{

namespace builder
{

class Init_SegmentationResult_camera_info
{
public:
  explicit Init_SegmentationResult_camera_info(::widowx_custom_msgs::msg::SegmentationResult & msg)
  : msg_(msg)
  {}
  ::widowx_custom_msgs::msg::SegmentationResult camera_info(::widowx_custom_msgs::msg::SegmentationResult::_camera_info_type arg)
  {
    msg_.camera_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::widowx_custom_msgs::msg::SegmentationResult msg_;
};

class Init_SegmentationResult_confidence
{
public:
  explicit Init_SegmentationResult_confidence(::widowx_custom_msgs::msg::SegmentationResult & msg)
  : msg_(msg)
  {}
  Init_SegmentationResult_camera_info confidence(::widowx_custom_msgs::msg::SegmentationResult::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_SegmentationResult_camera_info(msg_);
  }

private:
  ::widowx_custom_msgs::msg::SegmentationResult msg_;
};

class Init_SegmentationResult_area
{
public:
  explicit Init_SegmentationResult_area(::widowx_custom_msgs::msg::SegmentationResult & msg)
  : msg_(msg)
  {}
  Init_SegmentationResult_confidence area(::widowx_custom_msgs::msg::SegmentationResult::_area_type arg)
  {
    msg_.area = std::move(arg);
    return Init_SegmentationResult_confidence(msg_);
  }

private:
  ::widowx_custom_msgs::msg::SegmentationResult msg_;
};

class Init_SegmentationResult_corners
{
public:
  explicit Init_SegmentationResult_corners(::widowx_custom_msgs::msg::SegmentationResult & msg)
  : msg_(msg)
  {}
  Init_SegmentationResult_area corners(::widowx_custom_msgs::msg::SegmentationResult::_corners_type arg)
  {
    msg_.corners = std::move(arg);
    return Init_SegmentationResult_area(msg_);
  }

private:
  ::widowx_custom_msgs::msg::SegmentationResult msg_;
};

class Init_SegmentationResult_corner_average
{
public:
  explicit Init_SegmentationResult_corner_average(::widowx_custom_msgs::msg::SegmentationResult & msg)
  : msg_(msg)
  {}
  Init_SegmentationResult_corners corner_average(::widowx_custom_msgs::msg::SegmentationResult::_corner_average_type arg)
  {
    msg_.corner_average = std::move(arg);
    return Init_SegmentationResult_corners(msg_);
  }

private:
  ::widowx_custom_msgs::msg::SegmentationResult msg_;
};

class Init_SegmentationResult_centroid
{
public:
  explicit Init_SegmentationResult_centroid(::widowx_custom_msgs::msg::SegmentationResult & msg)
  : msg_(msg)
  {}
  Init_SegmentationResult_corner_average centroid(::widowx_custom_msgs::msg::SegmentationResult::_centroid_type arg)
  {
    msg_.centroid = std::move(arg);
    return Init_SegmentationResult_corner_average(msg_);
  }

private:
  ::widowx_custom_msgs::msg::SegmentationResult msg_;
};

class Init_SegmentationResult_color_name
{
public:
  explicit Init_SegmentationResult_color_name(::widowx_custom_msgs::msg::SegmentationResult & msg)
  : msg_(msg)
  {}
  Init_SegmentationResult_centroid color_name(::widowx_custom_msgs::msg::SegmentationResult::_color_name_type arg)
  {
    msg_.color_name = std::move(arg);
    return Init_SegmentationResult_centroid(msg_);
  }

private:
  ::widowx_custom_msgs::msg::SegmentationResult msg_;
};

class Init_SegmentationResult_header
{
public:
  Init_SegmentationResult_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SegmentationResult_color_name header(::widowx_custom_msgs::msg::SegmentationResult::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SegmentationResult_color_name(msg_);
  }

private:
  ::widowx_custom_msgs::msg::SegmentationResult msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::widowx_custom_msgs::msg::SegmentationResult>()
{
  return widowx_custom_msgs::msg::builder::Init_SegmentationResult_header();
}

}  // namespace widowx_custom_msgs

#endif  // WIDOWX_CUSTOM_MSGS__MSG__DETAIL__SEGMENTATION_RESULT__BUILDER_HPP_
