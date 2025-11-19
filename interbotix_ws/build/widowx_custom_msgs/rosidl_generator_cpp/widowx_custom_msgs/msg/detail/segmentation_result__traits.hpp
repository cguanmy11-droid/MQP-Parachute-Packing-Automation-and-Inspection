// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from widowx_custom_msgs:msg/SegmentationResult.idl
// generated code does not contain a copyright notice

#ifndef WIDOWX_CUSTOM_MSGS__MSG__DETAIL__SEGMENTATION_RESULT__TRAITS_HPP_
#define WIDOWX_CUSTOM_MSGS__MSG__DETAIL__SEGMENTATION_RESULT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "widowx_custom_msgs/msg/detail/segmentation_result__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'centroid'
// Member 'corner_average'
// Member 'corners'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'camera_info'
#include "sensor_msgs/msg/detail/camera_info__traits.hpp"

namespace widowx_custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SegmentationResult & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: color_name
  {
    out << "color_name: ";
    rosidl_generator_traits::value_to_yaml(msg.color_name, out);
    out << ", ";
  }

  // member: centroid
  {
    out << "centroid: ";
    to_flow_style_yaml(msg.centroid, out);
    out << ", ";
  }

  // member: corner_average
  {
    out << "corner_average: ";
    to_flow_style_yaml(msg.corner_average, out);
    out << ", ";
  }

  // member: corners
  {
    if (msg.corners.size() == 0) {
      out << "corners: []";
    } else {
      out << "corners: [";
      size_t pending_items = msg.corners.size();
      for (auto item : msg.corners) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: area
  {
    out << "area: ";
    rosidl_generator_traits::value_to_yaml(msg.area, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: camera_info
  {
    out << "camera_info: ";
    to_flow_style_yaml(msg.camera_info, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SegmentationResult & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: color_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color_name: ";
    rosidl_generator_traits::value_to_yaml(msg.color_name, out);
    out << "\n";
  }

  // member: centroid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "centroid:\n";
    to_block_style_yaml(msg.centroid, out, indentation + 2);
  }

  // member: corner_average
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "corner_average:\n";
    to_block_style_yaml(msg.corner_average, out, indentation + 2);
  }

  // member: corners
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.corners.size() == 0) {
      out << "corners: []\n";
    } else {
      out << "corners:\n";
      for (auto item : msg.corners) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: area
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "area: ";
    rosidl_generator_traits::value_to_yaml(msg.area, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: camera_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "camera_info:\n";
    to_block_style_yaml(msg.camera_info, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SegmentationResult & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace widowx_custom_msgs

namespace rosidl_generator_traits
{

[[deprecated("use widowx_custom_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const widowx_custom_msgs::msg::SegmentationResult & msg,
  std::ostream & out, size_t indentation = 0)
{
  widowx_custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use widowx_custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const widowx_custom_msgs::msg::SegmentationResult & msg)
{
  return widowx_custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<widowx_custom_msgs::msg::SegmentationResult>()
{
  return "widowx_custom_msgs::msg::SegmentationResult";
}

template<>
inline const char * name<widowx_custom_msgs::msg::SegmentationResult>()
{
  return "widowx_custom_msgs/msg/SegmentationResult";
}

template<>
struct has_fixed_size<widowx_custom_msgs::msg::SegmentationResult>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<widowx_custom_msgs::msg::SegmentationResult>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<widowx_custom_msgs::msg::SegmentationResult>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // WIDOWX_CUSTOM_MSGS__MSG__DETAIL__SEGMENTATION_RESULT__TRAITS_HPP_
