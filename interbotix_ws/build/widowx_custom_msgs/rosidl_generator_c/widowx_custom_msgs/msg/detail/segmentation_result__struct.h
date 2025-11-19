// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from widowx_custom_msgs:msg/SegmentationResult.idl
// generated code does not contain a copyright notice

#ifndef WIDOWX_CUSTOM_MSGS__MSG__DETAIL__SEGMENTATION_RESULT__STRUCT_H_
#define WIDOWX_CUSTOM_MSGS__MSG__DETAIL__SEGMENTATION_RESULT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'color_name'
#include "rosidl_runtime_c/string.h"
// Member 'centroid'
// Member 'corner_average'
// Member 'corners'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'camera_info'
#include "sensor_msgs/msg/detail/camera_info__struct.h"

/// Struct defined in msg/SegmentationResult in the package widowx_custom_msgs.
/**
  * 颜色分割结果消息
 */
typedef struct widowx_custom_msgs__msg__SegmentationResult
{
  std_msgs__msg__Header header;
  /// 检测到的颜色名称
  rosidl_runtime_c__String color_name;
  /// 质心坐标 (像素坐标系)
  geometry_msgs__msg__Point centroid;
  /// 四个角点的平均位置 (像素坐标系)
  geometry_msgs__msg__Point corner_average;
  /// 四个角点 (按顺序: top-left, top-right, bottom-right, bottom-left)
  geometry_msgs__msg__Point corners[4];
  /// 区域面积 (像素)
  float area;
  /// 置信度 (0.0-1.0)
  float confidence;
  /// 相机信息
  sensor_msgs__msg__CameraInfo camera_info;
} widowx_custom_msgs__msg__SegmentationResult;

// Struct for a sequence of widowx_custom_msgs__msg__SegmentationResult.
typedef struct widowx_custom_msgs__msg__SegmentationResult__Sequence
{
  widowx_custom_msgs__msg__SegmentationResult * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} widowx_custom_msgs__msg__SegmentationResult__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // WIDOWX_CUSTOM_MSGS__MSG__DETAIL__SEGMENTATION_RESULT__STRUCT_H_
