// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from widowx_custom_msgs:msg/SegmentationResult.idl
// generated code does not contain a copyright notice
#include "widowx_custom_msgs/msg/detail/segmentation_result__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `color_name`
#include "rosidl_runtime_c/string_functions.h"
// Member `centroid`
// Member `corner_average`
// Member `corners`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `camera_info`
#include "sensor_msgs/msg/detail/camera_info__functions.h"

bool
widowx_custom_msgs__msg__SegmentationResult__init(widowx_custom_msgs__msg__SegmentationResult * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    widowx_custom_msgs__msg__SegmentationResult__fini(msg);
    return false;
  }
  // color_name
  if (!rosidl_runtime_c__String__init(&msg->color_name)) {
    widowx_custom_msgs__msg__SegmentationResult__fini(msg);
    return false;
  }
  // centroid
  if (!geometry_msgs__msg__Point__init(&msg->centroid)) {
    widowx_custom_msgs__msg__SegmentationResult__fini(msg);
    return false;
  }
  // corner_average
  if (!geometry_msgs__msg__Point__init(&msg->corner_average)) {
    widowx_custom_msgs__msg__SegmentationResult__fini(msg);
    return false;
  }
  // corners
  for (size_t i = 0; i < 4; ++i) {
    if (!geometry_msgs__msg__Point__init(&msg->corners[i])) {
      widowx_custom_msgs__msg__SegmentationResult__fini(msg);
      return false;
    }
  }
  // area
  // confidence
  // camera_info
  if (!sensor_msgs__msg__CameraInfo__init(&msg->camera_info)) {
    widowx_custom_msgs__msg__SegmentationResult__fini(msg);
    return false;
  }
  return true;
}

void
widowx_custom_msgs__msg__SegmentationResult__fini(widowx_custom_msgs__msg__SegmentationResult * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // color_name
  rosidl_runtime_c__String__fini(&msg->color_name);
  // centroid
  geometry_msgs__msg__Point__fini(&msg->centroid);
  // corner_average
  geometry_msgs__msg__Point__fini(&msg->corner_average);
  // corners
  for (size_t i = 0; i < 4; ++i) {
    geometry_msgs__msg__Point__fini(&msg->corners[i]);
  }
  // area
  // confidence
  // camera_info
  sensor_msgs__msg__CameraInfo__fini(&msg->camera_info);
}

bool
widowx_custom_msgs__msg__SegmentationResult__are_equal(const widowx_custom_msgs__msg__SegmentationResult * lhs, const widowx_custom_msgs__msg__SegmentationResult * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // color_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->color_name), &(rhs->color_name)))
  {
    return false;
  }
  // centroid
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->centroid), &(rhs->centroid)))
  {
    return false;
  }
  // corner_average
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->corner_average), &(rhs->corner_average)))
  {
    return false;
  }
  // corners
  for (size_t i = 0; i < 4; ++i) {
    if (!geometry_msgs__msg__Point__are_equal(
        &(lhs->corners[i]), &(rhs->corners[i])))
    {
      return false;
    }
  }
  // area
  if (lhs->area != rhs->area) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // camera_info
  if (!sensor_msgs__msg__CameraInfo__are_equal(
      &(lhs->camera_info), &(rhs->camera_info)))
  {
    return false;
  }
  return true;
}

bool
widowx_custom_msgs__msg__SegmentationResult__copy(
  const widowx_custom_msgs__msg__SegmentationResult * input,
  widowx_custom_msgs__msg__SegmentationResult * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // color_name
  if (!rosidl_runtime_c__String__copy(
      &(input->color_name), &(output->color_name)))
  {
    return false;
  }
  // centroid
  if (!geometry_msgs__msg__Point__copy(
      &(input->centroid), &(output->centroid)))
  {
    return false;
  }
  // corner_average
  if (!geometry_msgs__msg__Point__copy(
      &(input->corner_average), &(output->corner_average)))
  {
    return false;
  }
  // corners
  for (size_t i = 0; i < 4; ++i) {
    if (!geometry_msgs__msg__Point__copy(
        &(input->corners[i]), &(output->corners[i])))
    {
      return false;
    }
  }
  // area
  output->area = input->area;
  // confidence
  output->confidence = input->confidence;
  // camera_info
  if (!sensor_msgs__msg__CameraInfo__copy(
      &(input->camera_info), &(output->camera_info)))
  {
    return false;
  }
  return true;
}

widowx_custom_msgs__msg__SegmentationResult *
widowx_custom_msgs__msg__SegmentationResult__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  widowx_custom_msgs__msg__SegmentationResult * msg = (widowx_custom_msgs__msg__SegmentationResult *)allocator.allocate(sizeof(widowx_custom_msgs__msg__SegmentationResult), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(widowx_custom_msgs__msg__SegmentationResult));
  bool success = widowx_custom_msgs__msg__SegmentationResult__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
widowx_custom_msgs__msg__SegmentationResult__destroy(widowx_custom_msgs__msg__SegmentationResult * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    widowx_custom_msgs__msg__SegmentationResult__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
widowx_custom_msgs__msg__SegmentationResult__Sequence__init(widowx_custom_msgs__msg__SegmentationResult__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  widowx_custom_msgs__msg__SegmentationResult * data = NULL;

  if (size) {
    data = (widowx_custom_msgs__msg__SegmentationResult *)allocator.zero_allocate(size, sizeof(widowx_custom_msgs__msg__SegmentationResult), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = widowx_custom_msgs__msg__SegmentationResult__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        widowx_custom_msgs__msg__SegmentationResult__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
widowx_custom_msgs__msg__SegmentationResult__Sequence__fini(widowx_custom_msgs__msg__SegmentationResult__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      widowx_custom_msgs__msg__SegmentationResult__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

widowx_custom_msgs__msg__SegmentationResult__Sequence *
widowx_custom_msgs__msg__SegmentationResult__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  widowx_custom_msgs__msg__SegmentationResult__Sequence * array = (widowx_custom_msgs__msg__SegmentationResult__Sequence *)allocator.allocate(sizeof(widowx_custom_msgs__msg__SegmentationResult__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = widowx_custom_msgs__msg__SegmentationResult__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
widowx_custom_msgs__msg__SegmentationResult__Sequence__destroy(widowx_custom_msgs__msg__SegmentationResult__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    widowx_custom_msgs__msg__SegmentationResult__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
widowx_custom_msgs__msg__SegmentationResult__Sequence__are_equal(const widowx_custom_msgs__msg__SegmentationResult__Sequence * lhs, const widowx_custom_msgs__msg__SegmentationResult__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!widowx_custom_msgs__msg__SegmentationResult__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
widowx_custom_msgs__msg__SegmentationResult__Sequence__copy(
  const widowx_custom_msgs__msg__SegmentationResult__Sequence * input,
  widowx_custom_msgs__msg__SegmentationResult__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(widowx_custom_msgs__msg__SegmentationResult);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    widowx_custom_msgs__msg__SegmentationResult * data =
      (widowx_custom_msgs__msg__SegmentationResult *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!widowx_custom_msgs__msg__SegmentationResult__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          widowx_custom_msgs__msg__SegmentationResult__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!widowx_custom_msgs__msg__SegmentationResult__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
