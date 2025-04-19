// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from wiimote_msgs:msg/IrSourceInfo.idl
// generated code does not contain a copyright notice
#ifndef WIIMOTE_MSGS__MSG__DETAIL__IR_SOURCE_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define WIIMOTE_MSGS__MSG__DETAIL__IR_SOURCE_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "wiimote_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "wiimote_msgs/msg/detail/ir_source_info__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wiimote_msgs
bool cdr_serialize_wiimote_msgs__msg__IrSourceInfo(
  const wiimote_msgs__msg__IrSourceInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wiimote_msgs
bool cdr_deserialize_wiimote_msgs__msg__IrSourceInfo(
  eprosima::fastcdr::Cdr &,
  wiimote_msgs__msg__IrSourceInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wiimote_msgs
size_t get_serialized_size_wiimote_msgs__msg__IrSourceInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wiimote_msgs
size_t max_serialized_size_wiimote_msgs__msg__IrSourceInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wiimote_msgs
bool cdr_serialize_key_wiimote_msgs__msg__IrSourceInfo(
  const wiimote_msgs__msg__IrSourceInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wiimote_msgs
size_t get_serialized_size_key_wiimote_msgs__msg__IrSourceInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wiimote_msgs
size_t max_serialized_size_key_wiimote_msgs__msg__IrSourceInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_wiimote_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, wiimote_msgs, msg, IrSourceInfo)();

#ifdef __cplusplus
}
#endif

#endif  // WIIMOTE_MSGS__MSG__DETAIL__IR_SOURCE_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
