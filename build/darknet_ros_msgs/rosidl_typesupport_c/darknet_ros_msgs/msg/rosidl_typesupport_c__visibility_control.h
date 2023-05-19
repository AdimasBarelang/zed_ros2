// generated from
// rosidl_typesupport_c/resource/rosidl_typesupport_c__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef DARKNET_ROS_MSGS__MSG__ROSIDL_TYPESUPPORT_C__VISIBILITY_CONTROL_H_
#define DARKNET_ROS_MSGS__MSG__ROSIDL_TYPESUPPORT_C__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_C_EXPORT_darknet_ros_msgs __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_C_IMPORT_darknet_ros_msgs __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_C_EXPORT_darknet_ros_msgs __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_C_IMPORT_darknet_ros_msgs __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_C_BUILDING_DLL_darknet_ros_msgs
    #define ROSIDL_TYPESUPPORT_C_PUBLIC_darknet_ros_msgs ROSIDL_TYPESUPPORT_C_EXPORT_darknet_ros_msgs
  #else
    #define ROSIDL_TYPESUPPORT_C_PUBLIC_darknet_ros_msgs ROSIDL_TYPESUPPORT_C_IMPORT_darknet_ros_msgs
  #endif
#else
  #define ROSIDL_TYPESUPPORT_C_EXPORT_darknet_ros_msgs __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_C_IMPORT_darknet_ros_msgs
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_C_PUBLIC_darknet_ros_msgs __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_C_PUBLIC_darknet_ros_msgs
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // DARKNET_ROS_MSGS__MSG__ROSIDL_TYPESUPPORT_C__VISIBILITY_CONTROL_H_
