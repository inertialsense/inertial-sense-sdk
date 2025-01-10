/**
 * @file ros_compat.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 10/6/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef EVALTOOL_ROS_COMPAT_H
#define EVALTOOL_ROS_COMPAT_H

#if defined(ROS2)
    #include "rclcpp/rclcpp/rclcpp.hpp"

    #define ROS_ERROR(msg) RCLCPP_ERROR(rclcpp::get_logger("Inertial_Sense_ROS"), msg)
    #define ROS_ERROR_STREAM(msg) RCLCPP_ERROR_STREAM(rclcpp::get_logger("Inertial_Sense_ROS"), msg)
    #define ROS_FATAL(msg) RCLCPP_FATAL(rclcpp::get_logger("Inertial_Sense_ROS"), msg)
    #define ROS_WARN(msg) RCLCPP_WARN(rclcpp::get_logger("Inertial_Sense_ROS"), msg)
    #define ROS_WARN_STREAM(msg) RCLCPP_WARN_STREAM(rclcpp::get_logger("Inertial_Sense_ROS"), msg)
    #define ROS_INFO_STREAM(msg) RCLCPP_INFO_STREAM(rclcpp::get_logger("Inertial_Sense_ROS"), msg)

    #define ROS_NODE_HANDLE         rclcpp::Node::SharedPtr
    #define ROS_TIMER               rclcpp::TimerBase::SharedPtr    
    #define ROS1_TIMEREVENT_ARG

    namespace ros_common = rclcpp;
#elif defined(ROS1)
    #include "ros/ros.h"
    
    #define ROS_NODE_HANDLE         ros::NodeHandle*
    #define ROS_TIMER               ros::Timer
    #define ROS1_TIMEREVENT_ARG     const ros::TimerEvent &timer_event

    namespace ros_common = ros;
#endif

#endif //EVALTOOL_ROS_COMPAT_H
