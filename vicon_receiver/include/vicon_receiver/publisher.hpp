// Original work Copyright (c) 2024 OPT4SMART (Andrea Testa)
// Modified work Copyright (c) 2026 prudolph
//
// This file is part of vicon_odometry_estimator (GPL-3.0).

#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "vicon_receiver/lowpass_filter.hpp"

// Per-segment publisher that turns successive Vicon pose samples into a
// nav_msgs/Odometry message: pose is reported as-is, twist is computed from
// finite-differenced position and quaternion derivative, then run through a
// 2nd-order Butterworth low-pass filter that copes with non-uniform dt.
//
// When debug mode is enabled an additional TwistStamped is published with
// the *unfiltered* velocity, so the user can compare phase-lag vs noise
// suppression in rqt.
class Publisher
{
public:
    bool is_ready = false;

    Publisher(const std::string & topic_name,
              const std::string & frame_id,
              const std::string & child_frame_id,
              double cutoff_hz,
              bool debug,
              rclcpp::Node * node);

    // Publish an Odometry message built from the new pose sample. The pose
    // must already be expressed in `frame_id` (typically the world / map
    // frame); the resulting twist is expressed in `child_frame_id` (body
    // frame) per the nav_msgs/Odometry convention.
    void publish(const geometry_msgs::msg::PoseStamped & pose);

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr raw_twist_pub_;

    std::string frame_id_;
    std::string child_frame_id_;
    bool debug_;

    // State persistence for differentiation.
    bool   has_prev_ = false;
    rclcpp::Time last_time_;
    geometry_msgs::msg::Pose last_pose_;

    // Filters (linear velocity and body-frame angular velocity).
    LowPassFilter2ndOrder3D filter_v_;
    LowPassFilter2ndOrder3D filter_w_;
};

#endif  // PUBLISHER_HPP
