// Original work Copyright (c) 2024 OPT4SMART (Andrea Testa)
// Modified work Copyright (c) 2026 prudolph
//
// This file is part of vicon_odometry_estimator (GPL-3.0).

#include "vicon_receiver/publisher.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

Publisher::Publisher(const std::string & topic_name,
                     const std::string & frame_id,
                     const std::string & child_frame_id,
                     double cutoff_hz,
                     bool debug,
                     rclcpp::Node * node)
: frame_id_(frame_id),
  child_frame_id_(child_frame_id),
  debug_(debug),
  filter_v_(cutoff_hz),
  filter_w_(cutoff_hz)
{
    odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10);
    if (debug_) {
        raw_twist_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(
            topic_name + "/twist_raw", 10);
    }
    is_ready = true;
}

void Publisher::publish(const geometry_msgs::msg::PoseStamped & pose)
{
    nav_msgs::msg::Odometry odom;
    odom.header = pose.header;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id  = child_frame_id_;
    odom.pose.pose = pose.pose;

    const rclcpp::Time t_now(pose.header.stamp);

    if (!has_prev_) {
        last_pose_ = pose.pose;
        last_time_ = t_now;
        has_prev_ = true;
        // No velocity estimate yet; publish zero twist for the very first
        // sample so downstream consumers still get a complete Odometry.
        odom_pub_->publish(odom);
        return;
    }

    const double dt = (t_now - last_time_).seconds();
    if (dt <= 0.0) {
        // Duplicate or out-of-order sample; just republish pose without
        // contaminating the differentiator.
        odom_pub_->publish(odom);
        return;
    }

    // ---- Linear velocity (world frame) ----
    const double vx_w = (pose.pose.position.x - last_pose_.position.x) / dt;
    const double vy_w = (pose.pose.position.y - last_pose_.position.y) / dt;
    const double vz_w = (pose.pose.position.z - last_pose_.position.z) / dt;

    // ---- Angular velocity from quaternion derivative ----
    // q_dot = (q_new - q_prev) / dt
    // omega_body = 2 * (q_prev^-1 * q_dot)  (vector part)
    tf2::Quaternion q_prev(last_pose_.orientation.x,
                           last_pose_.orientation.y,
                           last_pose_.orientation.z,
                           last_pose_.orientation.w);
    tf2::Quaternion q_new(pose.pose.orientation.x,
                          pose.pose.orientation.y,
                          pose.pose.orientation.z,
                          pose.pose.orientation.w);

    // Ensure shortest-path: flip sign if dot < 0 to avoid 2*pi jumps.
    if (q_prev.dot(q_new) < 0.0) {
        q_new = tf2::Quaternion(-q_new.x(), -q_new.y(), -q_new.z(), -q_new.w());
    }

    tf2::Quaternion q_dot(
        (q_new.x() - q_prev.x()) / dt,
        (q_new.y() - q_prev.y()) / dt,
        (q_new.z() - q_prev.z()) / dt,
        (q_new.w() - q_prev.w()) / dt);

    const tf2::Quaternion q_prev_inv = q_prev.inverse();
    const tf2::Quaternion omega_q = q_prev_inv * q_dot;

    const double wx_b = 2.0 * omega_q.x();
    const double wy_b = 2.0 * omega_q.y();
    const double wz_b = 2.0 * omega_q.z();

    // ---- Linear velocity in body frame (Odometry twist convention) ----
    // v_body = q_prev^-1 * v_world * q_prev
    tf2::Quaternion v_w_q(vx_w, vy_w, vz_w, 0.0);
    tf2::Quaternion v_b_q = q_prev_inv * v_w_q * q_prev;
    const double vx_b = v_b_q.x();
    const double vy_b = v_b_q.y();
    const double vz_b = v_b_q.z();

    // ---- Filter ----
    const auto v_f = filter_v_.apply({vx_b, vy_b, vz_b}, dt);
    const auto w_f = filter_w_.apply({wx_b, wy_b, wz_b}, dt);

    odom.twist.twist.linear.x  = v_f[0];
    odom.twist.twist.linear.y  = v_f[1];
    odom.twist.twist.linear.z  = v_f[2];
    odom.twist.twist.angular.x = w_f[0];
    odom.twist.twist.angular.y = w_f[1];
    odom.twist.twist.angular.z = w_f[2];

    odom_pub_->publish(odom);

    if (debug_ && raw_twist_pub_) {
        geometry_msgs::msg::TwistStamped raw;
        raw.header = odom.header;
        raw.twist.linear.x  = vx_b;
        raw.twist.linear.y  = vy_b;
        raw.twist.linear.z  = vz_b;
        raw.twist.angular.x = wx_b;
        raw.twist.angular.y = wy_b;
        raw.twist.angular.z = wz_b;
        raw_twist_pub_->publish(raw);
    }

    last_pose_ = pose.pose;
    last_time_ = t_now;
}
