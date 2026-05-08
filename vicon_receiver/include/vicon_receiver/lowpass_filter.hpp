// Modified work Copyright (c) 2026 prudolph
//
// This file is part of vicon_odometry_estimator, derived from the original
// OPT4SMART/ros2-vicon-receiver project (GPL-3.0).  See LICENSE.
//
// 2nd-order Butterworth low-pass filter (Direct-Form-I biquad) that
// re-derives its coefficients on every sample so it is robust to the
// non-uniform delta-t typical of Vicon UDP packet streams.

#ifndef VICON_RECEIVER_LOWPASS_FILTER_HPP
#define VICON_RECEIVER_LOWPASS_FILTER_HPP

#include <array>
#include <cmath>

class LowPassFilter2ndOrder
{
public:
    // cutoff_hz <= 0 disables filtering (pass-through).
    explicit LowPassFilter2ndOrder(double cutoff_hz = 0.0)
        : cutoff_hz_(cutoff_hz), initialised_(false),
          x1_(0.0), x2_(0.0), y1_(0.0), y2_(0.0) {}

    void set_cutoff(double cutoff_hz) { cutoff_hz_ = cutoff_hz; }

    void reset()
    {
        initialised_ = false;
        x1_ = x2_ = y1_ = y2_ = 0.0;
    }

    // Apply the filter to a single scalar sample with the actual time-step
    // dt that elapsed since the previous sample.  Coefficients are recomputed
    // each call via a bilinear (Tustin) discretisation with frequency
    // pre-warping so the magnitude response stays close to the analogue
    // Butterworth prototype regardless of dt jitter.
    double apply(double x, double dt)
    {
        if (cutoff_hz_ <= 0.0 || dt <= 0.0) {
            return x;
        }

        if (!initialised_) {
            x1_ = x2_ = x;
            y1_ = y2_ = x;
            initialised_ = true;
            return x;
        }

        const double wn   = 2.0 * M_PI * cutoff_hz_;
        const double K    = std::tan(wn * dt / 2.0);   // pre-warped
        const double K2   = K * K;
        const double sqrt2 = std::sqrt(2.0);
        const double norm = 1.0 / (1.0 + sqrt2 * K + K2);

        const double b0 = K2 * norm;
        const double b1 = 2.0 * b0;
        const double b2 = b0;
        const double a1 = 2.0 * (K2 - 1.0) * norm;
        const double a2 = (1.0 - sqrt2 * K + K2) * norm;

        const double y = b0 * x + b1 * x1_ + b2 * x2_ - a1 * y1_ - a2 * y2_;

        x2_ = x1_;
        x1_ = x;
        y2_ = y1_;
        y1_ = y;
        return y;
    }

private:
    double cutoff_hz_;
    bool   initialised_;
    double x1_, x2_, y1_, y2_;
};

// Convenience 3-axis wrapper.
class LowPassFilter2ndOrder3D
{
public:
    explicit LowPassFilter2ndOrder3D(double cutoff_hz = 0.0)
        : f_{LowPassFilter2ndOrder(cutoff_hz),
             LowPassFilter2ndOrder(cutoff_hz),
             LowPassFilter2ndOrder(cutoff_hz)} {}

    void set_cutoff(double cutoff_hz)
    {
        for (auto & f : f_) f.set_cutoff(cutoff_hz);
    }

    void reset()
    {
        for (auto & f : f_) f.reset();
    }

    std::array<double, 3> apply(const std::array<double, 3> & x, double dt)
    {
        return { f_[0].apply(x[0], dt),
                 f_[1].apply(x[1], dt),
                 f_[2].apply(x[2], dt) };
    }

private:
    std::array<LowPassFilter2ndOrder, 3> f_;
};

#endif  // VICON_RECEIVER_LOWPASS_FILTER_HPP
