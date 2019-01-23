/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <custom_msgs/LegArray.h>
#include "kf_tracker/utilities/kalman.h"

namespace kf_tracker
{

class TrackedLeg {
public:
  TrackedLeg(const custom_msgs::Leg& leg) : leg_(leg), kf_x_(0, 1, 2), kf_y_(0, 1, 2) {
    fade_counter_ = s_fade_counter_size_;
    initKF();
  }

  void predictState() {
    kf_x_.predictState();
    kf_y_.predictState();

    leg_.position.x = kf_x_.q_pred(0);
    leg_.position.y = kf_y_.q_pred(0);

    leg_.velocity.x = kf_x_.q_pred(1);
    leg_.velocity.y = kf_y_.q_pred(1);

    fade_counter_--;
  }

  void correctState(const custom_msgs::Leg& new_leg) {
    kf_x_.y(0) = new_leg.position.x;
    kf_y_.y(0) = new_leg.position.y;

    kf_x_.correctState();
    kf_y_.correctState();

    leg_.position.x = kf_x_.q_est(0);
    leg_.position.y = kf_y_.q_est(0);

    leg_.velocity.x = kf_x_.q_est(1);
    leg_.velocity.y = kf_y_.q_est(1);

    fade_counter_ = s_fade_counter_size_;
  }

  void updateState() {
    kf_x_.predictState();
    kf_y_.predictState();

    kf_x_.correctState();
    kf_y_.correctState();

    leg_.position.x = kf_x_.q_est(0);
    leg_.position.y = kf_y_.q_est(0);

    leg_.velocity.x = kf_x_.q_est(1);
    leg_.velocity.y = kf_y_.q_est(1);

    fade_counter_--;
  }

  static void setSamplingTime(double tp) {
    s_sampling_time_ = tp;
  }

  static void setCounterSize(int size) {
    s_fade_counter_size_ = size;
  }

  static void setCovariances(double process_var, double process_rate_var, double measurement_var) {
    s_process_variance_ = process_var;
    s_process_rate_variance_ = process_rate_var;
    s_measurement_variance_ = measurement_var;
  }

  bool hasFaded() const { return ((fade_counter_ <= 0) ? true : false); }
  const custom_msgs::Leg& getLeg() const { return leg_; }
  const KalmanFilter& getKFx() const { return kf_x_; }
  const KalmanFilter& getKFy() const { return kf_y_; }

private:
  void initKF() {
    kf_x_.A(0, 1) = s_sampling_time_;
    kf_y_.A(0, 1) = s_sampling_time_;

    kf_x_.C(0, 0) = 1.0;
    kf_y_.C(0, 0) = 1.0;

    kf_x_.R(0, 0) = s_measurement_variance_;
    kf_y_.R(0, 0) = s_measurement_variance_;

    kf_x_.Q(0, 0) = s_process_variance_;
    kf_y_.Q(0, 0) = s_process_variance_;

    kf_x_.Q(1, 1) = s_process_rate_variance_;
    kf_y_.Q(1, 1) = s_process_rate_variance_;

    kf_x_.q_pred(0) = leg_.position.x;
    kf_y_.q_pred(0) = leg_.position.y;

    kf_x_.q_pred(1) = leg_.velocity.x;
    kf_y_.q_pred(1) = leg_.velocity.y;

    kf_x_.q_est(0) = leg_.position.x;
    kf_y_.q_est(0) = leg_.position.y;

    kf_x_.q_est(1) = leg_.velocity.x;
    kf_y_.q_est(1) = leg_.velocity.y;
  }

  custom_msgs::Leg leg_;

  KalmanFilter kf_x_;
  KalmanFilter kf_y_;

  int fade_counter_;

  // Common variables
  static int s_fade_counter_size_;
  static double s_sampling_time_;
  static double s_process_variance_;
  static double s_process_rate_variance_;
  static double s_measurement_variance_;
  };

}
