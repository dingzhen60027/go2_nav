#pragma once

#include <algorithm>
#include <cmath>

namespace fast_icp_loc {

struct IcpGateLimits {
  double max_translation_delta{0.45};
  double max_yaw_delta{0.60};
  double max_fitness_score{1.0};
};

struct IcpGateDecision {
  bool update_pose{false};
  bool publish_pose{false};
  bool just_locked{false};
  bool restarted_confirmation{false};
};

class IcpAcceptanceGate {
public:
  void configure(const IcpGateLimits &tracking_limits,
                 const IcpGateLimits &initial_limits,
                 int required_confirmations) {
    tracking_limits_ = tracking_limits;
    initial_limits_ = initial_limits;
    confirmation_limits_ = tracking_limits;
    confirmation_limits_.max_fitness_score = std::min(
        tracking_limits.max_fitness_score, initial_limits.max_fitness_score);
    required_confirmations_ = std::max(1, required_confirmations);
    reset();
  }

  void reset() {
    tracking_locked_ = false;
    confirmation_count_ = 0;
  }

  IcpGateDecision evaluate(double translation_delta, double yaw_delta,
                           double fitness_score) {
    if (tracking_locked_) {
      const bool accepted = passes(tracking_limits_, translation_delta,
                                   yaw_delta, fitness_score);
      return {accepted, accepted, false, false};
    }

    if (confirmation_count_ == 0) {
      if (!passes(initial_limits_, translation_delta, yaw_delta, fitness_score)) {
        return {};
      }
      confirmation_count_ = 1;
      return acceptAcquisitionCandidate(false);
    }

    if (passes(confirmation_limits_, translation_delta, yaw_delta,
               fitness_score)) {
      ++confirmation_count_;
      return acceptAcquisitionCandidate(false);
    }

    if (passes(initial_limits_, translation_delta, yaw_delta, fitness_score)) {
      confirmation_count_ = 1;
      return acceptAcquisitionCandidate(true);
    }

    confirmation_count_ = 0;
    return {};
  }

  bool trackingLocked() const { return tracking_locked_; }
  int confirmationCount() const { return confirmation_count_; }
  int requiredConfirmations() const { return required_confirmations_; }

private:
  static bool passes(const IcpGateLimits &limits, double translation_delta,
                     double yaw_delta, double fitness_score) {
    return std::isfinite(translation_delta) && std::isfinite(yaw_delta) &&
           std::isfinite(fitness_score) && translation_delta >= 0.0 &&
           yaw_delta >= 0.0 && fitness_score >= 0.0 &&
           translation_delta <= limits.max_translation_delta &&
           yaw_delta <= limits.max_yaw_delta &&
           fitness_score <= limits.max_fitness_score;
  }

  IcpGateDecision acceptAcquisitionCandidate(bool restarted_confirmation) {
    if (confirmation_count_ >= required_confirmations_) {
      tracking_locked_ = true;
      return {true, true, true, restarted_confirmation};
    }
    return {true, false, false, restarted_confirmation};
  }

  IcpGateLimits tracking_limits_;
  IcpGateLimits initial_limits_;
  IcpGateLimits confirmation_limits_;
  int required_confirmations_{3};
  int confirmation_count_{0};
  bool tracking_locked_{false};
};

}  // namespace fast_icp_loc
