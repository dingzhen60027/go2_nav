#include "fast_icp_loc/icp_acceptance_gate.hpp"

#include <gtest/gtest.h>
#include <limits>

namespace fast_icp_loc {
namespace {

IcpAcceptanceGate makeGate(int confirmations = 3) {
  IcpAcceptanceGate gate;
  gate.configure({0.45, 0.60, 1.0}, {2.5, 1.57, 1.0}, confirmations);
  return gate;
}

TEST(IcpAcceptanceGateTest, InitialCorrectionCanExceedTrackingGate) {
  auto gate = makeGate();

  const auto decision = gate.evaluate(1.20, 0.80, 0.40);

  EXPECT_TRUE(decision.update_pose);
  EXPECT_FALSE(decision.publish_pose);
  EXPECT_FALSE(gate.trackingLocked());
  EXPECT_EQ(gate.confirmationCount(), 1);
}

TEST(IcpAcceptanceGateTest, LocksOnlyAfterStableConfirmations) {
  auto gate = makeGate();

  EXPECT_TRUE(gate.evaluate(1.20, 0.80, 0.40).update_pose);
  EXPECT_FALSE(gate.evaluate(0.08, 0.04, 0.30).publish_pose);
  const auto locked = gate.evaluate(0.05, 0.03, 0.25);

  EXPECT_TRUE(locked.update_pose);
  EXPECT_TRUE(locked.publish_pose);
  EXPECT_TRUE(locked.just_locked);
  EXPECT_TRUE(gate.trackingLocked());
}

TEST(IcpAcceptanceGateTest, UsesStrictGateAfterLock) {
  auto gate = makeGate(1);
  ASSERT_TRUE(gate.evaluate(1.20, 0.80, 0.40).just_locked);

  const auto rejected = gate.evaluate(0.70, 0.10, 0.20);
  EXPECT_FALSE(rejected.update_pose);
  EXPECT_FALSE(rejected.publish_pose);
  EXPECT_TRUE(gate.trackingLocked());

  const auto accepted = gate.evaluate(0.10, 0.05, 0.20);
  EXPECT_TRUE(accepted.update_pose);
  EXPECT_TRUE(accepted.publish_pose);
}

TEST(IcpAcceptanceGateTest, ResetReturnsLockedGateToAcquisition) {
  auto gate = makeGate();
  ASSERT_TRUE(gate.evaluate(1.20, 0.80, 0.40).update_pose);
  ASSERT_TRUE(gate.evaluate(0.08, 0.04, 0.30).update_pose);
  ASSERT_TRUE(gate.evaluate(0.05, 0.03, 0.25).just_locked);

  gate.reset();
  const auto acquisition = gate.evaluate(1.20, 0.80, 0.40);

  EXPECT_FALSE(gate.trackingLocked());
  EXPECT_TRUE(acquisition.update_pose);
  EXPECT_FALSE(acquisition.publish_pose);
}

TEST(IcpAcceptanceGateTest, ConfirmationKeepsStrictInitialFitness) {
  IcpAcceptanceGate gate;
  gate.configure({0.45, 0.60, 1.0}, {2.5, 1.57, 0.50}, 3);
  ASSERT_TRUE(gate.evaluate(1.20, 0.80, 0.40).update_pose);

  const auto rejected = gate.evaluate(0.10, 0.05, 0.70);

  EXPECT_FALSE(rejected.update_pose);
  EXPECT_EQ(gate.confirmationCount(), 0);
}

TEST(IcpAcceptanceGateTest, MovingAcquisitionCandidateRestartsConfirmation) {
  auto gate = makeGate();
  ASSERT_TRUE(gate.evaluate(1.20, 0.80, 0.40).update_pose);
  ASSERT_EQ(gate.confirmationCount(), 1);

  const auto restarted = gate.evaluate(0.80, 0.10, 0.30);

  EXPECT_TRUE(restarted.update_pose);
  EXPECT_TRUE(restarted.restarted_confirmation);
  EXPECT_FALSE(restarted.publish_pose);
  EXPECT_EQ(gate.confirmationCount(), 1);
}

TEST(IcpAcceptanceGateTest, RejectsBadFitnessAndNonFiniteMetrics) {
  auto gate = makeGate();

  EXPECT_FALSE(gate.evaluate(0.10, 0.10, 1.20).update_pose);
  EXPECT_FALSE(gate.evaluate(std::numeric_limits<double>::quiet_NaN(),
                             0.10, 0.20).update_pose);
  EXPECT_EQ(gate.confirmationCount(), 0);
}

}  // namespace
}  // namespace fast_icp_loc
