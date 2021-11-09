#include <gtest/gtest.h>
#include "gtest_cout.h"

#include <hubero_local_planner/geometry/geometry.h>
#include <hubero_local_planner/fuzz/social_conductor.h>

#include <chrono>
#include <thread>

using namespace hubero_local_planner::geometry;
using namespace hubero_local_planner::fuzz;

// Test cases
TEST(FuzzySocialConductor, behaviourStrength) {
    EXPECT_EQ(SocialConductor::computeBehaviourStrength(2.00 * SocialConductor::SOCIAL_BEHAVIOUR_RANGE), 0.00);
    EXPECT_EQ(SocialConductor::computeBehaviourStrength(1.00 * SocialConductor::SOCIAL_BEHAVIOUR_RANGE), 0.00);
    EXPECT_EQ(SocialConductor::computeBehaviourStrength(0.75 * SocialConductor::SOCIAL_BEHAVIOUR_RANGE), 0.25);
    EXPECT_EQ(SocialConductor::computeBehaviourStrength(0.50 * SocialConductor::SOCIAL_BEHAVIOUR_RANGE), 0.50);
    EXPECT_EQ(SocialConductor::computeBehaviourStrength(0.25 * SocialConductor::SOCIAL_BEHAVIOUR_RANGE), 0.75);
    EXPECT_NEAR(SocialConductor::computeBehaviourStrength(0.0 * SocialConductor::SOCIAL_BEHAVIOUR_RANGE), 1.00, 1e-03);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}