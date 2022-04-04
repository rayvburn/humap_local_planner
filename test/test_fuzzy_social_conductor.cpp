#include <gtest/gtest.h>
#include "gtest_cout.h"

#include <hubero_local_planner/geometry/geometry.h>
#include <hubero_local_planner/fuzz/social_conductor.h>

#include <chrono>
#include <memory>
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

TEST(FuzzySocialConductor, behaviourForceOrientationSimple) {
    SocialConductor sc;

    hubero_local_planner::FisParams params {};
    params.force_factor = 1.0;
    std::shared_ptr<const hubero_local_planner::FisParams> cfg =
        std::make_shared<hubero_local_planner::FisParams>(params);
    sc.initialize(cfg);

    Pose robot1(0.0, 0.0, IGN_PI_4);
    const double DIST1 = 1.0; // meters
    Processor::FisOutput fis_output {};
    fis_output.value = -IGN_PI_4;
    fis_output.term_name = "turn_right_accelerate";
    fis_output.membership = 1.0;

    // vectors
    std::vector<Processor::FisOutput> fis_outputs_v {fis_output};
    std::vector<double> dist_v {DIST1};
    sc.computeBehaviourForce(robot1, fis_outputs_v, dist_v);
    Vector sc_vector = sc.getSocialVector();
    EXPECT_NEAR(sc_vector.calculateDirection().getRadian(), robot1.getYaw() + fis_output.value, 1e-06);
}

// distance increased to prevent overflow of unit vector lengths
TEST(FuzzySocialConductor, behaviourForceOrientationAdditivity) {
    SocialConductor sc;

    hubero_local_planner::FisParams params {};
    params.force_factor = 1.0;
    std::shared_ptr<const hubero_local_planner::FisParams> cfg =
        std::make_shared<hubero_local_planner::FisParams>(params);
    sc.initialize(cfg);

    Pose robot1(0.0, 0.0, 0.0);
    const double DIST = SocialConductor::SOCIAL_BEHAVIOUR_RANGE * 0.9;
    Processor::FisOutput fis_output1 {};
    fis_output1.value = -IGN_PI_4;
    fis_output1.term_name = "turn_right_accelerate";
    fis_output1.membership = 1.0;

    Processor::FisOutput fis_output2 {};
    fis_output2.value = -3.0 * IGN_PI_4;
    fis_output2.term_name = "turn_right_decelerate";
    fis_output2.membership = 1.0;

    // vector additivity rule check
    Vector v1(Angle(fis_output1.value));
    Vector v2(Angle(fis_output2.value));
    Vector v_result = v1 + v2;

    // vectors
    std::vector<Processor::FisOutput> fis_outputs_v {fis_output1, fis_output2};
    std::vector<double> dist_v {DIST, DIST};
    sc.computeBehaviourForce(robot1, fis_outputs_v, dist_v);
    Vector sc_vector = sc.getSocialVector();
    EXPECT_DOUBLE_EQ(sc_vector.calculateDirection().getRadian(), v_result.calculateDirection().getRadian());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}