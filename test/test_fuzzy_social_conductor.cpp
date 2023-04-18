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
    SocialConductor sc;
    auto params_ptr = std::make_shared<hubero_local_planner::FisParams>();
    sc.initialize(params_ptr);

    // params to check against
    double dist_max = params_ptr->human_action_range;
    double speed_max_single = SocialConductor::RELATIVE_SPEED_MAX / 2.0;

    // speeds does not matter when distance is too big
    EXPECT_DOUBLE_EQ(SocialConductor::computeBehaviourStrength(dist_max, 2.00 * dist_max, 1e06, 1e06), 0.00);
    EXPECT_DOUBLE_EQ(SocialConductor::computeBehaviourStrength(dist_max, 1.01 * dist_max, 1e06, 1e06), 0.00);
    EXPECT_FALSE(    SocialConductor::computeBehaviourStrength(dist_max, 0.99 * dist_max, 1e06, 1e06) == 0.00);

    // no effect when obstacles are not moving
    EXPECT_DOUBLE_EQ(SocialConductor::computeBehaviourStrength(dist_max, 0.50 * dist_max, 0.0, 0.0), 0.00);

    // one b.strength is greater than another due to higher speeds
    auto s1 = SocialConductor::computeBehaviourStrength(dist_max, 0.50 * dist_max, speed_max_single / 2.0, speed_max_single / 2.0);
    EXPECT_GT(s1, 0.00);
    auto s2 = SocialConductor::computeBehaviourStrength(dist_max, 0.50 * dist_max, speed_max_single, speed_max_single);
    EXPECT_GT(s2, s1);
}

TEST(FuzzySocialConductor, behaviourForceOrientationSimple) {
    SocialConductor sc;

    hubero_local_planner::FisParams params {};
    params.force_factor = 1.0;
    std::shared_ptr<const hubero_local_planner::FisParams> cfg =
        std::make_shared<hubero_local_planner::FisParams>(params);
    sc.initialize(cfg);

    Pose robot1(0.0, 0.0, IGN_PI_4);
    Vector robot1_vel(0.5, 0.5, 0.0);
    // some non-zero speed is required for a valid output vector
    double robot1_speed = robot1_vel.calculateLength();
    const double DIST1 = cfg->human_action_range / 2.0;
    Processor::FisOutput fis_output {};
    fis_output.value = -IGN_PI_4;
    fis_output.term_name = "turn_right_accelerate";
    fis_output.membership = 1.0;

    // vectors
    std::vector<Processor::FisOutput> fis_outputs_v {fis_output};
    std::vector<double> speed_v {robot1_speed};
    std::vector<double> dist_v {DIST1};

    // direction of the vector will be the sum of robot's current yaw angle and the FIS behaviour's vector direction
    sc.computeBehaviourForce(robot1, robot1_speed, fis_outputs_v, speed_v, dist_v);
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
    Vector robot1_vel(0.5, 0.0, 0.0);
    // some non-zero speed is required for a valid output vector
    double robot1_speed = robot1_vel.calculateLength();
    // set to lay withing meaningful bounds
    const double DIST = cfg->human_action_range * 0.9;
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
    std::vector<double> speeds_v {robot1_speed, robot1_speed};
    std::vector<double> dist_v {DIST, DIST};

    sc.computeBehaviourForce(robot1, robot1_speed, fis_outputs_v, speeds_v, dist_v);
    Vector sc_vector = sc.getSocialVector();
    EXPECT_DOUBLE_EQ(sc_vector.calculateDirection().getRadian(), v_result.calculateDirection().getRadian());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
