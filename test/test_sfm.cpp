#include <gtest/gtest.h>
#include "gtest_cout.h"
#include <hubero_local_planner/sfm/social_force_model.h>

using namespace hubero::geometry;

// provide access to protected methods
class SfmExposed: public sfm::SocialForceModel {
public:
    static Vector computeInternalForce(
        const Vector& vel_robot,
        const Vector& d_robot_object,
        const double& mass,
        const double& speed_desired,
        const double& relaxation_time
    ) {
        return sfm::SocialForceModel::computeInternalForce(vel_robot, d_robot_object, mass, speed_desired, relaxation_time);
    }

    static Angle computeThetaAlphaBetaAngle2011(const Vector& robot, const Vector& object) {
        return sfm::SocialForceModel::computeThetaAlphaBetaAngle2011(robot, object);
    }

    static Angle computeThetaAlphaBetaAngle2014(const Vector& n_alpha, const Vector& d_alpha_beta) {
        return sfm::SocialForceModel::computeThetaAlphaBetaAngle2014(n_alpha, d_alpha_beta);
    }

    static Angle computeThetaAlphaBetaAngle(const Angle& actor_yaw, const Angle& object_yaw) {
        return sfm::SocialForceModel::computeThetaAlphaBetaAngle(actor_yaw, object_yaw);
    }

    static Vector computeNormalAlphaDirection(const Angle &robot_yaw, sfm::ParameterDescription param_description) {
        return sfm::SocialForceModel::computeNormalAlphaDirection(robot_yaw, param_description);
    }

    static Vector computePerpendicularToNormal(
			const Vector &n_alpha,
			const sfm::RelativeLocation &beta_rel_location,
			sfm::ParameterDescription param_description
	) {
        return sfm::SocialForceModel::computePerpendicularToNormal(n_alpha, beta_rel_location, param_description);
    }

    static double computeRelativeSpeed(const Vector& actor_vel, const Vector& object_vel) {
        return sfm::SocialForceModel::computeRelativeSpeed(actor_vel, object_vel);
    }
};

TEST(SocialForceModel, computeInternalForce) {
    hubero::geometry::Vector robot_pose1(0.0, 0.0, 0.0);
    hubero::geometry::Vector robot_vel1(0.0, 0.0, 0.0);
    hubero::geometry::Vector target1(5.0, 0.0, 0.0);
    hubero::geometry::Vector robot_target_dist1(target1 - robot_pose1);
    hubero::geometry::Vector force1 = SfmExposed::computeInternalForce(robot_vel1, robot_target_dist1, 100.0, 1.0, 0.5);
    hubero::geometry::Angle force1_dir(force1);
    ASSERT_DOUBLE_EQ(0.0, force1_dir.getRadian());


    hubero::geometry::Vector robot_pose2(-5.0, -10.0, -3.0 * IGN_PI_4);
    hubero::geometry::Vector robot_vel2(-1.0, -1.0, 0.0);
    hubero::geometry::Vector target2(-10.0, -15.0, 0.0);
    hubero::geometry::Vector robot_target_dist2(target2 - robot_pose2);
    hubero::geometry::Vector force2 = SfmExposed::computeInternalForce(robot_vel2, robot_target_dist2, 1.0, 10.0, 2.0);
    hubero::geometry::Angle force2_dir(force2);
    // keep the current velocity direction
    ASSERT_DOUBLE_EQ(-3.0 * IGN_PI_4, force2_dir.getRadian());


    hubero::geometry::Vector robot_pose3(0.0, 0.0, IGN_PI_2);
    hubero::geometry::Vector robot_vel3(+0.5, -1.0, 0.0);
    hubero::geometry::Vector target3(+5.0, -10.0, 0.0);
    hubero::geometry::Vector robot_target_dist3(target3 - robot_pose3);
    hubero::geometry::Vector force3 = SfmExposed::computeInternalForce(robot_vel3, robot_target_dist3, 1.0, 10.0, 2.0);
    hubero::geometry::Angle force3_dir(force3);
    ASSERT_NEAR(-1.1071, force3_dir.getRadian(), 1e-04); // atan2(-10, 5)


    // zero mass
    hubero::geometry::Vector force3_no_mass = SfmExposed::computeInternalForce(robot_vel3, robot_target_dist3, 0.0, 10.0, 2.0);
    ASSERT_NEAR(0.0, force3_no_mass.getX(), 1e-06);
    ASSERT_NEAR(0.0, force3_no_mass.getY(), 1e-06);
    ASSERT_NEAR(0.0, force3_no_mass.getZ(), 1e-06);
}

TEST(SocialForceModel, computeThetaAlphaBetaAngle2011) {
    // NOTE: computeThetaAlphaBetaAngle2011 uses only .vel of `DynamicObject`
    hubero::geometry::Vector robot_vel1(0.0, 0.0, 0.0);
    hubero::geometry::Vector object_vel1(1.0, -1.0, 0.0);
    Angle theta1 = SfmExposed::computeThetaAlphaBetaAngle2011(robot_vel1, object_vel1);
    // NaN since Eucl. norm of `robot_vel` is 0
    EXPECT_TRUE(std::isnan(theta1.getRadian()));

    hubero::geometry::Vector robot_vel2(2.0, 3.0, 0.3);
    hubero::geometry::Vector object_vel2(1.0, -1.0, 0.0);
    Angle theta2 = SfmExposed::computeThetaAlphaBetaAngle2011(robot_vel2, object_vel2);
    EXPECT_NEAR(theta2.getRadian(), 1.7675, 1e-04);

    hubero::geometry::Vector robot_vel3(1.0, -1.0, 0.0);
    hubero::geometry::Vector object_vel3(2.0, 3.0, 0.3);
    Angle theta3 = SfmExposed::computeThetaAlphaBetaAngle2011(robot_vel3, object_vel3);
    EXPECT_NEAR(theta3.getRadian(), 1.7675, 1e-04);
}

TEST(SocialForceModel, computeThetaAlphaBetaAngle2014) {
    // angle difference
    hubero::geometry::Vector n_alpha1(1.0, 0.0, 0.0);
    hubero::geometry::Vector d_alpha_beta1(2.0, 3.0, 1.0);
    Angle theta1 = SfmExposed::computeThetaAlphaBetaAngle2014(n_alpha1, d_alpha_beta1);
    EXPECT_NEAR(theta1.getRadian(), -0.9828, 1e-04);

    // angle between n_alpha and d_alpha_beta
    hubero::geometry::Vector n_alpha2(Angle(-IGN_PI_4));
    hubero::geometry::Vector d_alpha_beta2(Angle(+IGN_PI_2));
    Angle theta2 = SfmExposed::computeThetaAlphaBetaAngle2014(n_alpha2, d_alpha_beta2);
    EXPECT_NEAR(theta2.getRadian(), Angle(-IGN_PI_4 - IGN_PI_2).getRadian(), 1e-04);
}

TEST(SocialForceModel, computeThetaAlphaBetaAngle) {
    // angle between robot_yaw and object_yaw
    hubero::geometry::Angle robot_yaw1(-IGN_PI_4);
    hubero::geometry::Angle object_yaw1(+IGN_PI_2);
    Angle theta1 = SfmExposed::computeThetaAlphaBetaAngle(robot_yaw1, object_yaw1);
    EXPECT_DOUBLE_EQ(theta1.getRadian(), Angle(-IGN_PI_4 - IGN_PI_2).getRadian());

    hubero::geometry::Angle robot_yaw2(IGN_PI_4);
    hubero::geometry::Angle object_yaw2(IGN_PI_2);
    Angle theta2 = SfmExposed::computeThetaAlphaBetaAngle(robot_yaw2, object_yaw2);
    EXPECT_DOUBLE_EQ(theta2.getRadian(), Angle(IGN_PI_4 - IGN_PI_2).getRadian());
}

TEST(SocialForceModel, computeNormalAlphaDirection) {
    // points in the opposite direction of the robot motion
    hubero::geometry::Angle robot_yaw1(IGN_PI_4);
    hubero::geometry::Vector robot_n1 = SfmExposed::computeNormalAlphaDirection(robot_yaw1, sfm::PARAMETER_DESCRIPTION_2011);
    hubero::geometry::Angle robot_n1_dir = robot_n1.calculateDirection();
    hubero::geometry::Angle expected1(robot_yaw1.getRadian() + IGN_PI);
    EXPECT_DOUBLE_EQ(expected1.getRadian(), robot_n1_dir.getRadian());

    hubero::geometry::Angle robot_yaw2(3.0 * IGN_PI_4);
    hubero::geometry::Vector robot_n2 = SfmExposed::computeNormalAlphaDirection(robot_yaw2, sfm::PARAMETER_DESCRIPTION_2011);
    hubero::geometry::Angle robot_n2_dir = robot_n2.calculateDirection();
    hubero::geometry::Angle expected2(robot_yaw2.getRadian() + IGN_PI);
    EXPECT_DOUBLE_EQ(expected2.getRadian(), robot_n2_dir.getRadian());

    // points in the direction of the robot motion
    hubero::geometry::Angle robot_yaw3(3.0 * IGN_PI_4);
    hubero::geometry::Vector robot_n3 = SfmExposed::computeNormalAlphaDirection(robot_yaw3, sfm::PARAMETER_DESCRIPTION_2014);
    hubero::geometry::Angle robot_n3_dir = robot_n3.calculateDirection();
    hubero::geometry::Angle expected3(robot_yaw3.getRadian());
    EXPECT_DOUBLE_EQ(expected3.getRadian(), robot_n3_dir.getRadian());

    hubero::geometry::Angle robot_yaw4(-IGN_PI_2);
    hubero::geometry::Vector robot_n4 = SfmExposed::computeNormalAlphaDirection(robot_yaw4, sfm::PARAMETER_DESCRIPTION_2014);
    hubero::geometry::Angle robot_n4_dir = robot_n4.calculateDirection();
    hubero::geometry::Angle expected4(robot_yaw4.getRadian());
    EXPECT_DOUBLE_EQ(expected4.getRadian(), robot_n4_dir.getRadian());
}

TEST(SocialForceModel, computePerpendicularToNormal) {
    // PARAMETER_DESCRIPTION_2014 -> perpendicular to normal, pointing to the direction opposite to `rel_loc`
    // PARAMETER_DESCRIPTION_2011 -> perpendicular to normal, pointing to the direction complaint with `rel_loc`
    hubero::geometry::Vector n_a1(1.0, 0.0, 0.0);
    sfm::RelativeLocation rel_loc1 = sfm::LOCATION_RIGHT;

    sfm::ParameterDescription param_desc_1a = sfm::PARAMETER_DESCRIPTION_2014;
    hubero::geometry::Vector p_a1a = SfmExposed::computePerpendicularToNormal(n_a1, rel_loc1, param_desc_1a);
    hubero::geometry::Vector n_a1a_expected = n_a1.rotated(IGN_PI_2);
    EXPECT_NEAR(n_a1a_expected.getX(), p_a1a.getX(), 1e-06);
    EXPECT_NEAR(n_a1a_expected.getY(), p_a1a.getY(), 1e-06);
    EXPECT_NEAR(n_a1a_expected.getZ(), p_a1a.getZ(), 1e-06);

    sfm::ParameterDescription param_desc_1b = sfm::PARAMETER_DESCRIPTION_2011;
    hubero::geometry::Vector p_a1b = SfmExposed::computePerpendicularToNormal(n_a1, rel_loc1, param_desc_1b);
    hubero::geometry::Vector n_a1b_expected = n_a1.rotated(-IGN_PI_2);
    EXPECT_NEAR(n_a1b_expected.getX(), p_a1b.getX(), 1e-06);
    EXPECT_NEAR(n_a1b_expected.getY(), p_a1b.getY(), 1e-06);
    EXPECT_NEAR(n_a1b_expected.getZ(), p_a1b.getZ(), 1e-06);


    hubero::geometry::Vector n_a2(Angle(-IGN_PI_4));
    sfm::RelativeLocation rel_loc2 = sfm::LOCATION_LEFT;

    sfm::ParameterDescription param_desc_2a = sfm::PARAMETER_DESCRIPTION_2014;
    hubero::geometry::Vector p_a2a = SfmExposed::computePerpendicularToNormal(n_a2, rel_loc2, param_desc_2a);
    hubero::geometry::Vector n_a2a_expected = n_a2.rotated(-IGN_PI_2);
    EXPECT_NEAR(n_a2a_expected.getX(), p_a2a.getX(), 1e-06);
    EXPECT_NEAR(n_a2a_expected.getY(), p_a2a.getY(), 1e-06);
    EXPECT_NEAR(n_a2a_expected.getZ(), p_a2a.getZ(), 1e-06);

    sfm::ParameterDescription param_desc_2b = sfm::PARAMETER_DESCRIPTION_2011;
    hubero::geometry::Vector p_a2b = SfmExposed::computePerpendicularToNormal(n_a2, rel_loc2, param_desc_2b);
    hubero::geometry::Vector n_a2b_expected = n_a2.rotated(+IGN_PI_2);
    EXPECT_NEAR(n_a2b_expected.getX(), p_a2b.getX(), 1e-06);
    EXPECT_NEAR(n_a2b_expected.getY(), p_a2b.getY(), 1e-06);
    EXPECT_NEAR(n_a2b_expected.getZ(), p_a2b.getZ(), 1e-06);
}

TEST(SocialForceModel, computeRelativeSpeed) {
    // opposite direction
    Vector robot_vel1(1.25, 1.25, 0.0);
    Vector object_vel1(-1.25, -1.25, 0.0);
    Vector expected_vel1(robot_vel1 - object_vel1);
    double speed1 = SfmExposed::computeRelativeSpeed(robot_vel1, object_vel1);
    EXPECT_NEAR(expected_vel1.calculateLength(), speed1, 1e-04);

    // same direction
    Vector robot_vel2(2.75, 2.75, 0.0);
    Vector object_vel2(2.50, 2.50, 0.0);
    Vector expected_vel2(robot_vel2 - object_vel2);
    double speed2 = SfmExposed::computeRelativeSpeed(robot_vel2, object_vel2);
    EXPECT_NEAR(expected_vel2.calculateLength(), speed2, 1e-04);

    // non-zero ang
    Vector robot_vel3(2.75, 2.75, +IGN_PI_4);
    Vector object_vel3(2.50, 2.50, +IGN_PI_4);
    Vector expected_vel3(robot_vel3 - object_vel3);
    double speed3 = SfmExposed::computeRelativeSpeed(robot_vel3, object_vel3);
    // `expected_vel3` does not account for angular velocity, so the value is not located within tolerance
    double not_expected_speed3 = expected_vel3.calculateLength();
    EXPECT_FALSE(
        not_expected_speed3 > (speed3 + 1e-04)
        && not_expected_speed3 < (speed3 - 1e-04)
    );
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
