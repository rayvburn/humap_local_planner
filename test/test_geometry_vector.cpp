#include <gtest/gtest.h>
#include "gtest_cout.h"

#include <hubero_local_planner/geometry/geometry.h>

using namespace hubero::geometry;

static const double TOLERANCE_STRICT = 1e-09;
static const double TOLERANCE_SOFT = 1e-04;

TEST(HuberoGeometryVector, ctor_vector) {
    Vector v1;
    ASSERT_DOUBLE_EQ(v1.getX(), 0.0);
    ASSERT_DOUBLE_EQ(v1.getY(), 0.0);
    ASSERT_DOUBLE_EQ(v1.getZ(), 0.0);

    Vector v2(1.0, -2.0, 3.0);
    ASSERT_DOUBLE_EQ(v2.getX(), +1.0);
    ASSERT_DOUBLE_EQ(v2.getY(), -2.0);
    ASSERT_DOUBLE_EQ(v2.getZ(), +3.0);

    ignition::math::Vector3d v3_ign(v2.getX(), v2.getY(), v2.getZ());
    Vector v3(v3_ign);
    ASSERT_DOUBLE_EQ(v3.getX(), v2.getX());
    ASSERT_DOUBLE_EQ(v3.getY(), v2.getY());
    ASSERT_DOUBLE_EQ(v3.getZ(), v2.getZ());
    ASSERT_EQ(v3_ign, v3.getRawVector());
}

TEST(HuberoGeometryVector, setters) {
    Vector v1;
    ASSERT_DOUBLE_EQ(v1.getX(), 0.0);
    ASSERT_DOUBLE_EQ(v1.getY(), 0.0);
    ASSERT_DOUBLE_EQ(v1.getZ(), 0.0);
    v1.setX(1.0);
    ASSERT_DOUBLE_EQ(v1.getX(), 1.0);
    v1.setY(-2.5);
    ASSERT_DOUBLE_EQ(v1.getY(), -2.5);
    v1.setZ(3.33);
    ASSERT_DOUBLE_EQ(v1.getZ(), 3.33);
}

TEST(HuberoGeometryVector, rotation) {
    Vector v1(2.0, 0.0, 0.0);
    ASSERT_DOUBLE_EQ(v1.calculateDirection().getRadian(), 0.0);
    v1.rotate(IGN_PI_4);
    Angle v1_rot_dir(0.0 + IGN_PI_4);
    EXPECT_DOUBLE_EQ(v1.calculateDirection().getRadian(), v1_rot_dir.getRadian());

    Vector v2(0.0, 2.0, 0.0);
    ASSERT_DOUBLE_EQ(v2.calculateDirection().getRadian(), IGN_PI_2);
    Angle v2_rot_dir(IGN_PI_2 + IGN_PI_4);
    ASSERT_NEAR(v2.rotated(IGN_PI_4).calculateDirection().getRadian(), v2_rot_dir.getRadian(), TOLERANCE_STRICT);
    // make sure that v2 not modified by `rotated()` call
    ASSERT_DOUBLE_EQ(v2.calculateDirection().getRadian(), IGN_PI_2);

    Vector v3(0.0, -2.0, 0.0);
    ASSERT_DOUBLE_EQ(v3.calculateDirection().getRadian(), -IGN_PI_2);
    v3.rotate(+IGN_PI_2);
    Angle v3_rot_dir(-IGN_PI_2 + IGN_PI_2);
    ASSERT_NEAR(v3.calculateDirection().getRadian(), v3_rot_dir.getRadian(), TOLERANCE_STRICT);

    Vector v4(-2.0, 0.0, 0.0);
    ASSERT_DOUBLE_EQ(v4.calculateDirection().getRadian(), IGN_PI);
    Angle v4_rot_dir(IGN_PI + IGN_PI_2);
    ASSERT_NEAR(v4.rotated(IGN_PI_2).calculateDirection().getRadian(), v4_rot_dir.getRadian(), TOLERANCE_STRICT);
}

TEST(HuberoGeometryVector, ctor_angle) {
    Angle a1(IGN_PI_4);
    Vector v1(a1);
    ASSERT_NEAR(v1.calculateDirection().getRadian(), IGN_PI_4, TOLERANCE_STRICT);

    Angle a2(-3.0 * IGN_PI_4);
    Vector v2(a2);
    ASSERT_NEAR(v2.calculateDirection().getRadian(), -3.0 * IGN_PI_4, TOLERANCE_STRICT);

    Angle a3(2.0 * IGN_PI);
    Vector v3(a3);
    ASSERT_NEAR(v3.calculateDirection().getRadian(), 0.0, TOLERANCE_STRICT); // 2PI normalized
}

TEST(HuberoGeometryVector, ctor_pose2d) {
    // reference
    Pose pose_ref(0.567, 1.234, 0.987, 0.0, 0.0, IGN_PI);

    Vector v_from_tf(pose_ref.getAsTfPose());
    ASSERT_DOUBLE_EQ(v_from_tf.getX(), pose_ref.getX());
    ASSERT_DOUBLE_EQ(v_from_tf.getY(), pose_ref.getY());
    ASSERT_DOUBLE_EQ(v_from_tf.getZ(), pose_ref.getYaw());

    Vector v_from_geom_msg(pose_ref.getAsMsgPose());
    ASSERT_DOUBLE_EQ(v_from_geom_msg.getX(), pose_ref.getX());
    ASSERT_DOUBLE_EQ(v_from_geom_msg.getY(), pose_ref.getY());
    ASSERT_DOUBLE_EQ(v_from_geom_msg.getZ(), pose_ref.getYaw());
}

TEST(HuberoGeometryVector, conversion) {
    Vector v_ref(-2.123, -3.321, +5.987);

    ignition::math::Vector3d v_ign(v_ref.getX(), v_ref.getY(), v_ref.getZ());
    Vector v_from_ign(v_ign);
    ASSERT_EQ(v_from_ign.getRawVector(), v_ign);

    Eigen::Vector2d v_eigen2d(v_ref.getX(), v_ref.getY());
    Vector v_from_eigen2d(v_eigen2d);
    ASSERT_EQ(v_from_eigen2d.getX(), v_eigen2d[0]);
    ASSERT_EQ(v_from_eigen2d.getY(), v_eigen2d[1]);

    Eigen::Vector3d v_eigen3d(v_ref.getX(), v_ref.getY(), v_ref.getZ());
    Vector v_from_eigen3d(v_eigen3d);
    ASSERT_EQ(v_from_eigen3d.getAsEigen(), v_eigen3d);

    geometry_msgs::Twist v_twist;
    v_twist.linear.x = v_ref.getX();
    v_twist.linear.y = v_ref.getY();
    v_twist.angular.z = v_ref.getZ();
    Vector v_from_twist(v_twist);
    geometry_msgs::Twist v_twist_out = v_from_twist.getAsTwist();
    ASSERT_EQ(v_twist_out.linear.x, v_twist.linear.x);
    ASSERT_EQ(v_twist_out.linear.y, v_twist.linear.y);
    ASSERT_EQ(v_twist_out.angular.z, v_twist.angular.z);
}

TEST(HuberoGeometryVector, normalization) {
    Vector v1(0.0, 10.0, 0.0);
    v1.normalize();
    ASSERT_DOUBLE_EQ(v1.getX(), 0.0);
    ASSERT_DOUBLE_EQ(v1.getY(), 1.0);
    ASSERT_DOUBLE_EQ(v1.getZ(), 0.0);

    Vector v2(100.0, 100.0, 0.0);
    v2.normalize();
    ASSERT_NEAR(v2.getX(), 0.70710678118, TOLERANCE_STRICT);
    ASSERT_NEAR(v2.getY(), 0.70710678118, TOLERANCE_STRICT);
    ASSERT_NEAR(v2.getZ(), 0.0, TOLERANCE_STRICT);

    Vector v3(25.0, 25.0, 25.0);
    v3.normalize();
    ASSERT_NEAR(v3.getX(), 0.5774, TOLERANCE_SOFT);
    ASSERT_NEAR(v3.getY(), 0.5774, TOLERANCE_SOFT);
    ASSERT_NEAR(v3.getZ(), 0.5774, TOLERANCE_SOFT);
}

TEST(HuberoGeometryVector, computations_misc) {
    // Length
    Vector v1(0.0, 10.0, 0.0);
    ASSERT_NEAR(v1.calculateLength(), 10.0, TOLERANCE_SOFT);

    Vector v2(-2.0, -2.0, 0.0);
    ASSERT_NEAR(v2.calculateLength(), 2.8284, TOLERANCE_SOFT);

    Vector v3(-3.0, +4.0, -5.0);
    ASSERT_NEAR(v3.calculateLength(), 7.0711, TOLERANCE_SOFT);

    // Dot
    Vector v_dot(-2.0, -3.0, -5.0);
    Vector v4(1.0, -3.0, +5.0);
    // results verified with Matlab implementation
    ASSERT_NEAR(v4.dot(v_dot), -18.0, TOLERANCE_SOFT);
    ASSERT_NEAR(v_dot.dot(v_dot), +38.0, TOLERANCE_SOFT);

    // Cross
    Vector v_cross(0.5, 2.5, 1.25);
    Vector v5(-10.0, 25.0, 40.0);
    Vector vc5 = v5.cross(v_cross);
    ASSERT_NEAR(vc5.getX(), -68.7500, TOLERANCE_SOFT);
    ASSERT_NEAR(vc5.getY(), +32.5000, TOLERANCE_SOFT);
    ASSERT_NEAR(vc5.getZ(), -37.5000, TOLERANCE_SOFT);
}

TEST(HuberoGeometryVector, addition) {
    Vector v1;
    Vector v1_add(1.0, -0.2, 3.0);
    Vector v1_res(v1 + v1_add);
    ASSERT_DOUBLE_EQ(v1_res.getX(), v1_add.getX());
    ASSERT_DOUBLE_EQ(v1_res.getY(), v1_add.getY());
    ASSERT_DOUBLE_EQ(v1_res.getZ(), v1_add.getZ());

    Vector v2(10.0, 20.0, 30.0);
    Vector v2_add(-10.0, -10.0, -30.0);
    Vector v2_res = v2 + v2_add;
    ASSERT_DOUBLE_EQ(v2_res.getX(), 0.0);
    ASSERT_DOUBLE_EQ(v2_res.getY(), 10.0);
    ASSERT_DOUBLE_EQ(v2_res.getZ(), 0.0);

    v2 += v2_add;
    ASSERT_DOUBLE_EQ(v2.getX(), 0.0);
    ASSERT_DOUBLE_EQ(v2.getY(), 10.0);
    ASSERT_DOUBLE_EQ(v2.getZ(), 0.0);
}

TEST(HuberoGeometryVector, subtraction) {
    Vector v1(15.0, 12.0, -3.0);
    Vector v2(-3.0, 0.0, +15.0);

    Vector v_sub1(v1 - v2);
    ASSERT_DOUBLE_EQ(v_sub1.getX(), +18.0);
    ASSERT_DOUBLE_EQ(v_sub1.getY(), +12.0);
    ASSERT_DOUBLE_EQ(v_sub1.getZ(), -18.0);

    Vector v_sub2 = v1 - v2;
    ASSERT_DOUBLE_EQ(v_sub2.getX(), +18.0);
    ASSERT_DOUBLE_EQ(v_sub2.getY(), +12.0);
    ASSERT_DOUBLE_EQ(v_sub2.getZ(), -18.0);

    v1 -= v2;
    ASSERT_DOUBLE_EQ(v1.getX(), +18.0);
    ASSERT_DOUBLE_EQ(v1.getY(), +12.0);
    ASSERT_DOUBLE_EQ(v1.getZ(), -18.0);
}

TEST(HuberoGeometryVector, multiplication) {
    Vector v1(15.0, 12.0, -3.0);
    Vector v2(-3.0, 0.0, +3.0);
    double scalar_mult = 2.0;

    Vector v_mult1(v1 * v2);
    ASSERT_DOUBLE_EQ(v_mult1.getX(), -45.0);
    ASSERT_DOUBLE_EQ(v_mult1.getY(), 0.0);
    ASSERT_DOUBLE_EQ(v_mult1.getZ(), -9.0);

    Vector v_mult2 = v1 * v2;
    ASSERT_DOUBLE_EQ(v_mult2.getX(), -45.0);
    ASSERT_DOUBLE_EQ(v_mult2.getY(), 0.0);
    ASSERT_DOUBLE_EQ(v_mult2.getZ(), -9.0);

    v1 *= v2;
    ASSERT_DOUBLE_EQ(v1.getX(), -45.0);
    ASSERT_DOUBLE_EQ(v1.getY(), 0.0);
    ASSERT_DOUBLE_EQ(v1.getZ(), -9.0);

    v2 *= scalar_mult;
    ASSERT_DOUBLE_EQ(v2.getX(), -6.0);
    ASSERT_DOUBLE_EQ(v2.getY(), 0.0);
    ASSERT_DOUBLE_EQ(v2.getZ(), 6.0);
}

TEST(HuberoGeometryVector, division) {
    Vector v1(15.0, 12.0, -3.0);
    Vector v2(-3.0, 1.0, +3.0);
    Vector v2_z(-3.0, 0.0, +3.0);
    double scalar_div = 2.0;

    // zero division
    Vector v_div1z(v1 / v2_z);
    ASSERT_DOUBLE_EQ(v_div1z.getX(), -5.0);
    ASSERT_TRUE(std::isinf(v_div1z.getY()));
    ASSERT_DOUBLE_EQ(v_div1z.getZ(), -1.0);

    Vector v_div1(v1 / v2);
    ASSERT_DOUBLE_EQ(v_div1.getX(), -5.0);
    ASSERT_DOUBLE_EQ(v_div1.getY(), 12.0);
    ASSERT_DOUBLE_EQ(v_div1.getZ(), -1.0);

    Vector v_div2 = v1 / v2;
    ASSERT_DOUBLE_EQ(v_div2.getX(), -5.0);
    ASSERT_DOUBLE_EQ(v_div2.getY(), 12.0);
    ASSERT_DOUBLE_EQ(v_div2.getZ(), -1.0);

    v1 /= v2;
    ASSERT_DOUBLE_EQ(v1.getX(), -5.0);
    ASSERT_DOUBLE_EQ(v1.getY(), 12.0);
    ASSERT_DOUBLE_EQ(v1.getZ(), -1.0);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
