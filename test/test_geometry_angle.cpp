#include <gtest/gtest.h>
#include "gtest_cout.h"

#include <hubero_local_planner/geometry/geometry.h>

using namespace hubero_local_planner::geometry;

TEST(HuberoGeometryAngle, ctor) {
    Angle a1;
    ASSERT_DOUBLE_EQ(a1.getRadian(), 0.0);

    ignition::math::Angle a2_ign(IGN_PI);
    Angle a2(a2_ign);
    ASSERT_DOUBLE_EQ(a2.getRadian(), IGN_PI);

    // getRadian on the fly
    double a3_rad = Angle(IGN_PI - 2.0 * IGN_PI).getRadian();
    ASSERT_DOUBLE_EQ(a3_rad, -IGN_PI);
}

TEST(HuberoGeometryAngle, angleFromVector) {
    Vector v1(0.0, 0.0, 0.0);
    Angle a1(v1);
    EXPECT_DOUBLE_EQ(a1.getRadian(), 0.0);

    Vector v2(1.0, 0.0, 0.0);
    Angle a2(v2);
    EXPECT_DOUBLE_EQ(a2.getRadian(), 0.0);

    Vector v3(-1.0, -1.0, 10.0);
    Angle a3(v3);
    EXPECT_DOUBLE_EQ(a3.getRadian(), -3.0 * IGN_PI_4);

    Vector v4(-5.0, -1.0, 2.5);
    Angle a4(v4);
    EXPECT_NEAR(a4.getRadian(), -2.9442, 1e-04);

    Vector v5(+2.0, -2.0, 2.5);
    Angle a5(v5);
    EXPECT_DOUBLE_EQ(a5.getRadian(), -IGN_PI_4);

    // same as v5
    ignition::math::Vector3d v6(+2.0, -2.0, 2.5);
    Angle a6(v6);
    EXPECT_DOUBLE_EQ(a5.getRadian(), -IGN_PI_4);
}

TEST(HuberoGeometryAngle, normalization) {
    // wrap above +PI
    Angle a1(IGN_PI + 0.001);
    EXPECT_NEAR(a1.getRadian(), -IGN_PI, 1e-03);

    // wrap below -PI
    Angle a2(-IGN_PI - 0.001);
    EXPECT_NEAR(a2.getRadian(), +IGN_PI, 1e-03);

    Angle a3(IGN_PI + 0.001, false);
    EXPECT_NEAR(a3.getRadian(), +IGN_PI, 1e-03);

    Angle a4(IGN_PI + 1.0, false);
    EXPECT_DOUBLE_EQ(a4.getRadian(), +IGN_PI + 1.0);

    Angle a5(IGN_PI - 10.0, false);
    EXPECT_DOUBLE_EQ(a5.getRadian(), IGN_PI - 10.0);

    Angle a6(IGN_PI - 10.0, false);
    Angle a7(IGN_PI - 10.0);
    EXPECT_DOUBLE_EQ(a6.normalized().getRadian(), a7.getRadian());
    a6.normalize();
    EXPECT_DOUBLE_EQ(a6.getRadian(), a7.getRadian());
}

TEST(HuberoGeometryAngle, addition) {
    Angle ang_pi_2(IGN_PI_2);
    Angle ang_pi(IGN_PI);
    ASSERT_DOUBLE_EQ(ang_pi_2.getRadian(), IGN_PI_2);
    ASSERT_DOUBLE_EQ(ang_pi.getRadian(), IGN_PI);

    Angle a1_copy(ang_pi_2 + ang_pi);
    Angle a1_add = ang_pi_2 + ang_pi;
    ASSERT_DOUBLE_EQ(a1_copy.getRadian(), a1_add.getRadian());
    ASSERT_DOUBLE_EQ(a1_copy.normalized().getRadian(), a1_add.normalized().getRadian());
    ASSERT_DOUBLE_EQ(a1_copy.getRadian(), IGN_PI_2 + IGN_PI);
    ASSERT_DOUBLE_EQ(a1_copy.normalized().getRadian(), -IGN_PI_2);

    Angle a2(IGN_PI_2 + IGN_PI, false);
    ASSERT_DOUBLE_EQ(a2.getRadian(), IGN_PI_2 + IGN_PI);
    // make sure that copy constructor taking Angles returns the same value as the one taking raw double
    ASSERT_DOUBLE_EQ(a1_copy.getRadian(), a2.getRadian());

    // normalize (implicit 2nd arg: `true`)
    Angle a3(IGN_PI_2 + IGN_PI);
    EXPECT_DOUBLE_EQ(a3.getRadian(), a3.normalized().getRadian());
    EXPECT_DOUBLE_EQ(a3.getRadian(), -IGN_PI_2);
}

TEST(HuberoGeometryAngle, subtraction) {
    Angle ang_pi_2(IGN_PI_2);
    Angle ang_pi(IGN_PI);
    ASSERT_DOUBLE_EQ(ang_pi_2.getRadian(), IGN_PI_2);
    ASSERT_DOUBLE_EQ(ang_pi.getRadian(), IGN_PI);

    Angle a1_copy(ang_pi_2 - ang_pi);
    Angle a1_sub = ang_pi_2 - ang_pi;
    ASSERT_DOUBLE_EQ(a1_copy.getRadian(), a1_sub.getRadian());
    ASSERT_DOUBLE_EQ(a1_copy.normalized().getRadian(), a1_sub.normalized().getRadian());
    ASSERT_DOUBLE_EQ(a1_copy.getRadian(), IGN_PI_2 - IGN_PI);
    ASSERT_DOUBLE_EQ(a1_copy.normalized().getRadian(), -IGN_PI_2);

    Angle a2(IGN_PI_2 - IGN_PI, false);
    ASSERT_DOUBLE_EQ(a2.getRadian(), IGN_PI_2 - IGN_PI);
    ASSERT_DOUBLE_EQ(a1_copy.getRadian(), a2.getRadian());

    // normalize (implicit 2nd arg: `true`)
    Angle a3(IGN_PI_2 - IGN_PI);
    ASSERT_DOUBLE_EQ(a3.getRadian(), a3.normalized().getRadian());
    ASSERT_DOUBLE_EQ(a3.getRadian(), -IGN_PI_2);

    // with assignment
    Angle a4(IGN_PI);
    a4 -= ang_pi_2;
    ASSERT_DOUBLE_EQ(a4.getRadian(), IGN_PI_2);

    Angle a5(IGN_PI_2);
    a5 -= ang_pi;
    ASSERT_DOUBLE_EQ(a5.getRadian(), IGN_PI_2 - IGN_PI);
    a5.normalize();
    ASSERT_DOUBLE_EQ(a5.getRadian(), -IGN_PI_2);
}

TEST(HuberoGeometryAngle, multiplication) {
    Angle ang_pi(IGN_PI);
    ASSERT_DOUBLE_EQ(ang_pi.getRadian(), IGN_PI);

    Angle ang_multi_pi(ang_pi);
    ang_multi_pi *= ang_pi;
    Angle ang_multi_pi_cctor(ang_pi * ang_pi);
    ASSERT_DOUBLE_EQ(ang_multi_pi.getRadian(), IGN_PI * IGN_PI);
    ASSERT_DOUBLE_EQ(ang_multi_pi.getRadian(), ang_multi_pi_cctor.getRadian());
    ASSERT_NEAR(ang_multi_pi.normalized().getRadian(), -2.6968, 1e-03);
    ASSERT_DOUBLE_EQ(ang_multi_pi.normalized().getRadian(), ang_multi_pi_cctor.normalized().getRadian());
}

TEST(HuberoGeometryAngle, division) {
    Angle ang_pi(IGN_PI);
    ASSERT_DOUBLE_EQ(ang_pi.getRadian(), IGN_PI);

    Angle ang_multi_pi(ang_pi);
    ang_multi_pi /= ang_pi;
    Angle ang_multi_pi_cctor(ang_pi / ang_pi);
    ASSERT_DOUBLE_EQ(ang_multi_pi.getRadian(), IGN_PI / IGN_PI);
    ASSERT_DOUBLE_EQ(ang_multi_pi.getRadian(), ang_multi_pi_cctor.getRadian());
    ASSERT_DOUBLE_EQ(ang_multi_pi.normalized().getRadian(), 1.0);
    ASSERT_DOUBLE_EQ(ang_multi_pi.normalized().getRadian(), ang_multi_pi_cctor.normalized().getRadian());
}

TEST(HuberoGeometryAngle, comparison) {
    ASSERT_TRUE(Angle(IGN_PI_2) > Angle(IGN_PI_4));
    ASSERT_TRUE(Angle(IGN_PI_2) >= Angle(IGN_PI_4));
    ASSERT_TRUE(Angle(IGN_PI_2) != Angle(IGN_PI_4));
    ASSERT_TRUE(Angle(IGN_PI_2) == Angle(IGN_PI_2));
    ASSERT_TRUE(Angle(IGN_PI_4) <= Angle(IGN_PI_4));
    ASSERT_TRUE(Angle(-IGN_PI_2) <= Angle(IGN_PI_4));
}

TEST(HuberoGeometryAngle, setter) {
    Angle a1;
    ASSERT_DOUBLE_EQ(a1.getRadian(), 0.0);
    a1.setRadian(IGN_PI);
    ASSERT_DOUBLE_EQ(a1.getRadian(), IGN_PI);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
