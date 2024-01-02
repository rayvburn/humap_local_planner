#include <gtest/gtest.h>
#include "gtest_cout.h"

#include <humap_local_planner/geometry/geometry.h>

using namespace humap_local_planner::geometry;

TEST(HumapGeometryQuaternion, ctor) {
    // yaw
    Quaternion quat1(IGN_PI_4);
    ASSERT_NEAR(quat1.getRoll(), 0.0, 1e-06);
    ASSERT_NEAR(quat1.getPitch(), 0.0, 1e-06);
    ASSERT_NEAR(quat1.getYaw(), IGN_PI_4, 1e-06);

    // wxyz
    ignition::math::Quaterniond ign2(0.707, 0.0, 0.0, 0.707);
    Quaternion quat2(ign2);
    ASSERT_NEAR(quat2.getX(), 0.0, 1e-06);
    ASSERT_NEAR(quat2.getY(), 0.0, 1e-06);
    ASSERT_NEAR(quat2.getZ(), 0.707, 1e-06);
    ASSERT_NEAR(quat2.getW(), 0.707, 1e-06);

    // RPY (single)
    Quaternion quat3r(-IGN_PI_4, 0.0, 0.0);
    ASSERT_NEAR(quat3r.getRoll(), -IGN_PI_4, 1e-06);
    ASSERT_NEAR(quat3r.getPitch(), 0.0, 1e-06);
    ASSERT_NEAR(quat3r.getYaw(), 0.0, 1e-06);

    Quaternion quat3p(0.0, -IGN_PI_4, 0.0);
    ASSERT_NEAR(quat3p.getRoll(), 0.0, 1e-06);
    ASSERT_NEAR(quat3p.getPitch(), -IGN_PI_4, 1e-06);
    ASSERT_NEAR(quat3p.getYaw(), 0.0, 1e-06);

    Quaternion quat3y(0.0, 0.0, -IGN_PI_4);
    ASSERT_NEAR(quat3y.getRoll(), 0.0, 1e-06);
    ASSERT_NEAR(quat3y.getPitch(), 0.0, 1e-06);
    ASSERT_NEAR(quat3y.getYaw(), -IGN_PI_4, 1e-06);

    // xyzw
    Quaternion quat4(0.0, 0.0, 0.707, 0.707);
    ASSERT_NEAR(quat4.getX(), 0.0, 1e-06);
    ASSERT_NEAR(quat4.getY(), 0.0, 1e-06);
    ASSERT_NEAR(quat4.getZ(), 0.707, 1e-06);
    ASSERT_NEAR(quat4.getW(), 0.707, 1e-06);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
