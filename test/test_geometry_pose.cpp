#include <gtest/gtest.h>
#include "gtest_cout.h"

#include <hubero_local_planner/geometry/geometry.h>

using namespace hubero::geometry;

TEST(HuberoGeometryPose, ctor) {
	// default
	Pose pose_def;
	ASSERT_DOUBLE_EQ(pose_def.getX(), 0.0);
	ASSERT_DOUBLE_EQ(pose_def.getY(), 0.0);
	ASSERT_DOUBLE_EQ(pose_def.getZ(), 0.0);
	ASSERT_DOUBLE_EQ(pose_def.getQuaternionX(), 0.0);
	ASSERT_DOUBLE_EQ(pose_def.getQuaternionY(), 0.0);
	ASSERT_DOUBLE_EQ(pose_def.getQuaternionZ(), 0.0);
	ASSERT_DOUBLE_EQ(pose_def.getQuaternionW(), 1.0);
	ASSERT_DOUBLE_EQ(pose_def.getRoll(), 0.0);
	ASSERT_DOUBLE_EQ(pose_def.getPitch(), 0.0);
	ASSERT_DOUBLE_EQ(pose_def.getYaw(), 0.0);

	// pose 2D
	Pose pose1(10.0, 5.0, IGN_PI_2);
	ASSERT_DOUBLE_EQ(pose1.getX(), 10.0);
	ASSERT_DOUBLE_EQ(pose1.getY(), 5.0);
	ASSERT_DOUBLE_EQ(pose1.getZ(), 0.0);
	ASSERT_DOUBLE_EQ(pose1.getRoll(), 0.0);
	ASSERT_DOUBLE_EQ(pose1.getPitch(), 0.0);
	ASSERT_DOUBLE_EQ(pose1.getYaw(), IGN_PI_2);

	// IGN pose 3D
	ignition::math::Vector3d pose2_pos(-10.0, -5.0, 100.0);
	ignition::math::Quaterniond pose2_quat;
	// target values for each case
	double roll = IGN_PI_4;
	double pitch = IGN_PI_2;
	double yaw = IGN_PI;

	ignition::math::Vector3d pose2_euler_roll(roll, 0.0, 0.0);
	pose2_quat.Euler(pose2_euler_roll);
	Pose pose2_roll(pose2_pos, pose2_quat);
	ASSERT_DOUBLE_EQ(pose2_roll.getX(), -10.0);
	ASSERT_DOUBLE_EQ(pose2_roll.getY(), -5.0);
	ASSERT_DOUBLE_EQ(pose2_roll.getZ(), 100.0);
	ASSERT_DOUBLE_EQ(pose2_roll.getRoll(), roll);
	ASSERT_DOUBLE_EQ(pose2_roll.getPitch(), 0.0);
	ASSERT_DOUBLE_EQ(pose2_roll.getYaw(), 0.0);

	ignition::math::Vector3d pose2_euler_pitch(0.0, pitch, 0.0);
	pose2_quat.Euler(pose2_euler_pitch);
	Pose pose2_pitch(pose2_pos, pose2_quat);
	ASSERT_DOUBLE_EQ(pose2_pitch.getX(), -10.0);
	ASSERT_DOUBLE_EQ(pose2_pitch.getY(), -5.0);
	ASSERT_DOUBLE_EQ(pose2_pitch.getZ(), 100.0);
	ASSERT_DOUBLE_EQ(pose2_pitch.getRoll(), 0.0);
	ASSERT_DOUBLE_EQ(pose2_pitch.getPitch(), pitch);
	ASSERT_DOUBLE_EQ(pose2_pitch.getYaw(), 0.0);

	ignition::math::Vector3d pose2_euler_yaw(0.0, 0.0, yaw);
	pose2_quat.Euler(pose2_euler_yaw);
	Pose pose2_yaw(pose2_pos, pose2_quat);
	ASSERT_DOUBLE_EQ(pose2_yaw.getX(), -10.0);
	ASSERT_DOUBLE_EQ(pose2_yaw.getY(), -5.0);
	ASSERT_DOUBLE_EQ(pose2_yaw.getZ(), 100.0);
	ASSERT_DOUBLE_EQ(pose2_yaw.getRoll(), 0.0);
	ASSERT_DOUBLE_EQ(pose2_yaw.getPitch(), 0.0);
	ASSERT_DOUBLE_EQ(pose2_yaw.getYaw(), yaw);

	// x y z Rx Ry Rx Rw
	Pose pose3(-10.0, +5.0, -1.0, 0.0, 0.0, 0.7, 0.7);
	ASSERT_DOUBLE_EQ(pose3.getX(), -10.0);
	ASSERT_DOUBLE_EQ(pose3.getY(), +5.0);
	ASSERT_DOUBLE_EQ(pose3.getZ(), -1.0);
	ASSERT_DOUBLE_EQ(pose3.getQuaternionX(), 0.0);
	ASSERT_DOUBLE_EQ(pose3.getQuaternionY(), 0.0);
	ASSERT_DOUBLE_EQ(pose3.getQuaternionZ(), 0.7);
	ASSERT_DOUBLE_EQ(pose3.getQuaternionW(), 0.7);
}

TEST(HuberoGeometryPose, setters) {
	Pose pose1;
	pose1.setPosition(0.1, 0.2, 0.3);
	ASSERT_DOUBLE_EQ(pose1.getX(), 0.1);
	ASSERT_DOUBLE_EQ(pose1.getY(), 0.2);
	ASSERT_DOUBLE_EQ(pose1.getZ(), 0.3);

	Pose pose2;
	pose2.setPosition(Vector(0.3, 0.1, 0.2));
	ASSERT_DOUBLE_EQ(pose2.getX(), 0.3);
	ASSERT_DOUBLE_EQ(pose2.getY(), 0.1);
	ASSERT_DOUBLE_EQ(pose2.getZ(), 0.2);
	pose2.setPositionX(0.6);
	ASSERT_DOUBLE_EQ(pose2.getX(), 0.6);
	pose2.setPositionY(0.7);
	ASSERT_DOUBLE_EQ(pose2.getY(), 0.7);
	pose2.setPositionZ(0.8);
	ASSERT_DOUBLE_EQ(pose2.getZ(), 0.8);

	pose2.setOrientation(0.0, 0.0, IGN_PI);
	ASSERT_DOUBLE_EQ(pose2.getYaw(), IGN_PI);
}

TEST(HuberoGeometryPose, conversion) {
	Pose pose_ref(0.567, 1.234, 0.987, 0.0, 0.0, IGN_PI);

	geometry_msgs::Pose pose_geom_msg = pose_ref.getAsMsgPose();
	Pose pose_geom_msg_conv(pose_geom_msg);
	ASSERT_DOUBLE_EQ(pose_geom_msg_conv.getX(), 0.567);
	ASSERT_DOUBLE_EQ(pose_geom_msg_conv.getY(), 1.234);
	ASSERT_DOUBLE_EQ(pose_geom_msg_conv.getZ(), 0.987);
	ASSERT_DOUBLE_EQ(pose_geom_msg_conv.getRoll(), 0.0);
	ASSERT_DOUBLE_EQ(pose_geom_msg_conv.getPitch(), 0.0);
	ASSERT_DOUBLE_EQ(pose_geom_msg_conv.getYaw(), IGN_PI);
	ASSERT_EQ(pose_ref.getPose(), pose_geom_msg_conv.getPose());

	teb_local_planner::PoseSE2 pose_teb = pose_ref.getAsTebPose();
	Pose pose_teb_conv(pose_teb);
	ASSERT_DOUBLE_EQ(pose_teb_conv.getX(), 0.567);
	ASSERT_DOUBLE_EQ(pose_teb_conv.getY(), 1.234);
	ASSERT_DOUBLE_EQ(pose_teb_conv.getZ(), 0.0); // pose2d -> z = 0.0
	ASSERT_DOUBLE_EQ(pose_teb_conv.getRoll(), 0.0);
	ASSERT_DOUBLE_EQ(pose_teb_conv.getPitch(), 0.0);
	ASSERT_DOUBLE_EQ(pose_teb_conv.getYaw(), IGN_PI);
	// raw pose comparison does not make sense (.Z() situation)

	tf::Stamped<tf::Pose> pose_tf = pose_ref.getAsTfPose();
	Pose pose_tf_conv(pose_tf);
	ASSERT_DOUBLE_EQ(pose_tf_conv.getX(), 0.567);
	ASSERT_DOUBLE_EQ(pose_tf_conv.getY(), 1.234);
	ASSERT_DOUBLE_EQ(pose_tf_conv.getZ(), 0.987);
	ASSERT_DOUBLE_EQ(pose_tf_conv.getRoll(), 0.0);
	ASSERT_DOUBLE_EQ(pose_tf_conv.getPitch(), 0.0);
	ASSERT_DOUBLE_EQ(pose_tf_conv.getYaw(), IGN_PI);
	ASSERT_EQ(pose_ref.getPose(), pose_tf_conv.getPose());

	geometry_msgs::Transform pose_geom_tf = pose_ref.getAsTfTransform();
	Pose pose_geom_tf_conv(pose_geom_tf);
	ASSERT_DOUBLE_EQ(pose_geom_tf_conv.getX(), 0.567);
	ASSERT_DOUBLE_EQ(pose_geom_tf_conv.getY(), 1.234);
	ASSERT_DOUBLE_EQ(pose_geom_tf_conv.getZ(), 0.987);
	ASSERT_DOUBLE_EQ(pose_geom_tf_conv.getRoll(), 0.0);
	ASSERT_DOUBLE_EQ(pose_geom_tf_conv.getPitch(), 0.0);
	ASSERT_DOUBLE_EQ(pose_geom_tf_conv.getYaw(), IGN_PI);
	ASSERT_EQ(pose_ref.getPose(), pose_geom_tf_conv.getPose());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
