
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <hubero_local_planner/utils/transformations.h>
#include <hubero_local_planner/geometry/geometry.h>

#include "gtest_cout.h"

using namespace hubero_local_planner;
using namespace hubero_local_planner::geometry;

// local -> global velocity conversion does not change Z component of the vector
TEST(HuberoVelocityConversions, computeVelocityGlobal) {
	Vector vel_local(0.15, 0.0, 0.25);
	Angle yaw(0.0);
	Angle expected_global_dir;

	// 1st case, robot orientation Euler Z 0 deg
	Pose pose(0.0, 0.0, yaw.getRadian());

	Vector vel_global;
	computeVelocityGlobal(vel_local, pose, vel_global);
	// GTEST_COUT << "1)          Pose2D: x " << pose.getX()  		<< " y " << pose.getY() 	  << " yaw " << pose.getYaw() << " pitch " << pose.getPitch() << " roll " << pose.getRoll() << std::endl;
	// GTEST_COUT << "1) Local  Velocity: x " << vel_local.getX()  << " y " << vel_local.getY()  << " yaw " << vel_local.getZ()  << std::endl;
	// GTEST_COUT << "1) Global Velocity: x " << vel_global.getX() << " y " << vel_global.getY() << " yaw " << vel_global.getZ() << std::endl;
	// provides the next ASSERT to make any sense
	ASSERT_DOUBLE_EQ(pose.getYaw(), yaw.getRadian());
	expected_global_dir = Angle(vel_local.calculateDirection() + yaw);
	ASSERT_DOUBLE_EQ(vel_global.calculateDirection().getRadian(), expected_global_dir.getRadian());

	// 2nd case: robot orientation Euler, Z 45 deg
	yaw = Angle(IGN_PI_4);
	pose.setOrientation(0.0, 0.0, yaw.getRadian());
	computeVelocityGlobal(vel_local, pose, vel_global);
	// GTEST_COUT << "2)          Pose2D: x " << pose.getX()  		<< " y " << pose.getY() 	  << " yaw " << pose.getYaw() << " pitch " << pose.getPitch() << " roll " << pose.getRoll() << std::endl;
	// GTEST_COUT << "2) Local  Velocity: x " << vel_local.getX()  << " y " << vel_local.getY()  << " yaw " << vel_local.getZ()  << std::endl;
	// GTEST_COUT << "2) Global Velocity: x " << vel_global.getX() << " y " << vel_global.getY() << " yaw " << vel_global.getZ() << std::endl;
	// provides the next ASSERT to make any sense
	ASSERT_DOUBLE_EQ(pose.getYaw(), yaw.getRadian());
	expected_global_dir = Angle(vel_local.calculateDirection() + yaw);
	ASSERT_DOUBLE_EQ(vel_global.calculateDirection().getRadian(), expected_global_dir.getRadian());
	// TODO: extra check

	// 3rd case: robot orientation Euler, Z -90 deg
	yaw = Angle(-IGN_PI_2);
	pose.setOrientation(0.0, 0.0, yaw.getRadian());
	computeVelocityGlobal(vel_local, pose, vel_global);
	// GTEST_COUT << "3)          Pose2D: x " << pose.getX()  		<< " y " << pose.getY() 	  << " yaw " << pose.getYaw() << " pitch " << pose.getPitch() << " roll " << pose.getRoll() << std::endl;
	// GTEST_COUT << "3) Local  Velocity: x " << vel_local.getX()  << " y " << vel_local.getY()  << " yaw " << vel_local.getZ()  << std::endl;
	// GTEST_COUT << "3) Global Velocity: x " << vel_global.getX() << " y " << vel_global.getY() << " yaw " << vel_global.getZ() << std::endl;
	// provides the next ASSERT to make any sense
	ASSERT_DOUBLE_EQ(pose.getYaw(), yaw.getRadian());
	expected_global_dir = Angle(vel_local.calculateDirection() + yaw);
	ASSERT_DOUBLE_EQ(vel_global.calculateDirection().getRadian(), expected_global_dir.getRadian());

	// 4th case: robot orientation Euler, Z +90, local.angular.z negative
	yaw = Angle(IGN_PI_2);
	pose.setOrientation(0.0, 0.0, yaw.getRadian());
	computeVelocityGlobal(vel_local, pose, vel_global);
	// GTEST_COUT << "4)          Pose2D: x " << pose.getX()  		<< " y " << pose.getY() 	  << " yaw " << pose.getYaw() << " pitch " << pose.getPitch() << " roll " << pose.getRoll() << std::endl;
	// GTEST_COUT << "4) Local  Velocity: x " << vel_local.getX()  << " y " << vel_local.getY()  << " z " << vel_local.getZ()  << std::endl;
	// GTEST_COUT << "4) Global Velocity: x " << vel_global.getX() << " y " << vel_global.getY() << " z " << vel_global.getZ() << std::endl;
	// provides the next ASSERT to make any sense
	ASSERT_DOUBLE_EQ(pose.getYaw(), yaw.getRadian());
	expected_global_dir = Angle(vel_local.calculateDirection() + yaw);
	ASSERT_DOUBLE_EQ(vel_global.calculateDirection().getRadian(), expected_global_dir.getRadian());
}

TEST(HuberoVelocityConversions, computeTwist) {
	// parameters
	const double SIM_PERIOD = 1.0; 	// time delta between acceleration calculation
	const double ROBOT_MASS = 1.0; 	// kg
	const double MIN_VEL_X = 0.0; 	// m/s
	const double MAX_VEL_X = 1.50; 	// m/s
	const double MAX_ROT_VEL = 2.0;	// rad/sec
	const double TWIST_ROT_COMPENSATION = 0.0;

	Pose pose(0.0, 0.0, 0.0); // input, constant among cases
	Vector force; 			  // input
	Vector vel_local;		  // helper
	Vector vel_global;		  // input
	Vector cmd_vel;			  // output

	// 1st case, robot orientation Euler Z 0 deg, no force
	vel_local = Vector(1.0, 0.0, 0.0);
	// no force - no movement (note that force does not sum up throughout simulation - it's not complaint with physical laws, but we simplify things there)
	force = Vector(0.0, 0.0, 0.0);

	computeVelocityGlobal(vel_local, pose, vel_global);
	computeTwist(pose, force, vel_global, SIM_PERIOD, ROBOT_MASS, MIN_VEL_X, MAX_VEL_X, MAX_ROT_VEL, TWIST_ROT_COMPENSATION, cmd_vel);
	// GTEST_COUT << "1) Global Velocity: x " << vel_global.getX() << " y " << vel_global.getY() << " z " << vel_global.getZ() << std::endl;
	// GTEST_COUT << "1) Local  Velocity: x " << cmd_vel.getX()    << " y " << cmd_vel.getY()    << " z " << cmd_vel.getZ()    << std::endl;
	// force made robot maintain its speed
	EXPECT_DOUBLE_EQ(cmd_vel.getX(), 0.0);
	EXPECT_DOUBLE_EQ(cmd_vel.getY(), 0.0);
	EXPECT_DOUBLE_EQ(cmd_vel.getZ(), 0.0);

	// 2nd case, robot orientation Euler Z 0 deg, force along robot's front
	vel_local = Vector(1.0, 0.0, 0.0);
	force = Vector(1.0, 0.0, 0.0);

	computeVelocityGlobal(vel_local, pose, vel_global);
	computeTwist(pose, force, vel_global, SIM_PERIOD, ROBOT_MASS, MIN_VEL_X, MAX_VEL_X, MAX_ROT_VEL, TWIST_ROT_COMPENSATION, cmd_vel);
	// GTEST_COUT << "2) Global Velocity: x " << vel_global.getX() << " y " << vel_global.getY() << " z " << vel_global.getZ() << std::endl;
	// GTEST_COUT << "2) Local  Velocity: x " << cmd_vel.getX()  << " y " << cmd_vel.getY()  << " z " << cmd_vel.getZ()  << std::endl;
	EXPECT_DOUBLE_EQ(cmd_vel.getX(), (force.getX() / ROBOT_MASS) / SIM_PERIOD);
	EXPECT_DOUBLE_EQ(cmd_vel.getY(), 0.0);
	EXPECT_DOUBLE_EQ(cmd_vel.getZ(), 0.0);

	// 3rd case, robot orientation Euler Z 0 deg, force pointing to robot's front-left
	vel_local = Vector(1.0, 0.0, 0.0);
	force = Vector(-0.5, -0.5, 0.0);

	computeVelocityGlobal(vel_local, pose, vel_global);
	computeTwist(pose, force, vel_global, SIM_PERIOD, ROBOT_MASS, MIN_VEL_X, MAX_VEL_X, MAX_ROT_VEL, TWIST_ROT_COMPENSATION, cmd_vel);
	// GTEST_COUT << "3) Global Velocity: x " << vel_global.getX() << " y " << vel_global.getY() << " z " << vel_global.getZ() << std::endl;
	// GTEST_COUT << "3) Local  Velocity: x " << cmd_vel.getX()  << " y " << cmd_vel.getY()  << " z " << cmd_vel.getZ()  << std::endl;
	// force made robot turn in place
	EXPECT_DOUBLE_EQ(cmd_vel.getX(), 0.0);
	EXPECT_DOUBLE_EQ(cmd_vel.getY(), 0.0);
	// X points forward from the robot
	// force toward back-right of the robot -> rotation towards right must be applied
	EXPECT_LT(cmd_vel.getZ(), 0.0);
}

TEST(HuberoVelocityConversions, computeVelocityLocal) {
	Vector vel_local;

	// 1) going straight ahead along X axis
	computeVelocityLocal(
		Vector( 1.0,  0.0,  0.0      ), // vel_global
		Pose  ( NAN,  NAN,  0.0      ), // pose
		vel_local
	);
	EXPECT_DOUBLE_EQ(vel_local.getX(), 1.0);
	EXPECT_DOUBLE_EQ(vel_local.getY(), 0.0);
	EXPECT_DOUBLE_EQ(vel_local.getZ(), 0.0);

	// 2) going straight ahead along Y axis
	computeVelocityLocal(
		Vector( 0.0,  1.0,  0.0      ), // vel_global
		Pose  ( NAN,  NAN,  IGN_PI_2 ), // pose
		vel_local
	);
	EXPECT_DOUBLE_EQ(vel_local.getX(), 1.0);
	EXPECT_DOUBLE_EQ(vel_local.getY(), 0.0);
	EXPECT_DOUBLE_EQ(vel_local.getZ(), 0.0);

	// 3) going straight aback Y axis
	computeVelocityLocal(
		Vector( 0.0, -1.0,  0.0      ), // vel_global
		Pose  ( NAN,  NAN, -IGN_PI_2 ), // pose
		vel_local
	);
	EXPECT_DOUBLE_EQ(vel_local.getX(), 1.0);
	EXPECT_DOUBLE_EQ(vel_local.getY(), 0.0);
	EXPECT_DOUBLE_EQ(vel_local.getZ(), 0.0);

	// 4) going straight aback X axis
	computeVelocityLocal(
		Vector(-1.0,  0.0,  0.0      ), // vel_global
		Pose  ( NAN,  NAN,  IGN_PI   ), // pose
		vel_local
	);
	EXPECT_DOUBLE_EQ(vel_local.getX(), 1.0);
	EXPECT_DOUBLE_EQ(vel_local.getY(), 0.0);
	EXPECT_DOUBLE_EQ(vel_local.getZ(), 0.0);

	// 5) backwards aback X axis
	computeVelocityLocal(
		Vector(-1.0,  0.0,  0.0      ), // vel_global
		Pose  ( NAN,  NAN,  0.0      ), // pose
		vel_local
	);
	EXPECT_DOUBLE_EQ(vel_local.getX(), -1.0);
	EXPECT_DOUBLE_EQ(vel_local.getY(), 0.0);
	EXPECT_DOUBLE_EQ(vel_local.getZ(), 0.0);

	// 6) going with sqrt(2) at 45 degrees globally
	computeVelocityLocal(
		Vector( 1.0,  1.0,  0.0      ), // vel_global
		Pose  ( NAN,  NAN,  IGN_PI_4 ), // pose
		vel_local
	);
	EXPECT_DOUBLE_EQ(vel_local.getX(), std::sqrt(1.0 + 1.0));
	EXPECT_DOUBLE_EQ(vel_local.getY(), 0.0);
	EXPECT_DOUBLE_EQ(vel_local.getZ(), 0.0);

	// 7) fully backwards
	computeVelocityLocal(
		Vector(-1.0, -1.0,  0.0      ), // vel_global
		Pose  ( NAN,  NAN,  IGN_PI_4 ), // pose
		vel_local
	);
	EXPECT_DOUBLE_EQ(vel_local.getX(), -std::sqrt(1.0 + 1.0));
	EXPECT_DOUBLE_EQ(vel_local.getY(), 0.0);
	EXPECT_DOUBLE_EQ(vel_local.getZ(), 0.0);

	// 8)
	computeVelocityLocal(
		Vector(-1.0, -1.0, -IGN_PI_2       ), // vel_global
		Pose  ( NAN,  NAN, -3.0 * IGN_PI_4 ), // pose
		vel_local
	);
	EXPECT_DOUBLE_EQ(vel_local.getX(), std::sqrt(1.0 + 1.0));
	EXPECT_DOUBLE_EQ(vel_local.getY(), 0.0);
	EXPECT_DOUBLE_EQ(vel_local.getZ(), -IGN_PI_2);
}

TEST(HuberoVelocityConversions, DISABLED_computeVelocityLocalHolonomic) {
	Vector vel_local;

	// TODO: formulation for a holonomic-drives is needed
	computeVelocityLocal(
		Vector( 1.0,  1.0,  IGN_PI_2 ), // vel_global
		Pose  ( NAN,  NAN,  0.0      ), // pose
		vel_local
	);
	computeVelocityLocal(
		Vector(-1.0, -1.0,  IGN_PI_2 ), // vel_global
		Pose  ( NAN,  NAN,  0.0      ), // pose
		vel_local
	);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
