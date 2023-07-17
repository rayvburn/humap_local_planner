
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <humap_local_planner/utils/transformations.h>
#include <humap_local_planner/geometry/geometry.h>

#include "gtest_cout.h"

using namespace humap_local_planner;
using namespace humap_local_planner::geometry;

// local -> global velocity conversion does not change Z component of the vector
TEST(HumapVelocityConversions, computeVelocityGlobal) {
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

TEST(HumapVelocityConversions, computeTwist) {
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

TEST(HumapVelocityConversions, computeVelocityLocal) {
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

TEST(HumapVelocityConversions, DISABLED_computeVelocityLocalHolonomic) {
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

TEST(HumapVelocityConversions, adjustTwistWithAccLimits) {
	geometry::Vector vel_init(0.27, 0.0, 0.102);
	double acc_lim_x = 1.0;
	double acc_lim_y = 0.0;
	double acc_lim_th = 1.05;
	double vel_min_x = -0.1;
	double vel_min_y = 0.0;
	double vel_min_th = -1.05;
	double vel_max_x = 0.50;
	double vel_max_y = 0.00;
	double vel_max_th = +1.05;
	double sim_granularity = 0.25;

	// velocity setpoints
	std::vector<geometry::Vector> cmd_vel_loop{
		{0.00, 0.0, 0.40}, //
		{0.35, 0.0, 0.40}, //
		{0.35, 0.0, 0.40}, //
		{0.35, 0.0, 0.40}, //
		{0.00, 0.0, 0.00}, // % stop
		{0.00, 0.0, 0.00}, //
		{0.00, 0.0, 0.00}, //
		{0.50, 0.0, 0.00}, // % start
		{0.50, 0.0, 0.50}, // % turn
		{0.50, 0.0, 0.50}, //
		{0.50, 0.0, 0.50}, //
		{0.60, 0.0, 0.00}, //
		{0.60, 0.0, 0.00}, //
		{0.60, 0.0, 0.00}, //
		{0.00, 0.0, 0.00}, // % stop
		{0.00, 0.0, 0.00}, //
		{-0.1, 0.0, 1.00}, // % backward
		{-0.1, 0.0, 1.00}, //
		{-0.1, 0.0, 1.00}, //
		{-0.1, 0.0, 1.00}, //
		{0.00, 0.0, 0.00}, // % stop
		{0.00, 0.0, 0.00}, //
		{0.00, 0.0, 0.00}, //
		{0.00, 0.0, 0.00}, //
		{0.00, 0.0, 0.00}  //
	};

	std::vector<geometry::Vector> cmd_vel_loop_adjusted_maintain;
	cmd_vel_loop_adjusted_maintain.push_back(vel_init);

	for (const auto& cmd_vel: cmd_vel_loop) {
		geometry::Vector vel_curr = cmd_vel_loop_adjusted_maintain.back();
		geometry::Vector cmd_vel_mod = cmd_vel;
		adjustTwistWithAccLimits(
			vel_curr,
			acc_lim_x,
			acc_lim_y,
			acc_lim_th,
			vel_min_x,
			vel_min_y,
			vel_min_th,
			vel_max_x,
			vel_max_y,
			vel_max_th,
			sim_granularity,
			cmd_vel_mod,
			true // maintain
		);
		cmd_vel_loop_adjusted_maintain.push_back(cmd_vel_mod);
	}

	// verified with results from Matlab implementation placed at scripts/adjust_twist_with_acc_limits_test.m
	// `vel_loop` matrix is shown below
	std::vector<geometry::Vector> cmd_vel_results_maintain{
		{    0.2700,         0,    0.1020},
		{    0.0322,         0,    0.3645},
		{    0.2822,         0,    0.3924},
		{    0.3500,         0,    0.4000},
		{    0.3500,         0,    0.4000},
		{    0.1203,         0,    0.1375},
		{         0,         0,         0},
		{         0,         0,         0},
		{    0.2500,         0,         0},
		{    0.3812,         0,    0.2625},
		{    0.5000,         0,    0.5000},
		{    0.5000,         0,    0.5000},
		{    0.5000,         0,    0.5000},
		{    0.5000,         0,    0.5000},
		{    0.5000,         0,    0.5000},
		{    0.2500,         0,    0.2500},
		{         0,         0,         0},
		{   -0.0263,         0,    0.2625},
		{   -0.0525,         0,    0.5250},
		{   -0.0788,         0,    0.7875},
		{   -0.1000,         0,    1.0000},
		{   -0.0738,         0,    0.7375},
		{   -0.0475,         0,    0.4750},
		{   -0.0213,         0,    0.2125},
		{         0,         0,         0},
		{         0,         0,         0}
	};

	ASSERT_EQ(cmd_vel_loop_adjusted_maintain.size(), cmd_vel_results_maintain.size());
	for (unsigned int i = 0; i < cmd_vel_loop_adjusted_maintain.size(); i++) {
		EXPECT_NEAR(cmd_vel_loop_adjusted_maintain.at(i).getX(), cmd_vel_results_maintain.at(i).getX(), 1e-04);
		EXPECT_NEAR(cmd_vel_loop_adjusted_maintain.at(i).getY(), cmd_vel_results_maintain.at(i).getY(), 1e-04);
		EXPECT_NEAR(cmd_vel_loop_adjusted_maintain.at(i).getZ(), cmd_vel_results_maintain.at(i).getZ(), 1e-04);
	}

	// 2nd case
	std::vector<geometry::Vector> cmd_vel_loop_adjusted_trim;
	cmd_vel_loop_adjusted_trim.push_back(vel_init);

	for (const auto& cmd_vel: cmd_vel_loop) {
		geometry::Vector vel_curr = cmd_vel_loop_adjusted_trim.back();
		geometry::Vector cmd_vel_mod = cmd_vel;
		adjustTwistWithAccLimits(
			vel_curr,
			acc_lim_x,
			acc_lim_y,
			acc_lim_th,
			vel_min_x,
			vel_min_y,
			vel_min_th,
			vel_max_x,
			vel_max_y,
			vel_max_th,
			sim_granularity,
			cmd_vel_mod,
			false // trim
		);
		cmd_vel_loop_adjusted_trim.push_back(cmd_vel_mod);
	}

	// verified with results from Matlab implementation placed at scripts/adjust_twist_with_acc_limits_test.m
	// `vel_loop` matrix is shown below
	std::vector<geometry::Vector> cmd_vel_results_trim{
		{    0.2700,         0,    0.1020},
		{    0.0200,         0,    0.3645},
		{    0.2700,         0,    0.4000},
		{    0.3500,         0,    0.4000},
		{    0.3500,         0,    0.4000},
		{    0.1000,         0,    0.1375},
		{         0,         0,         0},
		{         0,         0,         0},
		{    0.2500,         0,         0},
		{    0.5000,         0,    0.2625},
		{    0.5000,         0,    0.5000},
		{    0.5000,         0,    0.5000},
		{    0.5000,         0,    0.2375},
		{    0.5000,         0,         0},
		{    0.5000,         0,         0},
		{    0.2500,         0,         0},
		{         0,         0,         0},
		{   -0.1000,         0,    0.2625},
		{   -0.1000,         0,    0.5250},
		{   -0.1000,         0,    0.7875},
		{   -0.1000,         0,    1.0000},
		{         0,         0,    0.7375},
		{         0,         0,    0.4750},
		{         0,         0,    0.2125},
		{         0,         0,         0},
		{         0,         0,         0}
	};

	ASSERT_EQ(cmd_vel_loop_adjusted_trim.size(), cmd_vel_results_trim.size());
	for (unsigned int i = 0; i < cmd_vel_loop_adjusted_trim.size(); i++) {
		EXPECT_NEAR(cmd_vel_loop_adjusted_trim.at(i).getX(), cmd_vel_results_trim.at(i).getX(), 1e-04);
		EXPECT_NEAR(cmd_vel_loop_adjusted_trim.at(i).getY(), cmd_vel_results_trim.at(i).getY(), 1e-04);
		EXPECT_NEAR(cmd_vel_loop_adjusted_trim.at(i).getZ(), cmd_vel_results_trim.at(i).getZ(), 1e-04);
	}
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
