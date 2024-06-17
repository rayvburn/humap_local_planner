#include <gtest/gtest.h>

// #include <ros/ros.h>
#include <humap_local_planner/trajectory.h>
#include <people_msgs_utils/person.h>

using namespace humap_local_planner;
using namespace humap_local_planner::geometry;

// NOTE: all test cases verified with the Matlab script 'compute_next_pose_test.m'
// Some require manual changes
auto DIFF_STRICT = 1e-09;

// Simple: without angular velocity
TEST(HumapTrajectory, trajectoryFromObject) {
	geometry_msgs::Point pos;
	pos.x = 0.0;
	pos.y = 0.0;
	pos.z = 0.0;
	geometry_msgs::Point vel;
	vel.x = 1.0;
	vel.y = 1.0;
	vel.z = 0.0;
	double orientation = std::atan2(vel.y, vel.x);
	people_msgs_utils::Person p("john", pos, vel, 1.0, {}, {});

	// false -> keep velocities (available via getters) in the global coordinate system
	Trajectory t(p, 1.0, 5, false);

	ASSERT_EQ(t.getSteps(), 5);
	ASSERT_EQ(t.getPoses().size(), 5);
	ASSERT_EQ(t.getVelocities().size(), 4);

	// initial pose and velocity
	ASSERT_DOUBLE_EQ(t.getPose(0).getX(), 0.0);
	ASSERT_DOUBLE_EQ(t.getPose(0).getY(), 0.0);
	ASSERT_DOUBLE_EQ(t.getPose(0).getYaw(), orientation);
	ASSERT_DOUBLE_EQ(t.getVelocity(0).getX(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(0).getY(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(0).getZ(), 0.0);
	// 2nd pose and vel
	ASSERT_DOUBLE_EQ(t.getPose(1).getX(), 1.0);
	ASSERT_DOUBLE_EQ(t.getPose(1).getY(), 1.0);
	ASSERT_DOUBLE_EQ(t.getPose(1).getYaw(), orientation);
	ASSERT_DOUBLE_EQ(t.getVelocity(1).getX(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(1).getY(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(1).getZ(), 0.0);
	// 3rd pose and vel
	ASSERT_DOUBLE_EQ(t.getPose(2).getX(), 2.0);
	ASSERT_DOUBLE_EQ(t.getPose(2).getY(), 2.0);
	ASSERT_DOUBLE_EQ(t.getPose(2).getYaw(), orientation);
	ASSERT_DOUBLE_EQ(t.getVelocity(2).getX(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(2).getY(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(2).getZ(), 0.0);
	// 4th pose and vel
	ASSERT_DOUBLE_EQ(t.getPose(3).getX(), 3.0);
	ASSERT_DOUBLE_EQ(t.getPose(3).getY(), 3.0);
	ASSERT_DOUBLE_EQ(t.getPose(3).getYaw(), orientation);
	ASSERT_DOUBLE_EQ(t.getVelocity(3).getX(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(3).getY(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(3).getZ(), 0.0);
	// 5th pose
	ASSERT_DOUBLE_EQ(t.getPose(4).getX(), 4.0);
	ASSERT_DOUBLE_EQ(t.getPose(4).getY(), 4.0);
	ASSERT_DOUBLE_EQ(t.getPose(4).getYaw(), orientation);
}

// Simple: without angular velocity
TEST(HumapTrajectory, trajectoryFromObject2) {
	geometry_msgs::Point pos;
	pos.x = 0.0;
	pos.y = 0.0;
	pos.z = 0.0;
	geometry_msgs::Point vel;
	vel.x = 1.0;
	vel.y = -0.33;
	vel.z = 0.0;
	double orientation = std::atan2(vel.y, vel.x);
	people_msgs_utils::Person p("josh", pos, vel, 1.0, {}, {});

	// false -> keep velocities (available via getters) in the global coordinate system
	Trajectory t(p, 0.1, 5, false);

	ASSERT_EQ(t.getSteps(), 5);
	ASSERT_EQ(t.getPoses().size(), 5);
	ASSERT_EQ(t.getVelocities().size(), 4);

	// initial pose and velocity
	ASSERT_DOUBLE_EQ(t.getPose(0).getX(), 0.0);
	ASSERT_DOUBLE_EQ(t.getPose(0).getY(), 0.0);
	ASSERT_DOUBLE_EQ(t.getPose(0).getYaw(), orientation);
	ASSERT_DOUBLE_EQ(t.getVelocity(0).getX(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(0).getY(), -0.33);
	ASSERT_DOUBLE_EQ(t.getVelocity(0).getZ(), 0.0);
	// 2nd pose and vel
	ASSERT_DOUBLE_EQ(t.getPose(1).getX(), +0.1000);
	ASSERT_DOUBLE_EQ(t.getPose(1).getY(), -0.0330);
	ASSERT_DOUBLE_EQ(t.getPose(1).getYaw(), orientation);
	ASSERT_DOUBLE_EQ(t.getVelocity(1).getX(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(1).getY(), -0.33);
	ASSERT_DOUBLE_EQ(t.getVelocity(1).getZ(), 0.0);
	// 3rd pose and vel
	ASSERT_DOUBLE_EQ(t.getPose(2).getX(), +0.2000);
	ASSERT_DOUBLE_EQ(t.getPose(2).getY(), -0.0660);
	ASSERT_DOUBLE_EQ(t.getPose(2).getYaw(), orientation);
	ASSERT_DOUBLE_EQ(t.getVelocity(2).getX(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(2).getY(), -0.33);
	ASSERT_DOUBLE_EQ(t.getVelocity(2).getZ(), 0.0);
	// 4th pose and vel
	ASSERT_DOUBLE_EQ(t.getPose(3).getX(), +0.3000);
	ASSERT_DOUBLE_EQ(t.getPose(3).getY(), -0.0990);
	ASSERT_DOUBLE_EQ(t.getPose(3).getYaw(), orientation);
	ASSERT_DOUBLE_EQ(t.getVelocity(3).getX(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(3).getY(), -0.33);
	ASSERT_DOUBLE_EQ(t.getVelocity(3).getZ(), 0.0);
	// 5th pose
	ASSERT_DOUBLE_EQ(t.getPose(4).getX(), +0.4000);
	ASSERT_DOUBLE_EQ(t.getPose(4).getY(), -0.1320);
	ASSERT_DOUBLE_EQ(t.getPose(4).getYaw(), orientation);
}

TEST(HumapTrajectory, trajectoryFromBaseLocalPlanner) {
	base_local_planner::Trajectory traj;
	traj.time_delta_ = 0.25;
	traj.xv_ = 1.0;
	traj.yv_ = -1.0;
	traj.thetav_ = 0.0;
	traj.cost_ = 0.0;

	// starting orientation is aligned with the direction of the velocity vector: atan2(vy, vx)
	// add subsequent poses
	traj.addPoint(0, 0, -0.785398163397448);
	traj.addPoint(0, -0.353553390593274, -0.785398163397448);
	traj.addPoint(0, -0.707106781186548, -0.785398163397448);
	traj.addPoint(0, -1.06066017177982, -0.785398163397448);

	// true -> convert to the global coordinate system
	Trajectory t(traj, true);
	ASSERT_DOUBLE_EQ(t.getTimeDelta(), traj.time_delta_);
	ASSERT_EQ(t.getSteps(), 4);
	ASSERT_EQ(t.getPoses().size(), 4);
	ASSERT_EQ(t.getVelocities().size(), 3);

	// initial pose and velocity
	ASSERT_NEAR(t.getPose(0).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(0).getY(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(0).getYaw(), -0.785398163397448, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(0).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(0).getY(), -1.414213562373095, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(0).getZ(), 0.0000, DIFF_STRICT);
	// 2nd pose and vel
	ASSERT_NEAR(t.getPose(1).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(1).getY(), -0.353553390593274, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(1).getYaw(), -0.785398163397448, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(1).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(1).getY(), -1.414213562373095, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(1).getZ(), 0.0000, DIFF_STRICT);
	// 3rd pose and vel
	ASSERT_NEAR(t.getPose(2).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(2).getY(), -0.707106781186548, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(2).getYaw(), -0.785398163397448, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(2).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(2).getY(), -1.414213562373095, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(2).getZ(), 0.0000, DIFF_STRICT);
	// 4th pose
	ASSERT_NEAR(t.getPose(3).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(3).getY(), -1.060660171779821, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(3).getYaw(), -0.785398163397448, DIFF_STRICT);

	// the same but retrieve velocities in local coordinate system
	Trajectory t2(traj, false);
	ASSERT_DOUBLE_EQ(t2.getTimeDelta(), traj.time_delta_);
	ASSERT_EQ(t2.getSteps(), 4);
	ASSERT_EQ(t2.getPoses().size(), 4);
	ASSERT_EQ(t2.getVelocities().size(), 3);

	// initial pose and velocity
	ASSERT_NEAR(t2.getPose(0).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(0).getY(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(0).getYaw(), -0.785398163397448, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(0).getX(), 1.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(0).getY(), -1.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(0).getZ(), 0.0000, DIFF_STRICT);
	// 2nd pose and vel
	ASSERT_NEAR(t2.getPose(1).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(1).getY(), -0.353553390593274, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(1).getYaw(), -0.785398163397448, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(1).getX(), 1.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(1).getY(), -1.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(1).getZ(), 0.0000, DIFF_STRICT);
	// 3rd pose and vel
	ASSERT_NEAR(t2.getPose(2).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(2).getY(), -0.707106781186548, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(2).getYaw(), -0.785398163397448, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(2).getX(), 1.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(2).getY(), -1.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(2).getZ(), 0.0000, DIFF_STRICT);
	// 4th pose
	ASSERT_NEAR(t2.getPose(3).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(3).getY(), -1.060660171779821, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(3).getYaw(), -0.785398163397448, DIFF_STRICT);
}

TEST(HumapTrajectory, trajectoryAngularFromBaseLocalPlanner) {
	base_local_planner::Trajectory traj;
	traj.time_delta_ = 1.0;
	traj.xv_ = 1.0;
	traj.yv_ = 0.5;
	traj.thetav_ = -0.25;
	traj.cost_ = 0.0;

	// starting orientation is aligned with the direction of the velocity vector: atan2(vy, vx)
	traj.addPoint(0.0000, 0.0000, 0.463647609000806);
	traj.addPoint(0.670820393249937, 0.894427190999916, 0.213647609000806);
	traj.addPoint(1.542071433324901, 1.595085185436783, -0.036352390999194);
	traj.addPoint(2.559582950388867, 2.058410462798365, -0.286352390999194);
	traj.addPoint(3.660091006547741, 2.255595701417830, -0.536352390999194);

	// true -> convert to the global coordinate system
	Trajectory t(traj, true);
	ASSERT_DOUBLE_EQ(t.getTimeDelta(), traj.time_delta_);
	ASSERT_EQ(t.getSteps(), 5);
	ASSERT_EQ(t.getPoses().size(), 5);
	ASSERT_EQ(t.getVelocities().size(), 4);

	// // initial pose and velocity
	ASSERT_NEAR(t.getPose(0).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(0).getY(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(0).getYaw(), 0.463647609000806, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(0).getX(), 0.670820393249937, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(0).getY(), 0.894427190999916, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(0).getZ(), -0.250000000000000, DIFF_STRICT);
	// 2nd pose and vel
	ASSERT_NEAR(t.getPose(1).getX(), 0.670820393249937, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(1).getY(), 0.894427190999916, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(1).getYaw(), 0.213647609000806, DIFF_STRICT);
	EXPECT_NEAR(t.getVelocity(1).getX(), 0.871251040074964, DIFF_STRICT);
	EXPECT_NEAR(t.getVelocity(1).getY(), 0.700657994436868, DIFF_STRICT);
	EXPECT_NEAR(t.getVelocity(1).getZ(), -0.250000000000000, DIFF_STRICT);
	// 3rd pose and vel
	EXPECT_NEAR(t.getPose(2).getX(), 1.542071433324901, DIFF_STRICT);
	EXPECT_NEAR(t.getPose(2).getY(), 1.595085185436783, DIFF_STRICT);
	EXPECT_NEAR(t.getPose(2).getYaw(), -0.036352390999194, DIFF_STRICT);
	EXPECT_NEAR(t.getVelocity(2).getX(), 1.017511517063966, DIFF_STRICT);
	EXPECT_NEAR(t.getVelocity(2).getY(), 0.463325277361582, DIFF_STRICT);
	EXPECT_NEAR(t.getVelocity(2).getZ(), -0.250000000000000, DIFF_STRICT);
	// 4th pose and vel
	EXPECT_NEAR(t.getPose(3).getX(), 2.559582950388867, DIFF_STRICT);
	EXPECT_NEAR(t.getPose(3).getY(), 2.058410462798365, DIFF_STRICT);
	EXPECT_NEAR(t.getPose(3).getYaw(), -0.286352390999194, DIFF_STRICT);
	EXPECT_NEAR(t.getVelocity(3).getX(), 1.100508056158875, DIFF_STRICT);
	EXPECT_NEAR(t.getVelocity(3).getY(), 0.197185238619465, DIFF_STRICT);
	EXPECT_NEAR(t.getVelocity(3).getZ(), -0.250000000000000, DIFF_STRICT);
	// 5th pose
	EXPECT_NEAR(t.getPose(4).getX(), 3.660091006547741, DIFF_STRICT);
	EXPECT_NEAR(t.getPose(4).getY(), 2.255595701417830, DIFF_STRICT);
	EXPECT_NEAR(t.getPose(4).getYaw(), -0.536352390999194, DIFF_STRICT);

	// the same but retrieve velocities in local coordinate system
	Trajectory t2(traj, false);
	ASSERT_DOUBLE_EQ(t2.getTimeDelta(), traj.time_delta_);
	ASSERT_EQ(t2.getSteps(), 5);
	ASSERT_EQ(t2.getPoses().size(), 5);
	ASSERT_EQ(t2.getVelocities().size(), 4);

	// initial pose and velocity
	ASSERT_NEAR(t2.getPose(0).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(0).getY(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(0).getYaw(), 0.463647609000806, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(0).getX(), 1.0, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(0).getY(), 0.5, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(0).getZ(), -0.25, DIFF_STRICT);
	// 2nd pose and vel
	ASSERT_NEAR(t2.getPose(1).getX(), 0.670820393249937, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(1).getY(), 0.894427190999916, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(1).getYaw(), 0.213647609000806, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(1).getX(), 1.0, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(1).getY(), 0.5, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(1).getZ(), -0.25, DIFF_STRICT);
	// 3rd pose and vel
	ASSERT_NEAR(t2.getPose(2).getX(), 1.542071433324901, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(2).getY(), 1.595085185436783, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(2).getYaw(), -0.036352390999194, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(2).getX(), 1.0, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(2).getY(), 0.5, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(2).getZ(), -0.25, DIFF_STRICT);
	// 4th pose and vel
	EXPECT_NEAR(t2.getPose(3).getX(), 2.559582950388867, DIFF_STRICT);
	EXPECT_NEAR(t2.getPose(3).getY(), 2.058410462798365, DIFF_STRICT);
	EXPECT_NEAR(t2.getPose(3).getYaw(), -0.286352390999194, DIFF_STRICT);
	EXPECT_NEAR(t2.getVelocity(3).getX(), 1.0, DIFF_STRICT);
	EXPECT_NEAR(t2.getVelocity(3).getY(), 0.5, DIFF_STRICT);
	EXPECT_NEAR(t2.getVelocity(3).getZ(), -0.25, DIFF_STRICT);
	// 5th pose
	EXPECT_NEAR(t2.getPose(4).getX(), 3.660091006547741, DIFF_STRICT);
	EXPECT_NEAR(t2.getPose(4).getY(), 2.255595701417830, DIFF_STRICT);
	EXPECT_NEAR(t2.getPose(4).getYaw(), -0.536352390999194, DIFF_STRICT);
}

TEST(HumapTrajectory, trajectoryNonholonomicFromBaseLocalPlanner) {
	base_local_planner::Trajectory traj;
	traj.time_delta_ = 0.2;
	traj.xv_ = 1.0;
	traj.yv_ = 0.0;
	traj.thetav_ = 1.0;
	traj.cost_ = 0.0;

	// starting orientation is aligned with the direction of the velocity vector: atan2(vy, vx)
	traj.addPoint(0.0000, 0.0000, 0.0);
	traj.addPoint(0.200000000000000, 0.0000, 0.200000000000000);
	traj.addPoint(0.396013315568248, 0.039733866159012, 0.400000000000000);
	traj.addPoint(0.580225514368825, 0.117617534620742, 0.600000000000000);

	// true -> convert to the global coordinate system
	Trajectory t(traj, true);
	ASSERT_DOUBLE_EQ(t.getTimeDelta(), traj.time_delta_);
	ASSERT_EQ(t.getSteps(), 4);
	ASSERT_EQ(t.getPoses().size(), 4);
	ASSERT_EQ(t.getVelocities().size(), 3);

	// initial pose and velocity
	ASSERT_NEAR(t.getPose(0).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(0).getY(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(0).getYaw(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(0).getX(), 1.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(0).getY(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(0).getZ(), 1.0000, DIFF_STRICT);
	// 2nd pose and vel
	ASSERT_NEAR(t.getPose(1).getX(), 0.200000000000000, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(1).getY(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(1).getYaw(), 0.200000000000000, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(1).getX(), 0.980066577841242, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(1).getY(), 0.198669330795061, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(1).getZ(), 1.0000, DIFF_STRICT);
	// 3rd pose and vel
	ASSERT_NEAR(t.getPose(2).getX(), 0.396013315568248, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(2).getY(), 0.039733866159012, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(2).getYaw(), 0.400000000000000, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(2).getX(), 0.921060994002885, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(2).getY(), 0.389418342308651, DIFF_STRICT);
	ASSERT_NEAR(t.getVelocity(2).getZ(), 1.0000, DIFF_STRICT);
	// 4th pose
	ASSERT_NEAR(t.getPose(3).getX(), 0.580225514368825, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(3).getY(), 0.117617534620742, DIFF_STRICT);
	ASSERT_NEAR(t.getPose(3).getYaw(), 0.600000000000000, DIFF_STRICT);

	// the same but retrieve velocities in local coordinate system
	Trajectory t2(traj, false);
	ASSERT_DOUBLE_EQ(t2.getTimeDelta(), traj.time_delta_);
	ASSERT_EQ(t2.getSteps(), 4);
	ASSERT_EQ(t2.getPoses().size(), 4);
	ASSERT_EQ(t2.getVelocities().size(), 3);

	// initial pose and velocity
	ASSERT_NEAR(t2.getPose(0).getX(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(0).getY(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(0).getYaw(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(0).getX(), 1.0, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(0).getY(), 0.0, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(0).getZ(), 1.0, DIFF_STRICT);
	// 2nd pose and vel
	ASSERT_NEAR(t2.getPose(1).getX(), 0.200000000000000, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(1).getY(), 0.0000, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(1).getYaw(), 0.200000000000000, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(1).getX(), 1.0, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(1).getY(), 0.0, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(1).getZ(), 1.0, DIFF_STRICT);
	// 3rd pose and vel
	ASSERT_NEAR(t2.getPose(2).getX(), 0.396013315568248, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(2).getY(), 0.039733866159012, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(2).getYaw(), 0.400000000000000, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(2).getX(), 1.0, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(2).getY(), 0.0, DIFF_STRICT);
	ASSERT_NEAR(t2.getVelocity(2).getZ(), 1.0, DIFF_STRICT);
	// 4th pose
	ASSERT_NEAR(t2.getPose(3).getX(), 0.580225514368825, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(3).getY(), 0.117617534620742, DIFF_STRICT);
	ASSERT_NEAR(t2.getPose(3).getYaw(), 0.600000000000000, DIFF_STRICT);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
