#include <gtest/gtest.h>

// #include <ros/ros.h>
#include <hubero_local_planner/trajectory.h>
#include <people_msgs_utils/person.h>

using namespace hubero_local_planner;
using namespace hubero_local_planner::geometry;

// Simple: without angular velocity
TEST(HuberoTrajectory, trajectoryFromObject) {
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

	Trajectory t(p, 1.0, 5);

	ASSERT_EQ(t.getSteps(), 5);
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
	// 5th pose and vel
	ASSERT_DOUBLE_EQ(t.getPose(4).getX(), 4.0);
	ASSERT_DOUBLE_EQ(t.getPose(4).getY(), 4.0);
	ASSERT_DOUBLE_EQ(t.getPose(4).getYaw(), orientation);
	ASSERT_DOUBLE_EQ(t.getVelocity(4).getX(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(4).getY(), 1.0);
	ASSERT_DOUBLE_EQ(t.getVelocity(4).getZ(), 0.0);
}

TEST(HuberoTrajectory, trajectoryFromBaseLocalPlanner) {
	base_local_planner::Trajectory traj;
	traj.time_delta_ = 1.0;
	traj.xv_ = 1.0;
	traj.yv_ = -1.0;
	traj.thetav_ = 0.0;
	traj.cost_ = 0.0;

	// starting orientation
	double direction = std::atan2(traj.yv_, traj.xv_);

	// compute velocity in the global coordinates
	geometry::Vector vel_local(traj.xv_, traj.yv_, traj.thetav_);
	// starting pose, but only direction matters here
	geometry::Pose pose(0.0, 0.0, direction);
	// stores global velocity
	geometry::Vector vel_global;
	computeVelocityGlobal(vel_local, pose, vel_global);

	// find subsequent poses
	auto pose0 = geometry::Pose(0.0, 0.0, direction);
	auto pose1 = computeNextPose(pose0, vel_global, traj.time_delta_);
	auto pose2 = computeNextPose(pose1, vel_global, traj.time_delta_);
	auto pose3 = computeNextPose(pose2, vel_global, traj.time_delta_);
	traj.addPoint(pose0.getX(), pose0.getY(), pose0.getYaw());
	traj.addPoint(pose1.getX(), pose1.getY(), pose1.getYaw());
	traj.addPoint(pose2.getX(), pose2.getY(), pose2.getYaw());
	traj.addPoint(pose3.getX(), pose3.getY(), pose3.getYaw());

	Trajectory t(traj);
	ASSERT_DOUBLE_EQ(t.getTimeDelta(), traj.time_delta_);
	ASSERT_DOUBLE_EQ(t.getSteps(), 4);

	// initial pose and velocity
	ASSERT_DOUBLE_EQ(t.getPose(0).getX(), pose0.getX());
	ASSERT_DOUBLE_EQ(t.getPose(0).getY(), pose0.getY());
	ASSERT_DOUBLE_EQ(t.getPose(0).getYaw(), pose0.getYaw());
	ASSERT_DOUBLE_EQ(t.getVelocity(0).getX(), vel_global.getX());
	ASSERT_DOUBLE_EQ(t.getVelocity(0).getY(), vel_global.getY());
	ASSERT_DOUBLE_EQ(t.getVelocity(0).getZ(), vel_global.getZ());
	// 2nd pose and vel
	ASSERT_DOUBLE_EQ(t.getPose(1).getX(), pose1.getX());
	ASSERT_DOUBLE_EQ(t.getPose(1).getY(), pose1.getY());
	ASSERT_DOUBLE_EQ(t.getPose(1).getYaw(), pose1.getYaw());
	ASSERT_DOUBLE_EQ(t.getVelocity(1).getX(), vel_global.getX());
	ASSERT_DOUBLE_EQ(t.getVelocity(1).getY(), vel_global.getY());
	ASSERT_DOUBLE_EQ(t.getVelocity(1).getZ(), vel_global.getZ());
	// 3rd pose and vel
	ASSERT_DOUBLE_EQ(t.getPose(2).getX(), pose2.getX());
	ASSERT_DOUBLE_EQ(t.getPose(2).getY(), pose2.getY());
	ASSERT_DOUBLE_EQ(t.getPose(2).getYaw(), pose2.getYaw());
	ASSERT_DOUBLE_EQ(t.getVelocity(2).getX(), vel_global.getX());
	ASSERT_DOUBLE_EQ(t.getVelocity(2).getY(), vel_global.getY());
	ASSERT_DOUBLE_EQ(t.getVelocity(2).getZ(), vel_global.getZ());
	// 4th pose and vel
	ASSERT_DOUBLE_EQ(t.getPose(3).getX(), pose3.getX());
	ASSERT_DOUBLE_EQ(t.getPose(3).getY(), pose3.getY());
	ASSERT_DOUBLE_EQ(t.getPose(3).getYaw(), pose3.getYaw());
	ASSERT_DOUBLE_EQ(t.getVelocity(3).getX(), vel_global.getX());
	ASSERT_DOUBLE_EQ(t.getVelocity(3).getY(), vel_global.getY());
	ASSERT_DOUBLE_EQ(t.getVelocity(3).getZ(), vel_global.getZ());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
