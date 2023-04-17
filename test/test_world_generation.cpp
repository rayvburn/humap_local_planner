#include <gtest/gtest.h>
#include <ros/ros.h>
#include <hubero_local_planner/geometry/geometry.h>
#include <hubero_local_planner/world.h>

#include "gtest_cout.h"

using namespace hubero_local_planner;
using namespace hubero_local_planner::geometry;

// parts of these tests are used in `test_sfm.cpp`

// to make sure that distance to goal is computed from robot's center to the goal
TEST(HuberoWorldGeneration, robotTargetDistance) {
    // 1st case
    const Pose ROBOT_POSE(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Pose GOAL_LOCAL(2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Pose GOAL_GLOBAL(3.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    World world = World(ROBOT_POSE, Vector(), GOAL_LOCAL, GOAL_GLOBAL);
    Robot robot = world.getRobotData();

    EXPECT_EQ(robot.target.dist, (GOAL_LOCAL.getX() - ROBOT_POSE.getX()));
    EXPECT_EQ(robot.goal.dist, (GOAL_GLOBAL.getX() - ROBOT_POSE.getX()));
    EXPECT_FLOAT_EQ(0, Angle(robot.target.dist_v).getRadian());

    // 2nd case
    const Pose ROBOT_POSE2(0.0, 0.0, 0.0, 0.0, 0.0, IGN_DTOR(90.0));
    const Pose GOAL_LOCAL2(5.0, 5.0, 0.0, 0.0, 0.0, 0.0);
    const Pose GOAL_GLOBAL2(10.0, 10.0, 0.0, 0.0, 0.0, IGN_DTOR(180.0));

    world = World(ROBOT_POSE2, Vector(), GOAL_LOCAL2, GOAL_GLOBAL2);
    robot = world.getRobotData();

    double norm = std::sqrt(
        std::pow(GOAL_LOCAL2.getX() - ROBOT_POSE2.getX(), 2)
        + std::pow(GOAL_LOCAL2.getY() - ROBOT_POSE2.getY(), 2)
    );
    EXPECT_EQ(robot.target.dist, norm);
    norm = std::sqrt(
        std::pow(GOAL_GLOBAL2.getX() - ROBOT_POSE2.getX(), 2)
        + std::pow(GOAL_GLOBAL2.getY() - ROBOT_POSE2.getY(), 2)
    );
    EXPECT_EQ(robot.goal.dist, norm);
    EXPECT_FLOAT_EQ(IGN_DTOR(45.0), Angle(robot.target.dist_v).getRadian());
}

TEST(HuberoWorldGeneration, predict) {
    const Pose ROBOT_POSE(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Pose GOAL_LOCAL(2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Pose GOAL_GLOBAL(3.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Pose OBSTACLE_POSE = GOAL_LOCAL;

    const Vector ROBOT_VEL(0.5, 0.0, 0.0);
    const Vector OBSTACLE_VEL(-0.25, 0.0, 0.0);

    World world = World(ROBOT_POSE, ROBOT_VEL, GOAL_LOCAL, GOAL_GLOBAL);
    // point obstacle, robot also represented as a point
    world.addObstacle(ROBOT_POSE, OBSTACLE_POSE, OBSTACLE_VEL);
    world.predict(ROBOT_VEL, 1.0);

    EXPECT_EQ(world.getRobotData().centroid.getX(), ROBOT_POSE.getX() + ROBOT_VEL.getX());
    EXPECT_EQ(world.getRobotData().centroid.getY(), ROBOT_POSE.getY() + ROBOT_VEL.getY());
    EXPECT_EQ(world.getRobotData().centroid.getZ(), ROBOT_POSE.getZ() + ROBOT_VEL.getZ());

    ASSERT_EQ(world.getDynamicObjectsData().size(), 1);
    auto obstacle = world.getDynamicObjectsData().at(0);
    EXPECT_EQ(obstacle.object.getX(), OBSTACLE_POSE.getX() + OBSTACLE_VEL.getX());
    EXPECT_EQ(obstacle.object.getY(), OBSTACLE_POSE.getY() + OBSTACLE_VEL.getY());
    EXPECT_EQ(obstacle.object.getZ(), OBSTACLE_POSE.getZ() + OBSTACLE_VEL.getZ());
}

TEST(HuberoWorldGeneration, predictSequence) {
    const Pose ROBOT_POSE(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Pose GOAL_LOCAL(2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Pose GOAL_GLOBAL(3.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Pose OBSTACLE_POSE = GOAL_LOCAL;

    const Vector ROBOT_VEL(0.5, 0.0, 0.0);
    const Vector OBSTACLE_VEL(-0.25, 0.0, 0.0);

    World world = World(ROBOT_POSE, ROBOT_VEL, GOAL_LOCAL, GOAL_GLOBAL);
    // point obstacle, robot also represented as a point
    world.addObstacle(ROBOT_POSE, OBSTACLE_POSE, OBSTACLE_VEL);

    base_local_planner::Trajectory traj_orig;
    traj_orig.time_delta_ = 0.1;
    traj_orig.xv_ = ROBOT_VEL.getX() + 0.1;
    traj_orig.yv_ = ROBOT_VEL.getY() - 0.1;
    traj_orig.thetav_ = ROBOT_VEL.getZ();
    // initial point - should be the same as the one passed to the World ctor
    traj_orig.addPoint(ROBOT_POSE.getX(), ROBOT_POSE.getY(), ROBOT_POSE.getYaw());
    // poses predicted "by hand"
    const geometry::Pose ROBOT_POSE2(
        ROBOT_POSE.getX() + traj_orig.time_delta_ * traj_orig.xv_,
        ROBOT_POSE.getY() + traj_orig.time_delta_ * traj_orig.yv_,
        ROBOT_POSE.getYaw()
    );
    traj_orig.addPoint(ROBOT_POSE2.getX(), ROBOT_POSE2.getY(), ROBOT_POSE2.getYaw());
    const geometry::Pose ROBOT_POSE3(
        ROBOT_POSE2.getX() + traj_orig.time_delta_ * traj_orig.xv_,
        ROBOT_POSE2.getY() + traj_orig.time_delta_ * traj_orig.yv_,
        ROBOT_POSE2.getYaw()
    );
    traj_orig.addPoint(ROBOT_POSE3.getX(), ROBOT_POSE3.getY(), ROBOT_POSE3.getYaw());

    // create a trajectory
    Trajectory traj(traj_orig);

    auto world_states = world.predict(traj);
    ASSERT_EQ(traj.getVelocities().size(), 3);
    ASSERT_EQ(world_states.size(), 3);
    ASSERT_EQ(world_states.at(0).getRobotData().centroid.getX(), ROBOT_POSE.getX());
    ASSERT_EQ(world_states.at(1).getRobotData().centroid.getX(), ROBOT_POSE2.getX());
    ASSERT_EQ(world_states.at(1).getRobotData().centroid.getY(), ROBOT_POSE2.getY());
    ASSERT_EQ(world_states.at(2).getRobotData().centroid.getX(), ROBOT_POSE3.getX());
    ASSERT_EQ(world_states.at(2).getRobotData().centroid.getY(), ROBOT_POSE3.getY());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
