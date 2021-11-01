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

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
