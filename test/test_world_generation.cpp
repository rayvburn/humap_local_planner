#include <gtest/gtest.h>
#include <ros/ros.h>
#include <hubero_common/typedefs.h>
#include <hubero_local_planner/utils/converter.h>
#include <hubero_local_planner/sfm/world.h>

#include "gtest_cout.h"

using namespace hubero_local_planner;

// to make sure that distance to goal is computed from robot's center to the goal
TEST(HuberoWorldGeneration, robotTargetDistance) {
    // 1st case
    const Pose3 ROBOT_POSE(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Pose3 GOAL_LOCAL(2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const Pose3 GOAL_GLOBAL(3.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    sfm::World world = sfm::World(ROBOT_POSE, Vector3(), GOAL_LOCAL, GOAL_GLOBAL);
    sfm::Robot robot = world.getRobotData();

    EXPECT_EQ(robot.target.dist, (GOAL_LOCAL.Pos().X() - ROBOT_POSE.Pos().X()));
    EXPECT_EQ(robot.goal.dist, (GOAL_GLOBAL.Pos().X() - ROBOT_POSE.Pos().X()));
    EXPECT_FLOAT_EQ(0, std::atan2(robot.target.dist_v.Y(), robot.target.dist_v.X()));

    // 2nd case
    const Pose3 ROBOT_POSE2(0.0, 0.0, 0.0, 0.0, 0.0, IGN_DTOR(90.0));
    const Pose3 GOAL_LOCAL2(5.0, 5.0, 0.0, 0.0, 0.0, 0.0);
    const Pose3 GOAL_GLOBAL2(10.0, 10.0, 0.0, 0.0, 0.0, IGN_DTOR(180.0));

    world = sfm::World(ROBOT_POSE2, Vector3(), GOAL_LOCAL2, GOAL_GLOBAL2);
    robot = world.getRobotData();

    double norm = std::sqrt(
        std::pow(GOAL_LOCAL2.Pos().X() - ROBOT_POSE2.Pos().X(), 2)
        + std::pow(GOAL_LOCAL2.Pos().Y() - ROBOT_POSE2.Pos().Y(), 2)
    );
    EXPECT_EQ(robot.target.dist, norm);
    norm = std::sqrt(
        std::pow(GOAL_GLOBAL2.Pos().X() - ROBOT_POSE2.Pos().X(), 2)
        + std::pow(GOAL_GLOBAL2.Pos().Y() - ROBOT_POSE2.Pos().Y(), 2)
    );
    EXPECT_EQ(robot.goal.dist, norm);
    EXPECT_FLOAT_EQ(IGN_DTOR(45.0), std::atan2(robot.target.dist_v.Y(), robot.target.dist_v.X()));
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "HuberoWorldGeneration");
	ros::NodeHandle nh;
	ROSCONSOLE_AUTOINIT;
	ros::start();
	return RUN_ALL_TESTS();
}
