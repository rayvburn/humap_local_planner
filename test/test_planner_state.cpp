#include <gtest/gtest.h>
#include <hubero_local_planner/planner_state.h>
#include <angles/angles.h>

using namespace hubero_local_planner;

geometry::Pose g_pose;
geometry::Pose g_goal;
geometry::Pose g_goal_local;
geometry::Pose g_goal_initiation;

bool isPositionReached() {
	return (g_goal.getPosition() - g_pose.getPosition()).calculateLength() < 0.1;
}

bool isGoalReached() {
	return isPositionReached()
		&& angles::shortest_angular_distance(g_pose.getYaw(), g_goal.getYaw()) < 0.3;
}

TEST(HuberoPlannerState, initial) {
	g_pose = geometry::Pose(0.0, 0.0, 0.0);
	g_goal = geometry::Pose(0.0, 0.0, 0.0);
	g_goal_local = geometry::Pose(0.0, 0.0, 0.0);
	g_goal_initiation = geometry::Pose(0.0, 0.0, 0.0);
	PlannerState state(g_pose, g_goal, g_goal_local, g_goal_initiation, isPositionReached, isGoalReached);
	EXPECT_EQ(state.getState(), PlannerState::STOPPED);
}

TEST(HuberoPlannerState, goalRequiresRotationAtTheStart) {
	g_pose = geometry::Pose(0.0, 0.0, IGN_DTOR(45));
	g_goal = geometry::Pose(-10.0, -10.0, 0.0);
	g_goal_local = geometry::Pose(-2.0, -2.0, 0.0);
	g_goal_initiation = geometry::Pose(-0.5, -0.5, 0.0);
	PlannerState state(g_pose, g_goal, g_goal_local, g_goal_initiation, isPositionReached, isGoalReached);
	state.update(true); // newGoalReceived
	EXPECT_FALSE(state.isGoalReached());
	EXPECT_EQ(state.getState(), PlannerState::INITIATE_EXECUTION);

	// rotated but not enough
	g_pose = geometry::Pose(0.0, 0.0, IGN_DTOR(-60));
	state.update();
	EXPECT_EQ(state.getState(), PlannerState::INITIATE_EXECUTION);

	// rotated enough
	g_pose = geometry::Pose(0.0, 0.0, IGN_DTOR(-135));
	state.update();
	EXPECT_EQ(state.getState(), PlannerState::MOVE);

	// got closer
	g_pose = geometry::Pose(-5.0, -5.0, IGN_DTOR(-135));
	g_goal_local = geometry::Pose(-7.0, -7.0, 0.0);
	g_goal_initiation = geometry::Pose(-7.5, -7.5, 0.0);
	state.update();
	EXPECT_EQ(state.getState(), PlannerState::MOVE);

	// position reached
	g_pose = geometry::Pose(-10.0, -10.0, IGN_DTOR(-135));
	g_goal_local = geometry::Pose(-10.0, -10.0, 0.0);
	state.update();
	EXPECT_EQ(state.getState(), PlannerState::ADJUST_ORIENTATION);

	// goal reached
	g_pose = geometry::Pose(-10.0, -10.0, 0.0);
	state.update();
	EXPECT_EQ(state.getState(), PlannerState::STOPPED);
}

TEST(HuberoPlannerState, moveInterrupted) {
	g_pose = geometry::Pose(0.0, 0.0, IGN_DTOR(45));
	g_goal = geometry::Pose(-10.0, -10.0, 0.0);
	g_goal_local = geometry::Pose(-2.0, -2.0, 0.0);
	g_goal_initiation = geometry::Pose(-2.5, -2.5, 0.0);
	PlannerState state(g_pose, g_goal, g_goal_local, g_goal_initiation, isPositionReached, isGoalReached);
	state.update(true); // newGoalReceived
	EXPECT_FALSE(state.isGoalReached());
	EXPECT_EQ(state.getState(), PlannerState::INITIATE_EXECUTION);

	// rotated but not enough
	g_pose = geometry::Pose(0.0, 0.0, IGN_DTOR(-60));
	state.update();
	EXPECT_EQ(state.getState(), PlannerState::INITIATE_EXECUTION);

	// rotated enough
	g_pose = geometry::Pose(0.0, 0.0, IGN_DTOR(-135));
	state.update();
	EXPECT_EQ(state.getState(), PlannerState::MOVE);

	// got closer
	g_pose = geometry::Pose(-5.0, -5.0, IGN_DTOR(-135));
	g_goal_local = geometry::Pose(-7.0, -7.0, 0.0);
	state.update();
	EXPECT_EQ(state.getState(), PlannerState::MOVE);

	// new goal
	g_goal = geometry::Pose(-5.0, 0.0, 0.0);
	g_goal_local = geometry::Pose(-5.0, -3.0, 0.0);
	g_goal_initiation = geometry::Pose(-5.0, -4., 0.0);
	state.update(true); // newGoalReceived
	EXPECT_EQ(state.getState(), PlannerState::INITIATE_EXECUTION);

	// rotated enough
	g_pose = geometry::Pose(-5.0, -5.0, IGN_DTOR(+90));
	state.update();
	EXPECT_EQ(state.getState(), PlannerState::MOVE);

	// got closer
	g_pose = geometry::Pose(-5.0, -2.5, IGN_DTOR(+90));
	g_goal_local = geometry::Pose(-5.0, -0.5, 0.0);
	g_goal_initiation = geometry::Pose(-5.0, -0.25, 0.0);
	state.update();
	EXPECT_EQ(state.getState(), PlannerState::MOVE);

	// position reached
	g_pose = geometry::Pose(-5.0, 0.0, IGN_DTOR(+90));
	g_goal_local = geometry::Pose(-5.0, 0.0, 0.0);
	g_goal_initiation = geometry::Pose(-5.0, 0.0, 0.0);
	state.update();
	EXPECT_EQ(state.getState(), PlannerState::ADJUST_ORIENTATION);

	// goal reached
	g_pose = geometry::Pose(-5.0, 0.0, 0.0);
	state.update();
	EXPECT_EQ(state.getState(), PlannerState::STOPPED);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
