#include <humap_local_planner/contextualized_cost_function.h>

namespace humap_local_planner {

ContextualizedCostFunction::ContextualizedCostFunction(costmap_2d::Costmap2D* costmap):
	costmap_(costmap) {}

void ContextualizedCostFunction::setSumScores(bool score_sums) {
	sum_scores_ = score_sums;
}

bool ContextualizedCostFunction::prepare() {
	return true;
}

double ContextualizedCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	double score = 0.0;

	// check all points
	for (unsigned int i = 0; i < traj.getPointsSize(); i++) {
		// world coords
		double wx, wy, wth = 0.0;
		traj.getPoint(i, wx, wy, wth);
		// map coords
		unsigned int mx, my = 0;
		if (!costmap_->worldToMap(wx, wy, mx, my)) {
			ROS_ERROR_NAMED(
				"contextualized_cost_function",
				"Could not transform world (wx %6.2f, wy %6.2f) to map",
				wx,
				wy
			);
			return -7.0;
		}

		double cost_current_cell = costmap_->getCost(mx, my);
		if (sum_scores_) {
			score += cost_current_cell;
		} else {
			score = std::max(cost_current_cell, score);
		}
	}
	return score;
}

}
