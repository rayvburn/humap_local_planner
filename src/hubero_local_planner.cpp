#include <hubero_local_planner/HuberoPlanner.h>

namespace hubero_local_planner {

HuberoPlanner::HuberoPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util):
	planner_util_(planner_util),
    obstacle_costs_(planner_util->getCostmap()),
    path_costs_(planner_util->getCostmap()),
    goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
    goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
    alignment_costs_(planner_util->getCostmap()) {

}

HuberoPlanner::~HuberoPlanner() {

}

void HuberoPlanner::reconfigure(HuberoPlannerConfig &cfg) {

}

}; // namespace hubero_local_planner
