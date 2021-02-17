#include <hubero_local_planner/hubero_planner.h>

namespace hubero_local_planner {

HuberoPlanner::HuberoPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util):
	planner_util_(planner_util),
    obstacle_costs_(planner_util->getCostmap()),
    path_costs_(planner_util->getCostmap()),
    goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
    goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
    alignment_costs_(planner_util->getCostmap())
{
	ros::NodeHandle private_nh("~/" + name);

    goal_front_costs_.setStopOnFailure(false);
    alignment_costs_.setStopOnFailure(false);

    // Assuming this planner is being run within the navigation stack, we can
    // just do an upward search for the frequency at which its being run. This
    // also allows the frequency to be overwritten locally.
    std::string controller_frequency_param_name;
    if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
    	sim_period_ = 0.05;
    } else {
		double controller_frequency = 0.0;
		private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
		if(controller_frequency > 0) {
			sim_period_ = 1.0 / controller_frequency;
		} else {
			ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
			sim_period_ = 0.05;
		}
    }
    ROS_INFO("Sim period is set to %.2f", sim_period_);

    oscillation_costs_.resetOscillationFlags();

    bool sum_scores;
    private_nh.param("sum_scores", sum_scores, false);
    obstacle_costs_.setSumScores(sum_scores);

    private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
    map_viz_.initialize(
		name,
		planner_util->getGlobalFrame(),
		boost::bind(&HuberoPlanner::getCellCosts, this, _1, _2, _3, _4, _5, _6)
    );

    std::string frame_id;
    private_nh.param("global_frame_id", frame_id, std::string("odom"));

    traj_cloud_ = new pcl::PointCloud<base_local_planner::MapGridCostPoint>;
    traj_cloud_->header.frame_id = frame_id;
    traj_cloud_pub_.advertise(private_nh, "trajectory_cloud", 1);
    private_nh.param("publish_traj_pc", publish_traj_pc_, false);

    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    // discards oscillating motions (assisgns cost -1)
    critics.push_back(&oscillation_costs_);
    // discards trajectories that move into obstacles
    critics.push_back(&obstacle_costs_);
    // prefers trajectories that make the nose go towards (local) nose goal
    critics.push_back(&goal_front_costs_);
    // prefers trajectories that keep the robot nose on nose path
    critics.push_back(&alignment_costs_);
    // prefers trajectories on global path
    critics.push_back(&path_costs_);
    // prefers trajectories that go towards (local) goal, based on wave propagation
    critics.push_back(&goal_costs_);
    // optionally prefer trajectories that don't spin
    critics.push_back(&twirling_costs_);

    // trajectory generators
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);

    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

    // private_nh.param("cheat_factor", cheat_factor_, 1.0);
}

HuberoPlanner::~HuberoPlanner() {

}

void HuberoPlanner::reconfigure(HuberoPlannerConfig &cfg) {
	std::lock_guard<std::mutex> lock(configuration_mutex_);
	// TODO
}

}; // namespace hubero_local_planner
