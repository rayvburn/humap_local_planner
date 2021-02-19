#include <hubero_local_planner/hubero_planner.h>
#include <math.h>

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
	delete planner_util_;
	delete traj_cloud_;
}

void HuberoPlanner::reconfigure(HuberoPlannerConfig &cfg) {
	std::lock_guard<std::mutex> lock(configuration_mutex_);
	// TODO
	std::cout << "reconfigure Callback" << std::endl;
}

bool HuberoPlanner::checkTrajectory(
		const Eigen::Vector3f pos,
		const Eigen::Vector3f vel,
		const Eigen::Vector3f vel_samples) {
	oscillation_costs_.resetOscillationFlags();
	base_local_planner::Trajectory traj;
	geometry_msgs::PoseStamped goal_pose = global_plan_.back();
	Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
	base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
	generator_.initialise(pos,
		vel,
		goal,
		&limits,
		vsamples_);
	generator_.generateTrajectory(pos, vel, vel_samples, traj);
	double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
	//if the trajectory is a legal one... the check passes
	if(cost >= 0) {
	  return true;
	}
	ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

	//otherwise the check fails
	return false;
}

base_local_planner::Trajectory HuberoPlanner::findBestPath(
        tf::Stamped<tf::Pose> global_pose,
        tf::Stamped<tf::Pose> global_vel,
        tf::Stamped<tf::Pose>& drive_velocities) {
    //make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> lock(configuration_mutex_);

    Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
    Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(), tf::getYaw(global_vel.getRotation()));
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    // prepare cost functions and generators for this run
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);

    result_traj_.cost_ = -7;
    // find best trajectory by sampling and scoring the samples
    std::vector<base_local_planner::Trajectory> all_explored;
    scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);

    if(publish_traj_pc_)
    {
        base_local_planner::MapGridCostPoint pt;
        traj_cloud_->points.clear();
        traj_cloud_->width = 0;
        traj_cloud_->height = 0;
        std_msgs::Header header;
        pcl_conversions::fromPCL(traj_cloud_->header, header);
        header.stamp = ros::Time::now();
        traj_cloud_->header = pcl_conversions::toPCL(header);
        for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
        {
            if(t->cost_<0)
                continue;
            // Fill out the plan
            for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
                double p_x, p_y, p_th;
                t->getPoint(i, p_x, p_y, p_th);
                pt.x=p_x;
                pt.y=p_y;
                pt.z=0;
                pt.path_cost=p_th;
                pt.total_cost=t->cost_;
                traj_cloud_->push_back(pt);
            }
        }
        traj_cloud_pub_.publish(*traj_cloud_);
    }

    // verbose publishing of point clouds
    if (publish_cost_grid_pc_) {
      //we'll publish the visualization of the costs to rviz before returning our best trajectory
      map_viz_.publishCostCloud(planner_util_->getCostmap());
    }

    // debrief stateful scoring functions
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_trans_vel);

    //if we don't have a legal trajectory, we'll just command zero
    if (result_traj_.cost_ < 0) {
      drive_velocities.setIdentity();
    } else {
      tf::Vector3 start(result_traj_.xv_, result_traj_.yv_, 0);
      drive_velocities.setOrigin(start);
      tf::Matrix3x3 matrix;
      matrix.setRotation(tf::createQuaternionFromYaw(result_traj_.thetav_));
      drive_velocities.setBasis(matrix);
    }

    return result_traj_;
}

void HuberoPlanner::updatePlanAndLocalCosts(tf::Stamped<tf::Pose> global_pose,
		const std::vector<geometry_msgs::PoseStamped>& new_plan,
		const std::vector<geometry_msgs::Point>& footprint_spec) {
	global_plan_.resize(new_plan.size());
	for (unsigned int i = 0; i < new_plan.size(); ++i) {
	  global_plan_[i] = new_plan[i];
	}

	obstacle_costs_.setFootprint(footprint_spec);

	// costs for going away from path
	path_costs_.setTargetPoses(global_plan_);

	// costs for not going towards the local goal as much as possible
	goal_costs_.setTargetPoses(global_plan_);

	// alignment costs
	geometry_msgs::PoseStamped goal_pose = global_plan_.back();

	Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
	double sq_dist =
		(pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
		(pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

	// we want the robot nose to be drawn to its final position
	// (before robot turns towards goal orientation), not the end of the
	// path for the robot center. Choosing the final position after
	// turning towards goal orientation causes instability when the
	// robot needs to make a 180 degree turn at the end
	std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
	double angle_to_goal = std::atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
	front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
	  forward_point_distance_ * cos(angle_to_goal);
	front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
	  sin(angle_to_goal);

	goal_front_costs_.setTargetPoses(front_global_plan);

	// keeping the nose on the path
	if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_) {
	  double resolution = planner_util_->getCostmap()->getResolution();
	  alignment_costs_.setScale(resolution * pdist_scale_ * 0.5);
	  // costs for robot being aligned with path (nose on path, not ju
	  alignment_costs_.setTargetPoses(global_plan_);
	} else {
	  // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
	  alignment_costs_.setScale(0.0);
	}
}

bool HuberoPlanner::getCellCosts(int cx,
		int cy,
		float &path_cost,
		float &goal_cost,
		float &occ_cost,
		float &total_cost) {
    path_cost = path_costs_.getCellCosts(cx, cy);
    goal_cost = goal_costs_.getCellCosts(cx, cy);
    occ_cost = planner_util_->getCostmap()->getCost(cx, cy);
    if (path_cost == path_costs_.obstacleCosts() ||
        path_cost == path_costs_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }

    double resolution = planner_util_->getCostmap()->getResolution();
    total_cost =
		/* DWA-specific computations removed */
        /* pdist_scale_ * resolution * */ path_cost +
        /* gdist_scale_ * resolution * */ goal_cost +
        /* occdist_scale_ * */ occ_cost;
    return true;
}

bool HuberoPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    oscillation_costs_.resetOscillationFlags();
    return planner_util_->setPlan(orig_global_plan);
}

}; // namespace hubero_local_planner
