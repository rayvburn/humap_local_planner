#include <hubero_local_planner/hubero_planner.h>
#include <hubero_local_planner/utils/transformations.h>
#include <hubero_local_planner/sfm/social_force_model.h>

#include <math.h>

#include <hubero_local_planner/utils/debug.h>
// debugging macros
#define DEBUG_BASIC 1
#define debug_print_basic(fmt, ...) _template_debug_print_basic_(DEBUG_BASIC, fmt, ##__VA_ARGS__)

namespace hubero_local_planner {

HuberoPlanner::HuberoPlanner(
		const std::string& name,
		std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util,
		RobotFootprintModelPtr robot_model,
		std::vector<geometry_msgs::Point> footprint_spec
):
	planner_util_(planner_util),
	goal_reached_(false),
	obstacles_(nullptr),
	robot_model_(robot_model),
	obstacle_costs_(planner_util_->getCostmap()),
	path_costs_(planner_util_->getCostmap()),
	goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
	goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
	alignment_costs_(planner_util_->getCostmap()),
	ttc_costs_(world_model_),
	speedy_goal_costs_(goal_)
{
	ros::NodeHandle private_nh("~/" + name);
	printf("[HuberoPlanner::HuberoPlanner] ctor, name: %s \r\n", name.c_str());

	// prints Fuzzy Inference System configuration, FIS used for trajectory generation
	generator_.printFisConfiguration();

	// configure trajectory critics
	/*
	 * Disabling of `setStopOnFailure` dramatically helps reaching goals hidden by an obstacle (wavefront propagation
	 * cannot reach it so a valid trajectory may be rejected)
	 */
	path_costs_.setStopOnFailure(false);
	goal_costs_.setStopOnFailure(false);
	goal_front_costs_.setStopOnFailure(false);
	alignment_costs_.setStopOnFailure(false);
	// whether footprint cost is summed throughout each point or maximum cost is used, max by default
	obstacle_costs_.setSumScores(false);
	oscillation_costs_.resetOscillationFlags();

	// set up all the cost functions that will be applied in order
	// (any function returning negative values will abort scoring, so the order can improve performance)
	std::vector<base_local_planner::TrajectoryCostFunction*> critics;
	critics.push_back(&obstacle_costs_);
	critics.push_back(&path_costs_);
	critics.push_back(&goal_costs_);
	critics.push_back(&alignment_costs_);
	critics.push_back(&goal_front_costs_);
	critics.push_back(&ttc_costs_);
	critics.push_back(&chc_costs_);
	critics.push_back(&speedy_goal_costs_);

	// trajectory generators
	std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
	generator_list.push_back(&generator_);

	// score sampled trajectories
	scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);
}

HuberoPlanner::~HuberoPlanner() {
	printf("[HuberoPlanner::HuberoPlanner] dtor \r\n");
}

void HuberoPlanner::reconfigure(HuberoConfigConstPtr cfg) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(configuration_mutex_);
	cfg_ = cfg;

	generator_.setParameters(
		cfg->getSfm(),
		cfg->getFis(),
		cfg->getGeneral()->sim_time,
		cfg->getGeneral()->sim_granularity,
		cfg->getGeneral()->angular_sim_granularity,
		cfg->getGeneral()->sim_period,
		cfg->getDiagnostics()->log_trajectory_generation_samples,
		cfg->getDiagnostics()->log_trajectory_generation_details,
		cfg->getDiagnostics()->log_trajectory_generation_fails
	);

	updateCostParameters();
}

bool HuberoPlanner::checkTrajectory(
		const Eigen::Vector3f pos,
		const Eigen::Vector3f vel,
		const Eigen::Vector3f vel_samples) {
	printf("[HuberoPlanner::checkTrajectory] \r\n");
	return false;
}

bool HuberoPlanner::updatePlan(const geometry_msgs::PoseStamped& global_pose) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(configuration_mutex_);

	// save for later use
	pose_ = Pose(global_pose);

	std::vector<geometry_msgs::PoseStamped> plan_local_pruned;
	// this prunes plan if appropriate parameter is set
	planner_util_->getLocalPlan(global_pose, plan_local_pruned);

	if (plan_local_pruned.empty()) {
		ROS_ERROR_NAMED("HuberoPlanner", "Could not update global plan - pruned plan is empty");
		return false;
	}

	// copy contents of the new global plan
	global_plan_.resize(plan_local_pruned.size());
	std::copy(plan_local_pruned.cbegin(), plan_local_pruned.cend(), global_plan_.begin());

	// try to update goal pose
	goal_ = getGoalFromPlan();

	// update local goal pose
	if (!chooseLocalGoal()) {
		ROS_ERROR_NAMED("HuberoPlanner", "Could not choose local goal during plan update");
		return false;
	}
	return true;
}

void HuberoPlanner::updateLocalCosts(const std::vector<geometry_msgs::Point>& footprint_spec) {
	// update robot footprint so it won't crash with anything
	obstacle_costs_.setFootprint(footprint_spec);
	// costs for going away from path
	path_costs_.setTargetPoses(global_plan_);
	// costs for not going towards the local goal as much as possible
	goal_costs_.setTargetPoses(global_plan_);
	// costs for robot being aligned with path (nose on path)
	alignment_costs_.setTargetPoses(global_plan_);

	/*
	 * Based on DWAPlanner::updatePlanAndLocalCosts authored by Eitan Marder-Eppstein
	 * We want the robot nose to be drawn to its final position before robot turns towards goal orientation
	 */
	std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
	Angle angle_to_goal(Vector(goal_.getY() - pose_.getY(), goal_.getX() - pose_.getX(), 0.0));
	angle_to_goal.normalize();

	double front_plan_shift_x = cfg_->getGeneral()->forward_point_distance * cos(angle_to_goal.getRadian());
	double front_plan_shift_y = cfg_->getGeneral()->forward_point_distance * sin(angle_to_goal.getRadian());
	front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x + front_plan_shift_x;
	front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + front_plan_shift_y;

	goal_front_costs_.setTargetPoses(front_global_plan);

	// reset TTC datasets collected during previous iteration
	ttc_costs_.reset();
}

base_local_planner::Trajectory HuberoPlanner::findBestTrajectory(
	const Vector& velocity,
	const ObstContainerConstPtr obstacles,
	const PeopleContainerConstPtr people,
	geometry_msgs::PoseStamped& drive_velocities
) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(configuration_mutex_);

	// assign, will likely be useful for planning
	vel_ = velocity;
	obstacles_ = obstacles;
	people_ = people;

	// velocity transformation - from base coordinate system to planner's frame (global velocity vector)
	Vector robot_vel_glob;
	computeVelocityGlobal(vel_, pose_, robot_vel_glob);

	world_model_ = World(pose_, robot_vel_glob, goal_local_, goal_);
	createEnvironmentModel(pose_);

	// prepare data for planning
	// TSP: Trajectory Sampling Parameters
	TrajectorySamplingParams tsp {};
	tsp.force_internal_amplifier_min = cfg_->getTrajectorySampling()->force_internal_amplifier_min;
	tsp.force_internal_amplifier_max = cfg_->getTrajectorySampling()->force_internal_amplifier_max;
	tsp.force_interaction_static_amplifier_min = cfg_->getTrajectorySampling()->force_interaction_static_amplifier_min;
	tsp.force_interaction_static_amplifier_max = cfg_->getTrajectorySampling()->force_interaction_static_amplifier_max;
	tsp.force_interaction_dynamic_amplifier_min = cfg_->getTrajectorySampling()->force_interaction_dynamic_amplifier_min;
	tsp.force_interaction_dynamic_amplifier_max = cfg_->getTrajectorySampling()->force_interaction_dynamic_amplifier_max;
	tsp.force_interaction_social_amplifier_min = cfg_->getTrajectorySampling()->force_interaction_social_amplifier_min;
	tsp.force_interaction_social_amplifier_max = cfg_->getTrajectorySampling()->force_interaction_social_amplifier_max;

	tsp.force_internal_amplifier_granularity = cfg_->getTrajectorySampling()->force_internal_amplifier_granularity;
	tsp.force_interaction_static_amplifier_granularity = cfg_->getTrajectorySampling()->force_interaction_static_amplifier_granularity;
	tsp.force_interaction_dynamic_amplifier_granularity = cfg_->getTrajectorySampling()->force_interaction_dynamic_amplifier_granularity;
	tsp.force_interaction_social_amplifier_granularity = cfg_->getTrajectorySampling()->force_interaction_social_amplifier_granularity;

	// initialize generator with updated parameters
	generator_.initialise(
		world_model_,
		vel_,
		tsp,
		cfg_->getLimits(),
		cfg_->getSfm()->mass,
		true
	);

	// find best trajectory by sampling and scoring the samples, `scored_sampling_planner_` uses `generator_` internally
	result_traj_.cost_ = -7;
	result_traj_.resetPoints();
	traj_explored_.clear();
	bool traj_valid = scored_sampling_planner_.findBestTrajectory(result_traj_, &traj_explored_);

	logTrajectoriesDetails();

	collectTrajectoryMotionData();

	// no legal trajectory, command zero
	if (!traj_valid || result_traj_.cost_ < 0) {
		drive_velocities.pose.position.x = 0;
		drive_velocities.pose.position.y = 0;
		drive_velocities.pose.position.z = 0;
		drive_velocities.pose.orientation.w = 1;
		drive_velocities.pose.orientation.x = 0;
		drive_velocities.pose.orientation.y = 0;
		drive_velocities.pose.orientation.z = 0;
		ROS_ERROR_NAMED(
			"HuberoPlanner",
			"Could not find a valid trajectory (cost %2.2f), applying zero velocity to base",
			result_traj_.cost_
		);
	} else {
		drive_velocities.pose.position.x = result_traj_.xv_;
		drive_velocities.pose.position.y = result_traj_.yv_;
		drive_velocities.pose.position.z = 0;
		tf2::Quaternion q;
		q.setRPY(0, 0, result_traj_.thetav_);
		tf2::convert(q, drive_velocities.pose.orientation);
	}
	return result_traj_;
}

base_local_planner::Trajectory HuberoPlanner::findTrajectory(
	const Vector& velocity,
	const ObstContainerConstPtr obstacles,
	const PeopleContainerConstPtr people,
	geometry_msgs::PoseStamped& drive_velocities
) {
	// make sure that our configuration doesn't change mid-run
	std::lock_guard<std::mutex> l(configuration_mutex_);

	vel_ = velocity;
	obstacles_ = obstacles;
	people_ = people;

	// velocity transformation - from base coordinate system to planner's frame (global velocity vector)
	Vector robot_vel_glob;
	computeVelocityGlobal(vel_, pose_, robot_vel_glob);

	world_model_ = World(pose_, robot_vel_glob, goal_local_, goal_);
	createEnvironmentModel(pose_);

	// initialize generator with updated parameters
	generator_.initialise(
		world_model_,
		velocity,
		cfg_->getLimits(),
		cfg_->getSfm()->mass
	);

	// trajectory that stores velocity commands, contains only seed vels
	base_local_planner::Trajectory traj;

	// perform SFM and fuzzy logic computations without planning
	generator_.generateTrajectoryWithoutPlanning(traj);

	drive_velocities.pose.position.x = traj.xv_;
	drive_velocities.pose.position.y = traj.yv_;
	drive_velocities.pose.position.z = 0;
	tf2::Quaternion q;
	q.setRPY(0, 0, traj.thetav_);
	tf2::convert(q, drive_velocities.pose.orientation);

	collectTrajectoryMotionData();

	return traj;
}

bool HuberoPlanner::computeCellCost(
	int cx,
	int cy,
	float& path_cost,
	float& goal_cost,
	float& occ_cost,
	float& total_cost
) {
	// initial checks if cell is traversible
	path_cost = path_costs_.getCellCosts(cx, cy);
	goal_cost = goal_costs_.getCellCosts(cx, cy);
	occ_cost = planner_util_->getCostmap()->getCost(cx, cy);

	// evaluate if any component's cost is critical
	bool cell_unreachable =
		path_cost == path_costs_.obstacleCosts()
		|| path_cost == path_costs_.unreachableCellCosts()
		|| occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

	if (cell_unreachable) {
		return false;
	}

	// scale and sum retrieved costs
	path_cost *= cfg_->getCost()->path_distance_scale;
	goal_cost *= cfg_->getCost()->goal_distance_scale;
	occ_cost *= cfg_->getCost()->occdist_scale;
	total_cost = path_cost + goal_cost + occ_cost;
	return true;
}

bool HuberoPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
	printf("[HuberoPlanner::setPlan] length %lu \r\n", orig_global_plan.size());
	return planner_util_->setPlan(orig_global_plan);
}

bool HuberoPlanner::checkGoalReached(const tf::Stamped<tf::Pose>& pose, const tf::Stamped<tf::Pose>& goal) {
	printf("[HuberoPlanner::checkGoalReached] \r\n");

	// this is slightly modified base_local_planner::isGoalReached() method
	double dist_xy = sqrt(
		pow(pose.getOrigin().getX() - goal.getOrigin().getX(), 2) +
		pow(pose.getOrigin().getY() - goal.getOrigin().getY(), 2)
	);
	debug_print_basic("\t dist_xy = %2.4f, xy_goal_tolerance = %2.4f  /  cond: %d \r\n",
			dist_xy,
			cfg_->getLimits()->xy_goal_tolerance,
			!(dist_xy > cfg_->getLimits()->xy_goal_tolerance)
	);

	if (dist_xy > cfg_->getLimits()->xy_goal_tolerance) {
		goal_reached_ = false;
		return false;
	}

	double dist_ang = angles::shortest_angular_distance(
			tf::getYaw(goal.getRotation()), tf::getYaw(pose.getRotation())
	);
	debug_print_basic("\t dist_ang = %2.4f, yaw_tolerance = %2.4f  /  cond: %d \r\n",
			dist_ang,
			cfg_->getLimits()->yaw_goal_tolerance,
			!(std::abs(dist_ang) > cfg_->getLimits()->yaw_goal_tolerance)
	);

	if (std::abs(dist_ang) > cfg_->getLimits()->yaw_goal_tolerance) {
		goal_reached_ = false;
		return false;
	}

	goal_reached_ = true;
	return true;
}

void HuberoPlanner::updateCostParameters() {
	// update cost scales (adjust them with costmap resolution)
	double cm_resolution = planner_util_->getCostmap()->getResolution();
	double occdist_scale_adjusted = cfg_->getCost()->occdist_scale * cm_resolution;
	double path_distance_scale_adjusted = cfg_->getCost()->path_distance_scale * cm_resolution;
	double goal_distance_scale_adjusted = cfg_->getCost()->goal_distance_scale * cm_resolution;

	obstacle_costs_.setScale(occdist_scale_adjusted);
	path_costs_.setScale(path_distance_scale_adjusted);
	goal_costs_.setScale(goal_distance_scale_adjusted);
	goal_front_costs_.setScale(goal_distance_scale_adjusted);
	alignment_costs_.setScale(path_distance_scale_adjusted);
	ttc_costs_.setScale(cfg_->getCost()->ttc_scale);
	chc_costs_.setScale(cfg_->getCost()->chc_scale);
	speedy_goal_costs_.setScale(cfg_->getCost()->speedy_goal_scale);

	// update other cost params
	oscillation_costs_.setOscillationResetDist(
		cfg_->getCost()->oscillation_reset_dist,
		cfg_->getCost()->oscillation_reset_angle
	);
	obstacle_costs_.setParams(
		cfg_->getLimits()->max_vel_trans,
		cfg_->getCost()->max_scaling_factor,
		cfg_->getCost()->scaling_speed
	);
	goal_front_costs_.setXShift(cfg_->getGeneral()->forward_point_distance);
	alignment_costs_.setXShift(cfg_->getGeneral()->forward_point_distance);
	ttc_costs_.setParameters(cfg_->getCost()->ttc_rollout_time, cfg_->getCost()->ttc_collision_distance);
	speedy_goal_costs_.setParameters(cfg_->getCost()->speedy_goal_distance, cfg_->getLimits()->min_vel_trans);
}

void HuberoPlanner::createEnvironmentModel(const Pose& pose_ref) {
	/*
	 * People are detected independently from obstacles so they must be erased from obstacles.
	 * Obstacle is treated as a person when most of its points are located within the given radius.
	 */
	static constexpr double PERSON_POLYGON_CONTAINMENT_RATE = 0.667;

	ObstContainer obstacles;
	for (const ObstaclePtr obstacle: *obstacles_) {
		// create a temporary polygon to find distances between person center and its boundary points
		geometry_msgs::Polygon polygon;
		obstacle->toPolygonMsg(polygon);

		// vector of containment rates (within people radiuses) of the obstacle
		std::vector<double> containment_rates;

		for (const Person& person: *people_) {
			// number of polygon points that are located within a radius of person model
			int polygon_pts_within_radius = 0;

			for (const auto& pt: polygon.points) {
				double dist = std::hypot(pt.x - person.getPose().getX(), pt.y - person.getPose().getY());
				if (dist <= cfg_->getGeneral()->person_model_radius) {
					polygon_pts_within_radius++;
				}
			}

			// if a majority of polygon pts is located within radius - abort this obstacle, otherwise add to container
			double containment_rate = static_cast<double>(polygon_pts_within_radius) / polygon.points.size();
			containment_rates.push_back(containment_rate);
		}

		if (
			!containment_rates.empty()
			&&
			*std::max_element(containment_rates.cbegin(), containment_rates.cend()) >= PERSON_POLYGON_CONTAINMENT_RATE
		) {
			continue;
		}
		obstacles.push_back(obstacle);
	}

	ROS_INFO_NAMED(
		"HuberoPlanner",
		"Creating environment model with %2lu obstacles extracted from world and %2lu people",
		obstacles.size(),
		people_->size()
	);

	// fill sparse world model with obstacles
	teb_local_planner::PoseSE2 pose = pose_ref.getAsTebPose();
	for (const ObstaclePtr obstacle: obstacles) {
		BaseRobotFootprintModel::ClosestPoints pts = robot_model_->calculateClosestPoints(pose, obstacle.get());

		Pose robot_closest_to_obstacle_pose(pts.robot);
		Pose obstacle_closest_to_robot_pose(pts.obstacle);

		Vector model_vel = Vector(
			(obstacle->getCentroidVelocity())[0],
			(obstacle->getCentroidVelocity())[1],
			0.0
		);

		bool force_dynamic_object_interpretation =
				cfg_->getSfm()->static_obj_interaction == sfm::StaticObjectInteraction::INTERACTION_REPULSIVE_EVASIVE;

		world_model_.addObstacle(
			robot_closest_to_obstacle_pose,
			obstacle_closest_to_robot_pose,
			model_vel,
			force_dynamic_object_interpretation
		);
	}

	// represent human with an obstacle
	for (const Person& person: *people_) {
		ObstaclePtr person_model_ptr = std::make_shared<CircularObstacle>(
			person.getPose().getX(),
			person.getPose().getY(),
			cfg_->getGeneral()->person_model_radius
		);

		auto pts = robot_model_->calculateClosestPoints(pose, person_model_ptr.get());

		Pose robot_closest_to_obstacle_pose(pts.robot);
		Pose obstacle_closest_to_robot_pose(pts.obstacle);

		bool force_dynamic_object_interpretation =
				cfg_->getSfm()->static_obj_interaction == sfm::StaticObjectInteraction::INTERACTION_REPULSIVE_EVASIVE;

		world_model_.addObstacle(
			robot_closest_to_obstacle_pose,
			obstacle_closest_to_robot_pose,
			person.getVelocity(),
			force_dynamic_object_interpretation
		);
	}
}

bool HuberoPlanner::chooseLocalGoal() {
	if (global_plan_.empty()) {
		ROS_ERROR_NAMED("HuberoPlanner", "Cannot choose local goal because global plan is empty");
		return false;
	}

	// find a point far enough so the social force can drive the robot towards instead of produce oscillations
	for (const auto& pose_plan: global_plan_) {
		double dist_x = pose_plan.pose.position.x - pose_.getX();
		double dist_y = pose_plan.pose.position.y - pose_.getY();
		double dist_sq = dist_x * dist_x + dist_y * dist_y;
		if (dist_sq > cfg_->getGeneral()->forward_point_distance) {
			goal_local_ = Pose(pose_plan.pose);
			return true;
		}
	}

	// seems that we have reached the furthest point in the plan
	goal_local_ = getGoalFromPlan();
	return true;
}

Pose HuberoPlanner::getGoalFromPlan() const {
	// it's safer to use second to last element, last may consist some strange pose,
	// e.g. point backwards or unreachable goal pose
	if (global_plan_.size() > 2) {
		return Pose(global_plan_.end()[-2].pose);
	}

	ROS_DEBUG_NAMED(
		"HuberoPlanner",
		"Cannot retrieve goal from the plan since it is too short (%lu poses)",
		global_plan_.size()
	);
	return goal_;
}

void HuberoPlanner::collectTrajectoryMotionData() {
	motion_data_.behaviour_active_ = generator_.getActiveFuzzyBehaviour();
	motion_data_.closest_points_static_ = generator_.getRobotStaticObstacleDistances();
	motion_data_.closest_points_dynamic_ = generator_.getRobotDynamicObstacleDistances();
	motion_data_.force_combined_ =
		generator_.getForceInternal()
		+ generator_.getForceInteraction()
		+ generator_.getForceSocial();
	motion_data_.force_interaction_ = generator_.getForceInteraction();
	motion_data_.force_internal_ = generator_.getForceInternal();
	motion_data_.force_social_ = generator_.getForceSocial();
}

void HuberoPlanner::logTrajectoriesDetails() {
	if (result_traj_.cost_ < 0) {
		ROS_ERROR_COND_NAMED(
			cfg_->getDiagnostics()->log_explored_trajectories,
			"HuberoPlanner",
			"Explored %lu trajectories, but couldn't find a valid one (cost %2.2f)",
			traj_explored_.size(),
			result_traj_.cost_
		);
	} else {
		ROS_INFO_COND_NAMED(
			cfg_->getDiagnostics()->log_explored_trajectories,
			"HuberoPlanner",
			"Explored %lu trajectories. Best trajectory consists of %u points and has cost of %2.2f. "
			"Commands base with x: %2.3f, y: %2.3f, th: %2.3f",
			traj_explored_.size(),
			result_traj_.getPointsSize(),
			result_traj_.cost_,
			result_traj_.xv_,
			result_traj_.yv_,
			result_traj_.thetav_
		);
	}

	// evaluate components contribution in total cost
	ROS_INFO_COND_NAMED(
		cfg_->getDiagnostics()->log_trajectory_cost_details,
		"HuberoPlanner",
		"Best trajectory cost details: "
		"obstacle %2.2f, oscillation %2.2f, path %2.2f, goal %2.2f, goal_front %2.2f, alignment %2.2f, "
		"TTC %2.2f, CHC %2.2f, speedy_goal %2.2f",
		obstacle_costs_.getScale() * obstacle_costs_.scoreTrajectory(result_traj_),
		oscillation_costs_.getScale() * oscillation_costs_.scoreTrajectory(result_traj_),
		path_costs_.getScale() * path_costs_.scoreTrajectory(result_traj_),
		goal_costs_.getScale() * goal_costs_.scoreTrajectory(result_traj_),
		goal_front_costs_.getScale() * goal_front_costs_.scoreTrajectory(result_traj_),
		alignment_costs_.getScale() * alignment_costs_.scoreTrajectory(result_traj_),
		ttc_costs_.getScale() * ttc_costs_.scoreTrajectory(result_traj_),
		chc_costs_.getScale() * chc_costs_.scoreTrajectory(result_traj_),
		speedy_goal_costs_.getScale() * speedy_goal_costs_.scoreTrajectory(result_traj_)
	);

	if (!cfg_->getDiagnostics()->log_explored_trajectories && !cfg_->getDiagnostics()->log_pts_of_explored_trajectories) {
		return;
	}

	// investigate costs and velocities of all explored trajectories
	int traj_num = 0;
	for (const auto& traj: traj_explored_) {
		double traj_x, traj_y, traj_th = 0.0;
		traj.getEndpoint(traj_x, traj_y, traj_th);

		// print basic trajectory info
		ROS_INFO_COND_NAMED(
			cfg_->getDiagnostics()->log_explored_trajectories,
			"HuberoPlanner",
			"Explored trajectory %3d / %3lu: cost %2.5f, vel {x %2.3f, y %2.3f, th %2.3f}, "
			"points %u, end {x %2.2f, y %2.2f, th %2.2f}",
			++traj_num,
			traj_explored_.size(),
			traj.cost_,
			traj.xv_, traj.yv_, traj.thetav_,
			traj.getPointsSize(),
			traj_x, traj_y, traj_th
		);

		// skip if points of explored trajectories are not needed to be printed
		if (!cfg_->getDiagnostics()->log_pts_of_explored_trajectories) {
			continue;
		}

		// also print trajectory points
		for (unsigned int pt_num = 0; pt_num < traj.getPointsSize(); pt_num++) {
			double x, y, th = 0.0;
			traj.getPoint(pt_num, x, y, th);
			ROS_INFO_NAMED(
				"HuberoPlanner",
				"\tpoint %3d / %3u: x %4.7f, y %4.7f, th %4.7f",
				pt_num + 1,
				traj.getPointsSize(),
				x,
				y,
				th
			);
		}
	}
}

}; // namespace hubero_local_planner
