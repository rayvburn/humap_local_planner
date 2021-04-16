#include <hubero_local_planner/hubero_planner.h>
#include <hubero_common/converter.h>
#include <math.h>

#include <hubero_local_planner/utils/debug.h>
// debugging macros
#define DEBUG_BASIC 1
#define debug_print_basic(fmt, ...) _template_debug_print_basic_(DEBUG_BASIC, fmt, ##__VA_ARGS__)

namespace hubero_local_planner {

HuberoPlanner::HuberoPlanner(
		const std::string& name,
		std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util,
		RobotFootprintModelPtr footprint_model
):
	planner_util_(planner_util),
	// TODO: make it param
	sim_period_(0.2),
	goal_reached_(false),
	obstacles_(nullptr),
	robot_model_(footprint_model)
{
	ros::NodeHandle private_nh("~/" + name);
	printf("[HuberoPlanner::HuberoPlanner] ctor, name: %s \r\n", name.c_str());
}

HuberoPlanner::~HuberoPlanner() {
	printf("[HuberoPlanner::HuberoPlanner] dtor \r\n");
}

void HuberoPlanner::initialize(HuberoConfigConstPtr cfg) {
	cfg_ = cfg;
	sfm_.init(cfg->getSfm());
	social_conductor_.initialize(cfg->getBehaviour());
}

bool HuberoPlanner::checkTrajectory(
		const Eigen::Vector3f pos,
		const Eigen::Vector3f vel,
		const Eigen::Vector3f vel_samples) {
	printf("[HuberoPlanner::checkTrajectory] \r\n");
	return false;
}

bool HuberoPlanner::compute(
		const tf::Stamped<tf::Pose>& pose,
		const geometry_msgs::Twist& velocity,
		const tf::Stamped<tf::Pose>& goal,
		const ObstContainerConstPtr obstacles,
		Eigen::Vector3f& force
) {
	// NOTE: typical operations are available in `base_local_planner` ns as free functions
	// base_local_planner::getGoalPose()
	// NOTE: stick to ROS interface with base_local_planner functions,
	// let Hubero Planner abstract from the actual tf conversions etc
	//
	Vector3 force_result;
	bool status = compute(
			converter::tfPoseToIgnPose(pose),
			converter::twistToIgnVector3(velocity), // TODO angular velocity!
			converter::tfPoseToIgnPose(goal),
			obstacles,
			force_result
	);
	force = converter::ignVectorToEigenV3f(force_result);

	// TODO: saturate

	return true;
}

bool HuberoPlanner::compute(
		const Pose3& pose,
		const Vector3& velocity,
		const Pose3& goal,
		const ObstContainerConstPtr obstacles,
		Vector3& force
) {
	// assign, will likely be useful for planning
	pose_ = pose;
	vel_ = velocity;
	goal_ = goal;
	obstacles_ = obstacles;
	goal_local_ = goal_;
	chooseGoalBasedOnGlobalPlan();

	// store vectors of poses of closest points between robot and other objects; makes use out of obstacles
	// representation (extracted from costmap) and robot's footprint;
	// these, in fact, are used only for visualisation
	std::vector<sfm::Distance> meaningful_interaction_static;
	std::vector<sfm::Distance> meaningful_interaction_dynamic;

	// perform SFM and fuzzy logic computations
	compute(pose, force, meaningful_interaction_static, meaningful_interaction_dynamic);

	// prepare data for visualization
	motion_data_.force_internal = sfm_.getForceInternal();
	motion_data_.force_interaction = sfm_.getForceInteraction();
	motion_data_.force_social = social_conductor_.getSocialVector();
	motion_data_.force_combined = force;
	motion_data_.behaviour_active = social_conductor_.getBehaviourActive();

	motion_data_.closest_points.clear();
	for (const auto& dist_static: meaningful_interaction_static) {
		motion_data_.closest_points.push_back(dist_static.object);
		motion_data_.closest_points.push_back(dist_static.robot);
	}
	for (const auto& dist_dynamic: meaningful_interaction_dynamic) {
		motion_data_.closest_points.push_back(dist_dynamic.object);
		motion_data_.closest_points.push_back(dist_dynamic.robot);
	}

	return true;
}

bool HuberoPlanner::compute(const Pose3& pose, Vector3& force) {
	std::vector<sfm::Distance> meaningful_interaction_static;
	std::vector<sfm::Distance> meaningful_interaction_dynamic;
	return compute(pose, force, meaningful_interaction_static, meaningful_interaction_dynamic);
}

bool HuberoPlanner::plan() {
	return false;
}

Vector3 HuberoPlanner::computeForce() {
	return Vector3();
}

base_local_planner::Trajectory HuberoPlanner::findBestPath(
        tf::Stamped<tf::Pose> global_pose,
        tf::Stamped<tf::Pose> global_vel,
        tf::Stamped<tf::Pose>& drive_velocities) {
	printf("[HuberoPlanner::findBestPath] \r\n");
	return base_local_planner::Trajectory();
}

void HuberoPlanner::updatePlanAndLocalCosts(tf::Stamped<tf::Pose> global_pose,
		const std::vector<geometry_msgs::PoseStamped>& new_plan,
		const std::vector<geometry_msgs::Point>& footprint_spec) {
	printf("[HuberoPlanner::updatePlanAndLocalCosts] \r\n");
}

bool HuberoPlanner::getCellCosts(int cx,
		int cy,
		float &path_cost,
		float &goal_cost,
		float &occ_cost,
		float &total_cost) {
	printf("[HuberoPlanner::getCellCosts] \r\n");
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

void HuberoPlanner::createEnvironmentModel(const Pose3& pose_ref) {
	teb_local_planner::PoseSE2 pose(pose_ref.Pos().X(), pose_ref.Pos().Y(), pose_ref.Rot().Yaw());

	for (const hubero_local_planner::ObstaclePtr obstacle: *obstacles_) {
		// TODO: consider actor personal space even when he is standing!

		// basic
		Pose3 robot_closest_to_obstacle_pose(pose.x(), pose.y(), 0.0, 0.0, 0.0, pose.theta());
		Pose3 obstacle_closest_to_robot_pose;

		BaseRobotFootprintModel::ClosestPoints pts = robot_model_->calculateClosestPoints(pose, obstacle.get());

		robot_closest_to_obstacle_pose = Pose3(pts.robot.x(), pts.robot.y(), 0.0, 0.0, 0.0, pts.robot.theta());
		obstacle_closest_to_robot_pose = Pose3(pts.obstacle.x(), pts.obstacle.y(), 0.0, 0.0, 0.0, pts.obstacle.theta());

		auto distance_v_eig =
				Eigen::Vector2d(
					obstacle_closest_to_robot_pose.Pos().X(),
					obstacle_closest_to_robot_pose.Pos().Y())
				- Eigen::Vector2d(
					pose.x(),
					pose.y()
		);
		Vector3 distance_v(distance_v_eig[0], distance_v_eig[1], 0.0);
		double distance = obstacle->getMinimumDistance(Eigen::Vector2d(pose.x(), pose.y()));
		Vector3 model_vel = Vector3((obstacle->getCentroidVelocity())[0], (obstacle->getCentroidVelocity())[1], 0.0);

		bool force_dynamic_object_interpretation =
				cfg_->getSfm()->static_obj_interaction == sfm::StaticObjectInteraction::INTERACTION_REPULSIVE_EVASIVE;
		world_model_.addObstacle(
			robot_closest_to_obstacle_pose,
			obstacle_closest_to_robot_pose,
			model_vel,
			force_dynamic_object_interpretation
		);
	}
}

bool HuberoPlanner::chooseGoalBasedOnGlobalPlan() {
	tf::Stamped<tf::Pose> pose_tf;
	pose_tf.frame_id_ = planner_util_->getGlobalFrame();
	pose_tf.stamp_ = ros::Time::now();
	tf::Vector3 origin(pose_.Pos().X(), pose_.Pos().Y(), pose_.Pos().Z());
	tf::Quaternion rot(pose_.Rot().X(), pose_.Rot().Y(), pose_.Rot().Z(), pose_.Rot().W());
	pose_tf.setOrigin(origin);
	pose_tf.setRotation(rot);

	std::vector<geometry_msgs::PoseStamped> plan_local_pruned;
	// this prunes plan if appropriate parameter is set
	planner_util_->getLocalPlan(pose_tf, plan_local_pruned);

	// NOTE: global_plan_ is not stored currently!
	// base_local_planner::prunePlan(poseTf, plan_local_, global_plan_);

	if (plan_local_pruned.size() > 0) {
		// find a point far enough so the social force can drive the robot towards instead of produce oscillations
		for (const auto& pose_plan : plan_local_pruned) {
			double dist_x = pose_plan.pose.position.x - pose_.Pos().X();
			double dist_y = pose_plan.pose.position.y - pose_.Pos().Y();
			double dist_sq = dist_x * dist_x + dist_y * dist_y;
			if (dist_sq > cfg_->getGeneral()->forward_point_distance) {
				goal_local_ = Pose3(
					pose_plan.pose.position.x,
					pose_plan.pose.position.y,
					pose_plan.pose.position.z,
					pose_plan.pose.orientation.x,
					pose_plan.pose.orientation.y,
					pose_plan.pose.orientation.z,
					pose_plan.pose.orientation.w
				);
				return true;
			}
		}
	}
	return false;
}

bool HuberoPlanner::compute(
	const Pose3& pose,
	Vector3& force,
	std::vector<sfm::Distance>& meaningful_interaction_static,
	std::vector<sfm::Distance>& meaningful_interaction_dynamic
) {
	world_model_ = sfm::World(pose, vel_, goal_/*goal_local_*/, goal_);
	createEnvironmentModel(pose);

	sfm_.computeSocialForce(
		world_model_,
		cfg_->getGeneral()->sim_period,
		meaningful_interaction_static,
		meaningful_interaction_dynamic
	);

	// actual `social` vector
	Vector3 human_action_force;

	std::vector<sfm::StaticObject> objects_static = world_model_.getStaticObjectsData();
	std::vector<sfm::DynamicObject> objects_dynamic = world_model_.getDynamicObjectsData();
	// evaluate whether more complex forces are supposed to be calculated
	// TODO: add param `disable fuzzy behaviours`
	if (!cfg_->getSfm()->disable_interaction_forces) {
		// All multi-element data are vectors of the same length whose corresponding elements are related
		// to the same \beta object (i.e. i-th index of each vector variable is related to the same \beta
		// object). Note: \beta objects can be considered as obstacles
		//
		// vector of \beta objects direction of motion
		std::vector<double> dir_beta_dynamic;
		// vector of \beta objects' relative to \f$\alpha\f$ locations
		std::vector<double> rel_loc_dynamic;
		// set of dynamic objects vector directions. Each of these vectors connect \f$\alpha\f$ with \beta_i
		std::vector<double> dist_angle_dynamic;
		// set of lengths of vectors connecting \beta -s and \alpha -s
		std::vector<double> dist_dynamic;
		// \f$\alpha\f$'s direction of motion expressed in world coordinate system
		double dir_alpha = world_model_.getRobotData().heading_dir;

		for (const sfm::DynamicObject& object: objects_dynamic) {
			dir_beta_dynamic.push_back(object.dir_beta);
			rel_loc_dynamic.push_back(object.rel_loc_angle);
			dist_angle_dynamic.push_back(object.dist_angle);
			dist_dynamic.push_back(object.dist);
		}

		// execute fuzzy operations block
		fuzzy_processor_.load(
			dir_alpha,
			dir_beta_dynamic,
			rel_loc_dynamic,
			dist_angle_dynamic
		);
		fuzzy_processor_.process();

		// create a force vector according to the activated `social behaviour`
		social_conductor_.apply(
			sfm_.getForceCombined(),
			dir_alpha,
			dist_dynamic,
			fuzzy_processor_.getOutput()
		);

		// assign `social` vector
		human_action_force = social_conductor_.getSocialVector();
	}

	force = sfm_.getForceCombined() + human_action_force;

	return true;
}

}; // namespace hubero_local_planner
