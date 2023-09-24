#include <humap_local_planner/humap_planner_ros.h>
#include <humap_local_planner/utils/transformations.h>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/UInt8.h>

#include <people_msgs_utils/utils.h>

#include <memory>
#include <regex>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(humap_local_planner::HumapPlannerROS, nav_core::BaseLocalPlanner)

namespace humap_local_planner {

HumapPlannerROS::HumapPlannerROS():
		initialized_(false),
		planner_util_(nullptr),
		planner_(nullptr),
		costmap_ros_(nullptr),
		costmap_converter_loader_(
			"costmap_converter",
			"costmap_converter::BaseCostmapToPolygons"
		),
		obstacles_(nullptr),
		people_(nullptr),
		groups_(nullptr),
		dsrv_(nullptr),
		cfg_(nullptr),
		tf_buffer_(nullptr),
		odom_helper_("odom"),
		vis_("odom") {
}

HumapPlannerROS::~HumapPlannerROS(){
	//make sure to clean things up
	delete dsrv_;
}

void HumapPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf_buffer, costmap_2d::Costmap2DROS* costmap_ros) {
	// check if the plugin is already initialized
	if (!isInitialized()) {
		// create Node Handle with name of plugin (as used in move_base for loading)
		ros::NodeHandle private_nh("~/" + name);
		g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
		g_plan_pruned_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan_pruned", 1);
		l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
		traj_pcl_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("trajectories", 1);
		ttc_markers_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("ttc_prediction", 1);
		cost_grid_pcl_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("cost_cloud", 1);
		planner_state_pub_ = private_nh.advertise<std_msgs::UInt8>("planner_state", 1);

		std::string people_topic("/people");
		private_nh.param("people_topic", people_topic, people_topic);
		people_sub_ = private_nh.subscribe(people_topic, 5, &HumapPlannerROS::peopleCB, this);

		// assign args
		tf_buffer_ = tf_buffer;
		costmap_ros_ = costmap_ros;

		costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

		planner_util_ = std::make_shared<base_local_planner::LocalPlannerUtil>();
		planner_util_->initialize(tf_buffer, costmap, costmap_ros_->getGlobalFrameID());

		// reserve some memory for obstacles
		obstacles_ = std::make_shared<ObstContainer>();
		obstacles_->reserve(500);

		people_ = std::make_shared<People>();
		people_->reserve(50);
		groups_ = std::make_shared<Groups>();
		groups_->reserve(25);

		// create robot footprint/contour model
		RobotFootprintModelPtr robot_model;
		std::vector<geometry_msgs::Point> footprint_spec;
		std::tie(robot_model, footprint_spec) = getRobotFootprintFromParamServer(private_nh);
		ROS_INFO("Robot model has inscribed radius of %2.4f", robot_model->getInscribedRadius());

		// get parameters of HumapConfig via the node handle and override the default config
		cfg_ = std::make_shared<HumapConfigROS>();
		cfg_->loadFromParamServer(private_nh);

		// local planner
		planner_ = std::make_shared<HumapPlanner>(name, planner_util_, robot_model, footprint_spec);
		planner_->reconfigure(cfg_);

		// visualization
		vis_ = Visualization(costmap_ros_->getGlobalFrameID());
		vis_.initialize(private_nh);
		vis_.reconfigure(cfg_->getSfm()->max_force);

		// load costmap converter params
		std::string costmap_converter_plugin("");
		private_nh.param("costmap_converter_plugin", costmap_converter_plugin, costmap_converter_plugin);
		double costmap_converter_rate = 2.0;
		private_nh.param("costmap_converter_rate", costmap_converter_rate, costmap_converter_rate);
		bool costmap_converter_spin_thread = true;
		private_nh.param("costmap_converter_spin_thread", costmap_converter_spin_thread, costmap_converter_spin_thread);

		// costmap converter - TEB-based section
		try {
			costmap_converter_ = costmap_converter_loader_.createInstance(costmap_converter_plugin);
			// converter_name is the plugin name without a namespace
			std::string converter_name = costmap_converter_loader_.getName(costmap_converter_plugin);
			converter_name = std::regex_replace(converter_name, std::regex("::"), "/");
			costmap_converter_->setOdomTopic(cfg_->odom_topic);
			costmap_converter_->initialize(ros::NodeHandle(private_nh, "costmap_converter/" + converter_name));
			costmap_converter_->setCostmap2D(costmap);
			costmap_converter_->startWorker(costmap_converter_rate, costmap, costmap_converter_spin_thread);
			ROS_INFO(
				"Costmap conversion plugin %s loaded. Will run at %1.1f Hz",
				converter_name.c_str(),
				costmap_converter_rate
			);
		} catch(pluginlib::PluginlibException& ex) {
			ROS_WARN("The specified costmap converter plugin cannot be loaded. Obstacles will not be recognized. Error message: %s", ex.what());
			costmap_converter_.reset();
		}

		// dynamic reconfigure server
		dsrv_ = new dynamic_reconfigure::Server<HumapPlannerConfig>(private_nh);
		dynamic_reconfigure::Server<HumapPlannerConfig>::CallbackType cb =
			boost::bind(&HumapPlannerROS::reconfigureCB, this, _1, _2);
		dsrv_->setCallback(cb);

	    // set initialized flag
	    initialized_ = true;

	    ROS_DEBUG("humap_local_planner plugin initialized.");
	    return;
	}

	ROS_DEBUG("humap_local_planner plugin has already been initialized, doing nothing.");
}

bool HumapPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
	if (!isInitialized()) {
	  ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
	  return false;
	}
	ROS_INFO("Got a new plan with %3lu poses", orig_global_plan.size());

	// reset executed path only when goal changes
	if (!global_plan_.empty() && orig_global_plan.back().pose != global_plan_.back().pose) {
		vis_.resetPath();
	}

	global_plan_ = orig_global_plan;
	return planner_->setPlan(orig_global_plan);
}

bool HumapPlannerROS::isGoalReached() {
	if (!isInitialized()) {
	  ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
	  return false;
	}

	//we assume the global goal is the last point in the global plan
	geometry_msgs::PoseStamped goal;
	planner_util_->getGoal(goal);
	geometry_msgs::PoseStamped pose;
	costmap_ros_->getRobotPose(pose);
	geometry_msgs::PoseStamped velocity;
	odom_helper_.getRobotVel(velocity);

	bool reached = planner_->checkGoalReached(pose, velocity, goal);
	if (reached) {
		ROS_INFO("Goal reached!");
	}
	return reached;
}

// based on TEB's `computeVelocityCommands`
bool HumapPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
	if(!isInitialized()) {
		ROS_ERROR("humap_local_planner has not been initialized, please call initialize() before using this planner");
		return false;
	}

	// retrieve environment information and pass them to the Humap planner

	// Get robot pose
	geometry_msgs::PoseStamped robot_pose_geom;
	costmap_ros_->getRobotPose(robot_pose_geom);
	Pose robot_pose(robot_pose_geom);

	// Get robot velocity
	geometry_msgs::PoseStamped robot_vel_geom;
	odom_helper_.getRobotVel(robot_vel_geom);
	Vector robot_vel(robot_vel_geom);

	// Get robot goal
	geometry_msgs::PoseStamped robot_goal_geom;
	planner_util_->getGoal(robot_goal_geom);
	Pose robot_goal(robot_goal_geom);

	ROS_DEBUG(
		"pose (`%s`): x %5.3f, y %5.3f, th %5.3f | "
		"vel (`%s`): x %5.3f, y %5.3f, th %5.3f | "
		"goal (`%s`): x %5.3f, y %5.3f, th %5.3f\r\n",
		robot_pose_geom.header.frame_id.c_str(),
		robot_pose.getX(),
		robot_pose.getY(),
		robot_pose.getYaw(),
		robot_vel_geom.header.frame_id.c_str(),
		robot_vel.getX(),
		robot_vel.getY(),
		robot_vel.getZ(),
		robot_goal_geom.header.frame_id.c_str(),
		robot_goal.getX(),
		robot_goal.getY(),
		robot_goal.getYaw()
	);

	/*
	 * The planner has significant computation times. Locking the callback mutex or configuration parameters mutex
	 * leads to the situation where the execution of other important callbacks (e.g., related to the costmap layer
	 * sources) are blocked.
	 * So, locking the mutex for the whole planning procedure is not an option, therefore copies of updated messages
	 * (from callbacks) are saved in the callbacks and processed just before the planning.
	 */
	// propagate the parameters if they have been updated since the last control loop
	bool config_updated = updateParameters();

	// prepare obstacles
	updateObstacleContainerWithCostmapConverter();

	// prepare people
	updatePeopleContainer(config_updated);

	// prepare local plan
	planner_->updatePlan(robot_pose_geom);

	// prepare planning outputs
	geometry_msgs::PoseStamped drive_cmds;
	base_local_planner::Trajectory trajectory;

	// use planning or proactive approach
	if (cfg_->getGeneral()->planning_approach) {
		// update costs for trajectory scoring
		planner_->updateLocalCosts(costmap_ros_->getRobotFootprint());
		// sample trajectories and choose the one with the lowest cost
		trajectory = planner_->findBestTrajectory(robot_vel, obstacles_, people_, groups_, drive_cmds);
	} else {
		trajectory = planner_->findTrajectory(robot_vel, obstacles_, people_, groups_, drive_cmds);
	}

	cmd_vel.linear.x = drive_cmds.pose.position.x;
	cmd_vel.linear.y = drive_cmds.pose.position.y;
	cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

	// check forced-stop mode for cost tuning
	if (cfg_->getDiagnostics()->force_robot_stop) {
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.angular.z = 0.0;
	}

	// visualization
	auto vis_data = planner_->getMotionData();

	vis_.publishForceInternal(robot_pose.getPosition(), vis_data.force_internal_);
	vis_.publishForceInteraction(robot_pose.getPosition(), vis_data.force_interaction_);
	vis_.publishForceSocial(robot_pose.getPosition(), vis_data.force_social_);
	vis_.publishForceCombined(robot_pose.getPosition(), vis_data.force_combined_);
	vis_.publishBehaviourActive(robot_pose.getPosition(), vis_data.behaviour_active_);
	vis_.publishClosestPoints(vis_data.closest_points_static_, vis_data.closest_points_dynamic_);
	vis_.publishPath(robot_pose);
	vis_.publishGrid(robot_pose, *planner_);
	vis_.publishRobotFootprint(robot_pose, planner_->getRobotFootprintModel());
	vis_.publishGoal(robot_goal.getPosition());
	vis_.publishGoalLocal(planner_->getGoalLocal().getPosition());
	vis_.publishGoalInitiation(planner_->getGoalInitiation().getPosition());
	vis_.publishPlannerState(robot_pose.getPosition(), planner_->getStateName());

	base_local_planner::publishPlan(createLocalPlan(trajectory), l_plan_pub_);
	base_local_planner::publishPlan(global_plan_, g_plan_pub_);

	// global path plan that was pruned by the local trajectory planner
	if (g_plan_pruned_pub_.getNumSubscribers()) {
		nav_msgs::Path pruned_plan;
		pruned_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
		pruned_plan.header.stamp = ros::Time::now();
		pruned_plan.poses = planner_->getGlobalPlanPruned();
		g_plan_pruned_pub_.publish(pruned_plan);
	}

	// publish point cloud with explored trajectories only if the topic is subscribed as parameter set accordingly
	if (traj_pcl_pub_.getNumSubscribers() > 0) {
		traj_pcl_pub_.publish(createExploredTrajectoriesPcl());
	}

	// publish markers with trajectory predictions used for TTC computation
	if (ttc_markers_pub_.getNumSubscribers() > 0) {
		ttc_markers_pub_.publish(createTTCTrajectoriesMarkers());
	}

	// publish the visualization of the grid costs
	if (cost_grid_pcl_pub_.getNumSubscribers() > 0) {
		cost_grid_pcl_pub_.publish(createCostGridPcl());
	}

	// publish planner state to ROS topic
	std_msgs::UInt8 planner_state;
	planner_state.data = static_cast<uint8_t>(planner_->getState());
	planner_state_pub_.publish(planner_state);

	// if we cannot move... tell someone
	if (trajectory.cost_ < 0) {
		ROS_DEBUG_NAMED(
			"HumapPlannerROS",
			"The local planner failed to find a valid plan, cost functions discarded all candidates. "
			"This can mean there is an obstacle too close to the robot."
		);
		return false;
	}
	return true;
}

// protected
void HumapPlannerROS::reconfigureCB(HumapPlannerConfig &config, uint32_t level) {
	std::lock_guard<std::mutex> l(cfg_mutex_);
	cfg_params_ = config;
	cfg_updated_ = true;
}

// protected
void HumapPlannerROS::peopleCB(const people_msgs::PeopleConstPtr& msg) {
	if (msg->header.frame_id.empty()) {
		ROS_ERROR("Cannot process people message due to lack of frame_id");
		return;
	}

	std::lock_guard<std::mutex> l(cb_mutex_);
	people_orig_ = *msg;
	people_container_updated_ = true;
}

// protected
bool HumapPlannerROS::updateParameters() {
	std::lock_guard<std::mutex> l(cfg_mutex_);

	if (!cfg_updated_) {
		return false;
	}

	// fill cfg_ with the updated config
	cfg_->reconfigure(cfg_params_);

	// update visualization-related objects
	vis_.reconfigure(cfg_->getSfm()->max_force);

	// update the planner with social trajectory generator
	planner_->reconfigure(cfg_);

	// copy LocalPlannerLimits and propagate recent changes to LocalPlannerUtil
	base_local_planner::LocalPlannerLimits limits = *cfg_->getLimits();
	planner_util_->reconfigureCB(limits, false);

	cfg_updated_ = false;
	return true;
}

// protected
/// from TEB
bool HumapPlannerROS::updateObstacleContainerWithCostmapConverter() {
	if (!costmap_converter_) {
		return false;
	}

	// don't need to re-create the same obstacle set for the planner
	if (!costmap_converter_->isUpdated()) {
		return false;
	}

	//Get obstacles from costmap converter
	costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
	if (!obstacles) {
		return false;
	}

	// clear currently existing obstacles
	obstacles_->clear();

	for (std::size_t i=0; i<obstacles->obstacles.size(); ++i) {
		const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
		const geometry_msgs::Polygon* polygon = &obstacle->polygon;

		if (polygon->points.size()==1 && obstacle->radius > 0) {
			// Circle
			obstacles_->push_back(
				ObstaclePtr(
					new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)
				)
			);
		} else if (polygon->points.size()==1) {
			// Point
			obstacles_->push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
		} else if (polygon->points.size()==2) {
			// Line
			obstacles_->push_back(ObstaclePtr(
				new LineObstacle(
				polygon->points[0].x, polygon->points[0].y,
				polygon->points[1].x, polygon->points[1].y)
				)
			);
		} else if (polygon->points.size()>2) {
			// Real polygon
			PolygonObstacle* polyobst = new PolygonObstacle;
			for (std::size_t j=0; j<polygon->points.size(); ++j) {
				polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
			}
			polyobst->finalizePolygon();
			obstacles_->push_back(ObstaclePtr(polyobst));
		}

		// Set velocity, if obstacle is moving
		if(!obstacles_->empty()) {
			obstacles_->back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
		}
	}
}

// protected
bool HumapPlannerROS::updatePeopleContainer(bool config_updated) {
	if (people_ == nullptr || groups_ == nullptr) {
		ROS_ERROR_NAMED(
			"HumapPlannerROS",
			"People container will not be updated because pointer(s) are null."
		);
		return false;
	}

	std::lock_guard<std::mutex> l(cb_mutex_);

	// notify if a people information topic connection may be broken
	if (people_orig_.header.frame_id.empty()) {
		// when empty `frame_id` is passed to the `lookupTransform`, the `tf2::InvalidArgumentException` is thrown
		ROS_WARN_DELAYED_THROTTLE_NAMED(
			10.0,
			"HumapPlannerROS",
			"It seems that the local planner did not receive any messages about the people in the environment "
			"(the frame_id is unknown). People will not be considered during planning."
		);
		// reset `people_container_updated_` flag because the data is ill-formed and cannot be "consumed"
		people_container_updated_ = false;
		return false;
	}

	// conditions that tell us that the container should be recalculated based on the newest data;
	// note that calculating pred. steps is done per specific cfg param set; therefore, we have to recompute to avoid
	// problems accessing, e.g., wrong trajectory velocity (out of bounds). Hence, config_updated is checked here
	if (!people_container_updated_ && !config_updated) {
		return false;
	}

	// reset the flag
	people_container_updated_ = false;

	// transform people poses to local planner's frame - by default = no transform
	geometry_msgs::TransformStamped transform;
	transform.child_frame_id = planner_util_->getGlobalFrame();
	transform.header.frame_id = people_orig_.header.frame_id;
	transform.header.stamp = ros::Time::now();
	// indicates no transform
	transform.transform.rotation.w = 1.0;

	// lookup transform if frame IDs are not equal
	if (transform.child_frame_id != transform.header.frame_id) {
		try {
			transform = tf_buffer_->lookupTransform(transform.child_frame_id, transform.header.frame_id, ros::Time(0));
		} catch (tf2::LookupException& ex) {
			ROS_ERROR_NAMED(
				"HumapPlannerROS",
				"Cannot find a valid transform to apply for people poses. Looking for transform from %s to %s. "
				"Exception: %s",
				transform.header.frame_id.c_str(),
				transform.child_frame_id.c_str(),
				ex.what()
			);
			return false;
		}
	}

	// extract all data embedded in people_msgs/People
	std::vector<people_msgs_utils::Person> people_orig;
	std::vector<people_msgs_utils::Group> groups_orig;
	std::tie(people_orig, groups_orig) = people_msgs_utils::createFromPeople(people_orig_.people);

	// clear vector, we receive a new aggregated info
	people_->clear();
	groups_->clear();

	// trajectory prediction helpers
	double dt = 0.0;
	unsigned int prediction_steps = 0;
	{
		// parameters safety (possible reconfigure in a meantime)
		std::lock_guard<std::mutex> l(cfg_->getMutex());
		dt = cfg_->getGeneral()->sim_granularity;
		prediction_steps = std::ceil(cfg_->getGeneral()->sim_time / cfg_->getGeneral()->sim_granularity);
	}

	// transform to the planner frame
	for (auto& person: people_orig) {
		// first, check reliability of the tracked person, accept only accurate ones
		if (person.getReliability() < 1e-02) {
			continue;
		}
		// transform pose and vel
		person.transform(transform);
		// collect person entries in the target container creating an object and predicting its trajectory
		people_->push_back(Person(person, dt, prediction_steps));
	}
	for (auto& group: groups_orig) {
		// first, check reliability of the tracked group, accept only accurate ones
		if (group.getReliability() < 1e-02) {
			continue;
		}
		// transform pose and vel
		group.transform(transform);
		// collect group entries in the target container creating an object and predicting its trajectory
		groups_->push_back(Group(group, dt, prediction_steps));
	}
	return true;
}

// protected
std::tuple<RobotFootprintModelPtr, std::vector<geometry_msgs::Point>>
HumapPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle& nh) {
	// let the point model be a very small circle in fact
	const double POINT_MODEL_RADIUS = 0.001;
	// used if shape params cannot be defined
	const auto FOOTPRINT_FALLBACK = costmap_2d::makeFootprintFromRadius(POINT_MODEL_RADIUS);

	std::string model_name;
	if (!nh.getParam("footprint_model/type", model_name)) {
		ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
		return std::make_tuple(std::make_shared<PointRobotFootprint>(), FOOTPRINT_FALLBACK);
	}

	// point
	if (model_name.compare("point") == 0) {
		ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
		return std::make_tuple(std::make_shared<PointRobotFootprint>(), FOOTPRINT_FALLBACK);
	}

	// circular
	if (model_name.compare("circular") == 0) {
		// get radius
		double radius;
		if (!nh.getParam("footprint_model/radius", radius)) {
			ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
						   << "/footprint_model/radius' does not exist. Using point-model instead.");
			return std::make_tuple(std::make_shared<PointRobotFootprint>(), FOOTPRINT_FALLBACK);
		}
		ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
		auto footprint_geom = costmap_2d::makeFootprintFromRadius(radius);
		return std::make_tuple(std::make_shared<CircularRobotFootprint>(radius), footprint_geom);
	}

	// line
	if (model_name.compare("line") == 0) {
		// check parameters
		if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end")) {
			ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
						   << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
			return std::make_tuple(std::make_shared<PointRobotFootprint>(), FOOTPRINT_FALLBACK);
		}
		// get line coordinates
		std::vector<double> line_start, line_end;
		nh.getParam("footprint_model/line_start", line_start);
		nh.getParam("footprint_model/line_end", line_end);
		if (line_start.size() != 2 || line_end.size() != 2) {
			ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
						   << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
			return std::make_tuple(std::make_shared<PointRobotFootprint>(), FOOTPRINT_FALLBACK);
		}

		ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] <<"]m, line_end: ["
					 << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
		// create a string that is used to recreate valid costmap footprint
		std::string model_spec =
			"[[" + std::to_string(line_start[0]) + "," + std::to_string(line_start[1]) + "],"
			+ "[" + std::to_string(line_end[0]) + "," + std::to_string(line_end[1]) + "]]";
		std::vector<geometry_msgs::Point> footprint_geom;
		costmap_2d::makeFootprintFromString(model_spec, footprint_geom);

		return std::make_tuple(
			std::make_shared<LineRobotFootprint>(
				Eigen::Map<const Eigen::Vector2d>(line_start.data()),
				Eigen::Map<const Eigen::Vector2d>(line_end.data())
			),
			footprint_geom
		);
	}

	// two circles
	if (model_name.compare("two_circles") == 0) {
		// check parameters
		if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius")
				|| !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius")) {
			ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
						   << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
			return std::make_tuple(std::make_shared<PointRobotFootprint>(), FOOTPRINT_FALLBACK);
		}
		double front_offset, front_radius, rear_offset, rear_radius;
		nh.getParam("footprint_model/front_offset", front_offset);
		nh.getParam("footprint_model/front_radius", front_radius);
		nh.getParam("footprint_model/rear_offset", rear_offset);
		nh.getParam("footprint_model/rear_radius", rear_radius);
		ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius
					<< "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
		// footprint model (for costmap) can be created from 2 separate circles, but must be ordered properly
		throw std::runtime_error(
			"`two_circles` footprint specification is currently not supported, use another model: "
			"`point`, `circular`, `line`, `polygon`"
		);
		return std::make_tuple(
			std::make_shared<TwoCirclesRobotFootprint>(
				front_offset,
				front_radius,
				rear_offset,
				rear_radius
			),
			FOOTPRINT_FALLBACK
		);
	}

	// polygon
	if (model_name.compare("polygon") == 0) {
		// check parameters
		XmlRpc::XmlRpcValue footprint_xmlrpc;
		if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc) ) {
			ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
						   << "/footprint_model/vertices' does not exist. Using point-model instead.");
			return std::make_tuple(std::make_shared<PointRobotFootprint>(), FOOTPRINT_FALLBACK);
		}
		// get vertices
		if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
			try {
				teb::Point2dContainer polygon = teb::LocalPlannerROS::makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
				ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");

				// create a string representation of polygon-footprint spec
				std::string model_spec = "[";
				for (const auto& pt: polygon) {
					auto str_pt = "[" + std::to_string(pt[0]) + ", " + std::to_string(pt[1]) + "]";
					model_spec += str_pt;
				}
				model_spec += "]";
				std::vector<geometry_msgs::Point> footprint_geom;
				costmap_2d::makeFootprintFromString(model_spec, footprint_geom);

				return std::make_tuple(std::make_shared<PolygonRobotFootprint>(polygon), footprint_geom);
			} catch (const std::exception& ex) {
				ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
				return std::make_tuple(std::make_shared<PointRobotFootprint>(), FOOTPRINT_FALLBACK);
			}
		} else {
			ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
						   << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
			return std::make_tuple(std::make_shared<PointRobotFootprint>(), FOOTPRINT_FALLBACK);
		}
	}
	// otherwise
	ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
	return std::make_tuple(std::make_shared<PointRobotFootprint>(), FOOTPRINT_FALLBACK);
}

std::vector<geometry_msgs::PoseStamped> HumapPlannerROS::createLocalPlan(
	const base_local_planner::Trajectory& traj
) const {
	std::vector<geometry_msgs::PoseStamped> path;
	// Fill out the local plan
	for(unsigned int i = 0; i < traj.getPointsSize(); ++i) {
		double p_x, p_y, p_th;
		traj.getPoint(i, p_x, p_y, p_th);

		geometry_msgs::PoseStamped p;
		p.header.frame_id = planner_util_->getGlobalFrame();
		p.header.stamp = ros::Time::now();
		p.pose.position.x = p_x;
		p.pose.position.y = p_y;
		p.pose.position.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, p_th);
		tf2::convert(q, p.pose.orientation);
		path.push_back(p);
	}

	// reset the visualization of the plan
	if (path.empty()) {
		geometry_msgs::PoseStamped p;
		costmap_ros_->getRobotPose(p);
		p.header.frame_id = planner_util_->getGlobalFrame();
		p.header.stamp = ros::Time::now();
		path.push_back(p);
	}
	return path;
}

sensor_msgs::PointCloud2 HumapPlannerROS::createExploredTrajectoriesPcl() const {
	auto explored_trajs = planner_->getExploredTrajectories();

	sensor_msgs::PointCloud2 traj_cloud;
	traj_cloud.header.frame_id = planner_util_->getGlobalFrame();
	traj_cloud.header.stamp = ros::Time::now();

	if (explored_trajs.empty()) {
		return traj_cloud;
	}

	sensor_msgs::PointCloud2Modifier cloud_mod(traj_cloud);
	cloud_mod.setPointCloud2Fields(
		5,
		"x", 1, sensor_msgs::PointField::FLOAT32,
		"y", 1, sensor_msgs::PointField::FLOAT32,
		"z", 1, sensor_msgs::PointField::FLOAT32,
		"theta", 1, sensor_msgs::PointField::FLOAT32,
		"cost", 1, sensor_msgs::PointField::FLOAT32
	);

	// check how many points there will be in a point cloud
	unsigned int num_points = 0;
	for (
		std::vector<base_local_planner::Trajectory>::const_iterator t = explored_trajs.begin();
		t != explored_trajs.end();
		++t
	) {
		// ignore trajectories with negative cost
		if (t->cost_ < 0) {
			continue;
		}
		num_points += t->getPointsSize();
	}

	cloud_mod.resize(num_points);
	sensor_msgs::PointCloud2Iterator<float> iter_x(traj_cloud, "x");
	for (
		std::vector<base_local_planner::Trajectory>::const_iterator t = explored_trajs.begin();
		t != explored_trajs.end();
		++t
	) {
		// this should not happen (previous loop), but ignore trajectories with negative cost
		if (t->cost_ < 0) {
			continue;
		}

		// Fill out the plan
		for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
			double p_x, p_y, p_th;
			t->getPoint(i, p_x, p_y, p_th);
			iter_x[0] = p_x;
			iter_x[1] = p_y;
			iter_x[2] = 0.0;
			iter_x[3] = p_th;
			iter_x[4] = t->cost_;
			++iter_x;
		}
	}
	return traj_cloud;
}

visualization_msgs::MarkerArray HumapPlannerROS::createTTCTrajectoriesMarkers() const {
	// const reference here
	auto data_static = planner_->getTTCPredictionsStatic();
	auto data_dynamic = planner_->getTTCPredictionsDynamic();

	std_msgs::ColorRGBA color_static;
	color_static.r = 0.7;
	color_static.g = 0.1;
	color_static.b = 0.0;
	// static are well established thus alpha (reliability) is high
	color_static.a = 1.0;

	std_msgs::ColorRGBA color_dynamic;
	color_dynamic.r = 1.0;
	color_dynamic.g = 0.5;
	color_dynamic.b = 0.0;

	std_msgs::ColorRGBA color_robot;
	color_robot.r = 0.15;
	color_robot.g = 0.70;
	color_robot.b = 0.35;

	// prepare container for the marker list with static objects
	visualization_msgs::Marker mobj_static;
	mobj_static.header.frame_id = planner_util_->getGlobalFrame();
	mobj_static.header.stamp = ros::Time::now();
	mobj_static.type = visualization_msgs::Marker::CUBE_LIST;
	mobj_static.action = visualization_msgs::Marker::ADD;
	mobj_static.ns = "static";
	mobj_static.id = 0;
	mobj_static.scale.x = 0.15;
	mobj_static.scale.y = 0.15;
	mobj_static.scale.z = 0.05;
	mobj_static.pose.orientation.w = 1.0;

	// prepare container for the marker list with dynamic objects
	visualization_msgs::Marker mobj_dynamic = mobj_static;
	mobj_dynamic.type = visualization_msgs::Marker::SPHERE_LIST;
	mobj_dynamic.ns = "dynamic";
	mobj_dynamic.id = 1;

	// prepare container for the marker list with example robot poses
	visualization_msgs::Marker mrobot = mobj_static;
	mrobot.type = visualization_msgs::Marker::CUBE_LIST;
	mrobot.ns = "robot";
	mrobot.id = 2;

	if (!data_static.empty() && !data_static.at(0).empty()) {
		/*
		* These are static obstacles:
		* elements of `data_static`: do not iterate over different robot trajectories
		* elements of `traj`: do not iterate over trajectory's different timestamps
		*/
		// iterate only over objects of an example trajectory at the first timestamp
		for (const auto& obj: data_static.at(0).at(0)) {
			geometry_msgs::Point pt_obstacle;
			pt_obstacle.x = obj.object.getX();
			pt_obstacle.y = obj.object.getY();
			pt_obstacle.z = 0.0;
			mobj_static.points.push_back(pt_obstacle);
			mobj_static.colors.push_back(color_static);
		}

		// to differentiate alpha channel of the color
		bool first_timestep = true;
		// visualize robot trajectory - shows only robot closest point to an arbitrary obstacle
		for (const auto& ts: data_static.at(0)) {
			// arbitrary obstacle
			auto obj_robot_arrangement = ts.at(0);
			// vis robot
			geometry_msgs::Point pt_robot;
			pt_robot.x = obj_robot_arrangement.robot.getX();
			pt_robot.y = obj_robot_arrangement.robot.getY();
			pt_robot.z = 0.0;
			mrobot.points.push_back(pt_robot);
			auto color = color_robot;
			// reliability (alpha) drops over time
			if (first_timestep) {
				first_timestep = false;
				color.a = 0.90;
			} else {
				color.a = 0.08;
			}
			mrobot.colors.push_back(color);
		}
	}

	if (!data_dynamic.empty()) {
		/*
		* Visualize dynamic obstacles
		* elements of `data_dynamic`: do not iterate over different robot trajectories
		*/
		// to differentiate alpha channel of the color
		bool first_timestep = true;

		// iterate over timestamps of an example trajectory
		for (const auto& ts: data_dynamic.at(0)) {
			// iterate over objects
			for (const auto& obj: ts) {
				geometry_msgs::Point pt;
				pt.x = obj.object.getX();
				pt.y = obj.object.getY();
				pt.z = 0.0;
				mobj_dynamic.points.push_back(pt);
				auto color = color_dynamic;
				// reliability (alpha) drops over time
				if (first_timestep) {
					first_timestep = false;
					color.a = 0.90;
				} else {
					color.a = 0.04;
				}
				mobj_dynamic.colors.push_back(color);
			}
		}
	}

	// collect all markers
	visualization_msgs::MarkerArray marray;
	marray.markers.push_back(mobj_static);
	marray.markers.push_back(mobj_dynamic);
	marray.markers.push_back(mrobot);
	return marray;
}

sensor_msgs::PointCloud2 HumapPlannerROS::createCostGridPcl() const {
	sensor_msgs::PointCloud2 cost_cloud;
	cost_cloud.header.frame_id = planner_util_->getGlobalFrame();
	cost_cloud.header.stamp = ros::Time::now();

	sensor_msgs::PointCloud2Modifier cloud_mod(cost_cloud);
	cloud_mod.setPointCloud2Fields(
		9,
		"x", 1, sensor_msgs::PointField::FLOAT32,
		"y", 1, sensor_msgs::PointField::FLOAT32,
		"z", 1, sensor_msgs::PointField::FLOAT32,
		"total_cost", 1, sensor_msgs::PointField::FLOAT32,
		"path_cost", 1, sensor_msgs::PointField::FLOAT32,
		"goal_cost", 1, sensor_msgs::PointField::FLOAT32,
		"occ_cost", 1, sensor_msgs::PointField::FLOAT32,
		"alignment_cost", 1, sensor_msgs::PointField::FLOAT32,
		"goal_front_cost", 1, sensor_msgs::PointField::FLOAT32
	);

	unsigned int x_size = planner_util_->getCostmap()->getSizeInCellsX();
	unsigned int y_size = planner_util_->getCostmap()->getSizeInCellsY();
	double z_coord = 0.0;
	double x_coord, y_coord;

	cloud_mod.resize(x_size * y_size);
	sensor_msgs::PointCloud2Iterator<float> iter_x(cost_cloud, "x");

	for (unsigned int cx = 0; cx < x_size; cx++) {
		for (unsigned int cy = 0; cy < y_size; cy++) {
			planner_util_->getCostmap()->mapToWorld(cx, cy, x_coord, y_coord);
			std::map<std::string, float> costs;
			if (planner_->computeCellCost(cx, cy, costs)) {
				iter_x[0] = x_coord;
				iter_x[1] = y_coord;
				iter_x[2] = z_coord;
				iter_x[3] = costs["total"];
				iter_x[4] = costs["path"];
				iter_x[5] = costs["goal"];
				iter_x[6] = costs["layered"];
				iter_x[7] = costs["alignment"];
				iter_x[8] = costs["goal_front"];
				++iter_x;
			}
		}
	}
	return cost_cloud;
}

}; // namespace humap_local_planner
