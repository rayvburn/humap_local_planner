#include <hubero_local_planner/hubero_planner_ros.h>
#include <hubero_local_planner/utils/transformations.h>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <memory>

#include <hubero_local_planner/utils/debug.h>
// debugging macros
#define DEBUG_BASIC 1
#define DEBUG_VERBOSE 1
#define debug_print_basic(fmt, ...) _template_debug_print_basic_(DEBUG_BASIC, fmt, ##__VA_ARGS__)
#define debug_print_verbose(fmt, ...) _template_debug_print_basic_(DEBUG_VERBOSE, fmt, ##__VA_ARGS__)
#define debug_print_warn(fmt, ...) _template_debug_print_warn_(1, fmt, ##__VA_ARGS__)

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(hubero_local_planner::HuberoPlannerROS, nav_core::BaseLocalPlanner)

namespace hubero_local_planner {

HuberoPlannerROS::HuberoPlannerROS():
		initialized_(false),
		planner_util_(nullptr),
		planner_(nullptr),
		costmap_ros_(nullptr),
		costmap_converter_loader_(
			"costmap_converter",
			"costmap_converter::BaseCostmapToPolygons"
		),
		obstacles_(nullptr),
		dsrv_(nullptr),
		cfg_(nullptr),
		tf_buffer_(nullptr),
		odom_helper_("odom"),
		vis_("odom") {
}

HuberoPlannerROS::~HuberoPlannerROS(){
	//make sure to clean things up
	delete dsrv_;
}

void HuberoPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf_buffer, costmap_2d::Costmap2DROS* costmap_ros) {
	// check if the plugin is already initialized
	if (!isInitialized()) {
		// create Node Handle with name of plugin (as used in move_base for loading)
		ros::NodeHandle private_nh("~/" + name);
		g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
		l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
		traj_pcl_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("trajectories", 1);
		cost_grid_pcl_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("cost_cloud", 1);

		// assign args
		tf_buffer_ = tf_buffer;
		costmap_ros_ = costmap_ros;
		costmap_ros_->getRobotPose(current_pose_);

		costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

		planner_util_ = std::make_shared<base_local_planner::LocalPlannerUtil>();
		planner_util_->initialize(tf_buffer, costmap, costmap_ros_->getGlobalFrameID());

		// reserve some memory for obstacles
		obstacles_ = std::make_shared<ObstContainer>();
		obstacles_->reserve(500);

		// create robot footprint/contour model
		RobotFootprintModelPtr robot_model;
		std::vector<geometry_msgs::Point> footprint_spec;
		std::tie(robot_model, footprint_spec) = getRobotFootprintFromParamServer(private_nh);

		debug_print_basic("Robot model - inscribed radius = %2.4f \r\n",
				robot_model->getInscribedRadius()
		);

		// get parameters of HuberoConfig via the node handle and override the default config
		cfg_ = std::make_shared<HuberoConfigROS>();
		cfg_->loadFromParamServer(private_nh);

		// local planner
		planner_ = std::make_shared<HuberoPlanner>(name, planner_util_, robot_model, footprint_spec);
		planner_->reconfigure(cfg_);

		// visualization
		vis_.initialize(private_nh);
		vis_.reconfigure(cfg_->getSfm()->max_force);

		// costmap converter - TEB-based section
		try {
			// TODO: make it a param
			std::string costmap_converter_plugin_name = "costmap_converter::CostmapToLinesDBSRANSAC";

			costmap_converter_ = costmap_converter_loader_.createInstance(costmap_converter_plugin_name);
			// converter_name is the plugin name without a namespace
			std::string converter_name = costmap_converter_loader_.getName(costmap_converter_plugin_name);
			costmap_converter_->setOdomTopic(cfg_->odom_topic);
			costmap_converter_->initialize(ros::NodeHandle(private_nh, "costmap_converter/" + converter_name));
			costmap_converter_->setCostmap2D(costmap);
			costmap_converter_->startWorker(
				ros::Rate(2.0), // cfg_.obstacles.costmap_converter_rate
				costmap,
				true // cfg_.obstacles.costmap_converter_spin_thread
			);

			ROS_INFO_STREAM("Costmap conversion plugin " << costmap_converter_plugin_name << " loaded.");
		} catch(pluginlib::PluginlibException& ex) {
			ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
			costmap_converter_.reset();
		}

		// dynamic reconfigure server
		dsrv_ = new dynamic_reconfigure::Server<HuberoPlannerConfig>(private_nh);
		dynamic_reconfigure::Server<HuberoPlannerConfig>::CallbackType cb =
			boost::bind(&HuberoPlannerROS::reconfigureCB, this, _1, _2);
		dsrv_->setCallback(cb);

	    // set initialized flag
	    initialized_ = true;

	    ROS_DEBUG("hubero_local_planner plugin initialized.");
	    return;
	}

	ROS_DEBUG("hubero_local_planner plugin has already been initialized, doing nothing.");
}

bool HuberoPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
	if (!isInitialized()) {
	  ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
	  return false;
	}
	ROS_INFO("Got a new plan");

	// reset executed path only when goal changes
	if (!global_plan_.empty() && orig_global_plan.back().pose != global_plan_.back().pose) {
		vis_.resetPath();
	}

	global_plan_ = orig_global_plan;
	return planner_->setPlan(orig_global_plan);
}

bool HuberoPlannerROS::isGoalReached() {
	if (!isInitialized()) {
	  ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
	  return false;
	}

	//we assume the global goal is the last point in the global plan
	geometry_msgs::PoseStamped goal;
	planner_util_->getGoal(goal);
	geometry_msgs::PoseStamped pose;
	costmap_ros_->getRobotPose(pose);

    // conversion from geometry_msgs/PoseStamped to tf::Stamped
    tf::Stamped<tf::Pose> pose_tf = geometry::Pose(pose).getAsTfPose();
    pose_tf.stamp_ = pose.header.stamp;
    pose_tf.frame_id_ = pose.header.frame_id;

    tf::Stamped<tf::Pose> goal_tf = geometry::Pose(goal).getAsTfPose();
    goal_tf.stamp_ = goal.header.stamp;
    goal_tf.frame_id_ = goal.header.frame_id;

	bool reached = planner_->checkGoalReached(pose_tf, goal_tf);
	if (reached) {
		ROS_INFO("Goal reached!");
	}
	return reached;
}

// based on TEB's `computeVelocityCommands`
bool HuberoPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
	if(!isInitialized()) {
		ROS_ERROR("hubero_local_planner has not been initialized, please call initialize() before using this planner");
		return false;
	}
	ROS_INFO("[HuberoPlannerROS] computeVelocityCommands()");

	// retrieve environment information and pass them to the HuBeRo planner

	// Get robot pose
	geometry_msgs::PoseStamped robot_pose_geom;
	costmap_ros_->getRobotPose(robot_pose_geom);
	Pose robot_pose(robot_pose_geom);
	debug_print_verbose("pose: x %2.4f / y %2.4f / z %2.4f / Rr %2.4f, Rp %2.4f, Ry %2.4f | frame: %s \r\n",
		robot_pose.getX(),
		robot_pose.getY(),
		robot_pose.getZ(),
		robot_pose.getRoll(),
		robot_pose.getPitch(),
		robot_pose.getYaw(),
		costmap_ros_->getGlobalFrameID().c_str()
	);

	// Get robot velocity
	geometry_msgs::PoseStamped robot_vel_geom;
	odom_helper_.getRobotVel(robot_vel_geom);
	Vector robot_vel(robot_vel_geom);
	debug_print_verbose("vel: x %2.4f / y %2.4f / yaw %2.4f \r\n",
		robot_vel.getX(),
		robot_vel.getY(),
		robot_vel.getZ()
	);

	// Get robot goal
	geometry_msgs::PoseStamped robot_goal_geom;
	planner_util_->getGoal(robot_goal_geom);
	Pose robot_goal(robot_goal_geom);
	debug_print_verbose("goal: x %2.4f / y %2.4f / yaw %2.4f | frame: %s \r\n",
			robot_goal.getX(),
			robot_goal.getY(),
			robot_goal.getYaw(),
			robot_goal_geom.header.frame_id.c_str()
	);

	// prepare obstacles
	updateObstacleContainerWithCostmapConverter();

	geometry_msgs::PoseStamped drive_cmds;
	base_local_planner::Trajectory trajectory;
	// use planning or proactive approach
	if (cfg_->getGeneral()->planning_approach) {
		trajectory = planner_->findBestTrajectory(robot_pose, robot_vel, robot_goal, obstacles_, drive_cmds);
	} else {
		trajectory = planner_->findTrajectory(robot_pose, robot_vel, robot_goal, obstacles_, drive_cmds);
	}

	cmd_vel.linear.x = drive_cmds.pose.position.x;
	cmd_vel.linear.y = drive_cmds.pose.position.y;
	cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

	// visualization
	auto vis_data = planner_->getMotionData();

	vis_.publishForceInternal(robot_pose.getPosition(), vis_data.force_internal);
	vis_.publishForceInteraction(robot_pose.getPosition(), vis_data.force_interaction);
	vis_.publishForceSocial(robot_pose.getPosition(), vis_data.force_social);
	vis_.publishForceCombined(robot_pose.getPosition(), vis_data.force_combined);
	vis_.publishBehaviourActive(robot_pose.getPosition(), vis_data.behaviour_active);
	vis_.publishClosestPoints(vis_data.closest_points);
	vis_.publishPath(robot_pose);
	// TODO: handle this on visualization and planner sides
	// vis_.publishGrid(robot_pose, *planner_);
	vis_.publishRobotFootprint(robot_pose, planner_->getRobotFootprintModel());
	vis_.publishGoal(robot_goal.getPosition());
	vis_.publishGoalLocal(planner_->getGoalLocal().getPosition());

	base_local_planner::publishPlan(createLocalPlan(trajectory), l_plan_pub_);
	base_local_planner::publishPlan(global_plan_, g_plan_pub_);

	// publish point cloud with explored trajectories only if the topic is subscribed as parameter set accordingly
	if (traj_pcl_pub_.getNumSubscribers() > 0 && cfg_->getGeneral()->publish_traj_pcl) {
		traj_pcl_pub_.publish(createExploredTrajectoriesPcl());
	}

	// publish the visualization of the grid costs
	if (cost_grid_pcl_pub_.getNumSubscribers() > 0 && cfg_->getGeneral()->publish_cost_grid_pcl) {
		cost_grid_pcl_pub_.publish(createCostGridPcl());
	}
	return true;
}

bool HuberoPlannerROS::computeCellCost(int cx, int cy, float &total_cost) const {
	total_cost = planner_util_->getCostmap()->getCost(cx, cy);
	return true;
}

// protected
void HuberoPlannerROS::reconfigureCB(HuberoPlannerConfig &config, uint32_t level) {
	// fill cfg_ with updated config
	cfg_->reconfigure(config);

	// update visualization-related objects
	vis_.reconfigure(cfg_->getSfm()->max_force);

	// update Hubero planner with social trajectory generator
	planner_->reconfigure(cfg_);
}

// protected
/// from TEB
bool HuberoPlannerROS::updateObstacleContainerWithCostmapConverter() {
	if (!costmap_converter_) {
		return false;
	}

	//Get obstacles from costmap converter
	costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
	if (!obstacles) {
		return false;
	}

	ROS_INFO("[HuberoPlannerROS] obstacleContainerUpdate - obstacles %ld", obstacles->obstacles.size());

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
	return true;
}

// protected
std::tuple<RobotFootprintModelPtr, std::vector<geometry_msgs::Point>>
HuberoPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle& nh) {
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

std::vector<geometry_msgs::PoseStamped> HuberoPlannerROS::createLocalPlan(
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
	return path;
}

sensor_msgs::PointCloud2 HuberoPlannerROS::createExploredTrajectoriesPcl() const {
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

sensor_msgs::PointCloud2 HuberoPlannerROS::createCostGridPcl() const {
	sensor_msgs::PointCloud2 cost_cloud;
	cost_cloud.header.frame_id = planner_util_->getGlobalFrame();
	cost_cloud.header.stamp = ros::Time::now();

	sensor_msgs::PointCloud2Modifier cloud_mod(cost_cloud);
	cloud_mod.setPointCloud2Fields(
		4,
		"x", 1, sensor_msgs::PointField::FLOAT32,
		"y", 1, sensor_msgs::PointField::FLOAT32,
		"z", 1, sensor_msgs::PointField::FLOAT32,
		"total_cost", 1, sensor_msgs::PointField::FLOAT32
	);

	unsigned int x_size = planner_util_->getCostmap()->getSizeInCellsX();
	unsigned int y_size = planner_util_->getCostmap()->getSizeInCellsY();
	double z_coord = 0.0;
	double x_coord, y_coord;

	cloud_mod.resize(x_size * y_size);
	sensor_msgs::PointCloud2Iterator<float> iter_x(cost_cloud, "x");

	float total_cost;
	for (unsigned int cx = 0; cx < x_size; cx++) {
		for (unsigned int cy = 0; cy < y_size; cy++) {
			planner_util_->getCostmap()->mapToWorld(cx, cy, x_coord, y_coord);
			if (computeCellCost(cx, cy, total_cost)) {
				iter_x[0] = x_coord;
				iter_x[1] = y_coord;
				iter_x[2] = z_coord;
				iter_x[3] = total_cost;
				++iter_x;
			}
		}
	}
	return cost_cloud;
}

}; // namespace hubero_local_planner
