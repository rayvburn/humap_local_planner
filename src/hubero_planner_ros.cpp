#include <hubero_local_planner/hubero_planner_ros.h>
#include <hubero_common/converter.h>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <memory>

#include <hubero_local_planner/utils/debug.h>
// debugging macros
#define DEBUG_BASIC 1
#define DEBUG_VERBOSE 0
#define debug_print_basic(fmt, ...) _template_debug_print_basic_(DEBUG_BASIC, fmt, ##__VA_ARGS__)
#define debug_print_verbose(fmt, ...) _template_debug_print_basic_(DEBUG_VERBOSE, fmt, ##__VA_ARGS__)

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
		tf_(nullptr),
		odom_helper_("odom"),
		vis_("odom") {
}

HuberoPlannerROS::~HuberoPlannerROS(){
	//make sure to clean things up
	delete dsrv_;
}

void HuberoPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) {
	// check if the plugin is already initialized
	if (!isInitialized()) {
		// create Node Handle with name of plugin (as used in move_base for loading)
		ros::NodeHandle private_nh("~/" + name);

		// assign args
		tf_ = tf;
		costmap_ros_ = costmap_ros;
		costmap_ros_->getRobotPose(current_pose_);

		costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

		planner_util_ = std::make_shared<base_local_planner::LocalPlannerUtil>();
		planner_util_->initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

		// reserve some memory for obstacles
		obstacles_ = std::make_shared<ObstContainer>();
		obstacles_->reserve(500);

		// create robot footprint/contour model
		RobotFootprintModelPtr robot_model = getRobotFootprintFromParamServer(private_nh);

		debug_print_basic("Robot model - inscribed radius = %2.4f \r\n",
				robot_model->getInscribedRadius()
		);

		// get parameters of HuberoConfig via the node handle and override the default config
		cfg_ = std::make_shared<HuberoConfigROS>();
		cfg_->loadFromParamServer(private_nh);

		// local planner
		planner_ = std::make_shared<HuberoPlanner>(name, planner_util_, robot_model);
		planner_->initialize(cfg_);

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

	return planner_->setPlan(orig_global_plan);
}

bool HuberoPlannerROS::isGoalReached() {
	//we assume the global goal is the last point in the global plan
	tf::Stamped<tf::Pose> goal;
	planner_util_->getGoal(goal);
	tf::Stamped<tf::Pose> pose;
	costmap_ros_->getRobotPose(pose);

	return planner_->checkGoalReached(pose, goal);
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
	tf::Stamped<tf::Pose> robot_pose;
	costmap_ros_->getRobotPose(robot_pose);
	debug_print_verbose("pose: x %2.4f / y %2.4f / z %2.4f / Rx %2.4f, Ry %2.4f, Rz %2.4f, Rw %2.4f | frame: %s \r\n",
		robot_pose.getOrigin().x(),
		robot_pose.getOrigin().y(),
		robot_pose.getOrigin().z(),
		robot_pose.getRotation().x(),
		robot_pose.getRotation().y(),
		robot_pose.getRotation().z(),
		robot_pose.getRotation().w(),
		costmap_ros_->getGlobalFrameID().c_str()
	);

	// Get robot velocity
	tf::Stamped<tf::Pose> robot_vel_tf;
	odom_helper_.getRobotVel(robot_vel_tf);
	geometry_msgs::Twist robot_vel_;
	robot_vel_.linear.x = robot_vel_tf.getOrigin().getX();
	robot_vel_.linear.y = robot_vel_tf.getOrigin().getY();
	robot_vel_.angular.z = tf::getYaw(robot_vel_tf.getRotation());
	debug_print_verbose("vel: x %2.4f / y %2.4f / yaw %2.4f \r\n",
		robot_vel_.linear.x,
		robot_vel_.linear.y,
		robot_vel_.angular.z
	);

	// Get robot goal
	tf::Stamped<tf::Pose> robot_goal;
	planner_util_->getGoal(robot_goal);
	debug_print_verbose("goal: x %2.4f / y %2.4f / yaw %2.4f | frame: %s \r\n",
			robot_goal.getOrigin().x(),
			robot_goal.getOrigin().y(),
			tf::getYaw(robot_goal.getRotation()),
			robot_goal.frame_id_.c_str()
	);

	// prepare obstacles
	updateObstacleContainerWithCostmapConverter();

	// compute force exerted on the particle (robot)
	Eigen::Vector3f force;
	planner_->compute(robot_pose, robot_vel_, robot_goal, obstacles_, force);

	computeTwist(robot_pose, force, cmd_vel);

	// visualization
	auto vis_data = planner_->getMotionData();
	Pose3 pose = converter::tfPoseToIgnPose(robot_pose);

	vis_.publishForceInternal(pose.Pos(), vis_data.force_internal);
	vis_.publishForceInteraction(pose.Pos(), vis_data.force_interaction);
	vis_.publishForceSocial(pose.Pos(), vis_data.force_social);
	vis_.publishForceCombined(pose.Pos(), vis_data.force_combined);
	vis_.publishBehaviourActive(pose.Pos(), vis_data.behaviour_active);
	vis_.publishClosestPoints(vis_data.closest_points);
	vis_.publishPath(pose);
	vis_.publishGrid(
			pose,
			converter::twistToIgnVector3(robot_vel_),
			converter::tfPoseToIgnPose(robot_goal),
			obstacles_,
			*planner_
	);
	vis_.publishRobotFootprint(pose, planner_->getRobotFootprintModel());

	return true;
}

// protected
void HuberoPlannerROS::reconfigureCB(HuberoPlannerConfig &config, uint32_t level) {
	cfg_->reconfigure(config);
	vis_.reconfigure(cfg_->getSfm()->max_force);
}

// protected
void HuberoPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
	ROS_INFO("[HuberoPlannerROS] publishLocalPlan()");
	base_local_planner::publishPlan(path, l_plan_pub_);
}

// protected
void HuberoPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
	ROS_INFO("[HuberoPlannerROS] publishGlobalPlan()");
	base_local_planner::publishPlan(path, g_plan_pub_);
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
/// from TEB
RobotFootprintModelPtr HuberoPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle& nh) {
	std::string model_name;
	if (!nh.getParam("footprint_model/type", model_name)) {
		ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
		return std::make_shared<PointRobotFootprint>();
	}

	// point
	if (model_name.compare("point") == 0) {
		ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
		return std::make_shared<PointRobotFootprint>();
	}

	// circular
	if (model_name.compare("circular") == 0) {
		// get radius
		double radius;
		if (!nh.getParam("footprint_model/radius", radius)) {
			ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
						   << "/footprint_model/radius' does not exist. Using point-model instead.");
			return std::make_shared<PointRobotFootprint>();
		}
		ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
		return std::make_shared<CircularRobotFootprint>(radius);
	}

	// line
	if (model_name.compare("line") == 0) {
		// check parameters
		if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end")) {
			ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
						   << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
			return std::make_shared<PointRobotFootprint>();
		}
		// get line coordinates
		std::vector<double> line_start, line_end;
		nh.getParam("footprint_model/line_start", line_start);
		nh.getParam("footprint_model/line_end", line_end);
		if (line_start.size() != 2 || line_end.size() != 2) {
			ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
						   << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
			return std::make_shared<PointRobotFootprint>();
		}

		ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] <<"]m, line_end: ["
					 << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
		return std::make_shared<LineRobotFootprint>(
				Eigen::Map<const Eigen::Vector2d>(line_start.data()),
				Eigen::Map<const Eigen::Vector2d>(line_end.data())
		);
	}

	// two circles
	if (model_name.compare("two_circles") == 0) {
		// check parameters
		if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius")
				|| !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius")) {
			ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
						   << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
			return std::make_shared<PointRobotFootprint>();
		}
		double front_offset, front_radius, rear_offset, rear_radius;
		nh.getParam("footprint_model/front_offset", front_offset);
		nh.getParam("footprint_model/front_radius", front_radius);
		nh.getParam("footprint_model/rear_offset", rear_offset);
		nh.getParam("footprint_model/rear_radius", rear_radius);
		ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius
					<< "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
		return std::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
	}

	// polygon
	if (model_name.compare("polygon") == 0) {
		// check parameters
		XmlRpc::XmlRpcValue footprint_xmlrpc;
		if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc) ) {
			ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
						   << "/footprint_model/vertices' does not exist. Using point-model instead.");
			return std::make_shared<PointRobotFootprint>();
		}
		// get vertices
		if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
			try {
				teb::Point2dContainer polygon = teb::LocalPlannerROS::makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
				ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
				return std::make_shared<PolygonRobotFootprint>(polygon);
			} catch (const std::exception& ex) {
				ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
				return std::make_shared<PointRobotFootprint>();
			}
		} else {
			ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
						   << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
			return std::make_shared<PointRobotFootprint>();
		}
	}
	// otherwise
	ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
	return std::make_shared<PointRobotFootprint>();
}

void HuberoPlannerROS::computeTwist(const tf::Stamped<tf::Pose>& pose, const Eigen::Vector3f& force, geometry_msgs::Twist& cmd_vel) const {
	// orientation of the force vector in the odometry frame
	Vector3 force_v(force[0], force[1], force[2]);
	Angle angle_force_v(std::atan2(force_v.Normalized().Y(), force_v.Normalized().X()));
	angle_force_v.Normalize();

	Angle angle_diff(angle_force_v - tf::getYaw(pose.getRotation()));
	angle_diff.Normalize();

	debug_print_verbose("force orient: %2.4f, base link orient: %2.4f, diff: %2.4f \r\n",
			angle_force_v.Degree(),
			tf::getYaw(pose.getRotation()) * 180 / IGN_PI,
			angle_diff.Degree()
	);

	double dt = cfg_->getGeneral()->sim_period;

	cmd_vel.angular.z = cfg_->getLimits()->max_rot_vel * sin(angle_diff.Radian()) * dt;
	cmd_vel.linear.x = cfg_->getLimits()->max_vel_x * cos(angle_diff.Radian()) * dt;

	debug_print_verbose("angular = %2.4f (max = %2.4f), linear = %2.4f (max = %2.4f) \r\n",
			cmd_vel.angular.z,
			cfg_->getLimits()->max_rot_vel,
			cmd_vel.linear.x,
			cfg_->getLimits()->max_vel_x
	);
}

}; // namespace hubero_local_planner
