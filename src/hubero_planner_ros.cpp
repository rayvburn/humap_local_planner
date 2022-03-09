#include <hubero_local_planner/hubero_planner_ros.h>

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
	if (!isInitialized()) {
	  ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
	  return false;
	}
	ROS_INFO("Got a new plan");
	global_plan_ = orig_global_plan;
	vis_.resetPath();
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
	// velocity transformation - from base coordinate system to planner's frame (global velocity vector)
	Vector robot_vel_glob;
	computeVelocityGlobal(robot_vel, robot_pose, robot_vel_glob);

	// compute force exerted on the particle (robot)
	Vector force;
	planner_->compute(robot_pose, robot_vel_glob, robot_goal, obstacles_, force);

	Vector velocity_cmd;
	computeTwist(robot_pose, force, robot_vel_glob, velocity_cmd);
	cmd_vel = velocity_cmd.getAsTwist();

	// visualization
	auto vis_data = planner_->getMotionData();

	vis_.publishForceInternal(robot_pose.getPosition(), vis_data.force_internal);
	vis_.publishForceInteraction(robot_pose.getPosition(), vis_data.force_interaction);
	vis_.publishForceSocial(robot_pose.getPosition(), vis_data.force_social);
	vis_.publishForceCombined(robot_pose.getPosition(), vis_data.force_combined);
	vis_.publishBehaviourActive(robot_pose.getPosition(), vis_data.behaviour_active);
	vis_.publishClosestPoints(vis_data.closest_points);
	vis_.publishPath(robot_pose);
	vis_.publishGrid(robot_pose, *planner_);
	vis_.publishRobotFootprint(robot_pose, planner_->getRobotFootprintModel());
	vis_.publishGoal(robot_goal.getPosition());
	vis_.publishGoalLocal(planner_->getGoalLocal().getPosition());

	//publishLocalPlan(/*TODO*/);
	publishGlobalPlan(global_plan_);
	return true;
}

void HuberoPlannerROS::computeTwist(
	const Pose& pose,
	const Vector& force,
	const Vector& robot_vel_glob,
	const double& sim_period,
	const double& robot_mass,
	const double& min_vel_x,
	const double& max_vel_x,
	const double& max_rot_vel,
	const double& twist_rotation_compensation,
	Vector& cmd_vel
) {
	// no force - no movement (strain physical laws)
	if (force.calculateLength() <= 1e-08) {
		cmd_vel = Vector(0.0, 0.0, 0.0);
		return;
	}

	// convert 2D forces into robot forces with non-holonomic contraints
	double yaw = pose.getYaw();

	// alias
	double dt = sim_period;

	// conversion
	Angle force_dir(force);

	// global force affects global acceleration directly
	Vector acc_v = force / robot_mass;
	Vector vel_v_new = acc_v * dt;

	// inverted rotation matrix expressed as 2 eqn
	// @url https://github.com/yinzixuan126/modified_dwa/blob/b379c01e37adc1f6414005750633b05e1a024ae5/iri_navigation/iri_akp_local_planner_companion/local_lib/src/scene_elements/robot.cpp#L91
	// projection to robot pose (dot product)
	double vv = +cos(yaw) * vel_v_new.getX() + sin(yaw) * vel_v_new.getY();
	// cross product theta x f
	double vw = -sin(yaw) * vel_v_new.getX() + cos(yaw) * vel_v_new.getY();

	// EXPERIMENTAL
	Angle ang_z_force_diff(force_dir.getRadian() - yaw);
    // strengthen rotation
	double vw_mod = vw + twist_rotation_compensation * ang_z_force_diff.getRadian();

	// logging section
	debug_print_verbose("force - x: %2.3f, y: %2.3f, th: %2.3f, mass - %2.3f\r\n",
		force.getX(), force.getY(), force.getZ(), robot_mass
	);

	debug_print_verbose("accel - x: %2.3f, y: %2.3f, th: %2.3f, vel - x: %2.3f, y: %2.3f, th: %2.3f\r\n",
		acc_v.getX(), acc_v.getY(), acc_v.getZ(),
		vel_v_new.getX(), vel_v_new.getY(), vel_v_new.getZ()
	);

	debug_print_verbose("cmd_vel 'init'  - lin.x: %2.3f, ang.z: %2.3f\r\n", vv, vw);

	// hard non-linearities section:
	// check if within limits: try to maintain path, ignoring trajectory
	if (vv > max_vel_x || vw > max_rot_vel) {
		double excess_x = vv / max_vel_x;
		double excess_z = vw / max_rot_vel;
		double shortening_factor = 1 / std::max(excess_x, excess_z);
		// shorten proportionally
		vv *= shortening_factor;
		vw *= shortening_factor;
		debug_print_warn(
			"Requested velocity not within limits! excess_x %2.3f, excess_z %2.3f, multiplier %2.3f"
			"cmd_vel 'modded' lin.x: %2.3f, ang.z: %2.3f\r\n",
			excess_x, excess_z, shortening_factor, vv, vw
		);
	}

	// trim minimum velocity to `min_vel_x`
	if (vv < min_vel_x) {
		vv = min_vel_x;
		debug_print_warn("Requested velocity is smaller than min! 'modded' lin.x %2.3f\r\n", vv);
	}

	// assign final values
	cmd_vel.setX(vv);
	cmd_vel.setY(0.0);
	cmd_vel.setZ(vw);

	debug_print_verbose(
		"cmd_vel 'final' - lin.x: %2.3f (min %2.3f, max %2.3f), ang.z: %2.3f (max %2.3f)\r\n",
		cmd_vel.getX(), min_vel_x, max_vel_x, cmd_vel.getZ(), max_rot_vel
	);
}

void HuberoPlannerROS::computeVelocityGlobal(
		const Vector& vel_local,
		const Pose& pose,
		Vector& vel_global
) {
	// slide 38 at https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture3-RobotMotion.pdf
	double yaw = pose.getYaw();
	Eigen::Matrix3d rotation_yaw_inv;
	// create a rotation matrix
	rotation_yaw_inv <<
		cos(yaw), 0, 0,
		sin(yaw), 0, 0,
		0, 0, 1;
	// compute global velocity vector
	Eigen::Vector3d vel_global_eigen = rotation_yaw_inv * vel_local.getAsEigen();
	// prepare twist expressed in global coordinates
	vel_global = Vector(vel_global_eigen);

	debug_print_verbose("velocity local  : x: %2.4f, y: %2.4f, ang_z: %2.4f   (yaw: %2.4f,  sin(yaw): %2.4f,  cos(yaw): %2.4f) \r\n",
		vel_local.getX(),
		vel_local.getY(),
		vel_local.getZ(),
		yaw,
		sin(yaw),
		cos(yaw)
	);

	debug_print_verbose("velocity global: x: %2.4f, y: %2.4f, ang_z: %2.4f \r\n",
		vel_global.getX(),
		vel_global.getY(),
		vel_global.getZ()
	);
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

void HuberoPlannerROS::computeTwist(
	const Pose& pose,
	const Vector& force,
	const Vector& robot_vel_glob,
	Vector& cmd_vel) const
{
	// call static method
	HuberoPlannerROS::computeTwist(
		pose,
		force,
		robot_vel_glob,
		cfg_->getGeneral()->sim_period,
		cfg_->getSfm()->mass,
		cfg_->getLimits()->min_vel_x,
		cfg_->getLimits()->max_vel_x,
		cfg_->getLimits()->min_vel_theta,
		cfg_->getGeneral()->twist_rotation_compensation,
		cmd_vel
	);
}

}; // namespace hubero_local_planner
