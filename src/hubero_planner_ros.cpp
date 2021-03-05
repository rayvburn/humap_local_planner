#include <hubero_local_planner/hubero_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

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
		vis_("map") {
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

		// get parameters of HuberoConfig via the node handle and override the default config
		cfg_ = std::make_shared<HuberoConfigROS>();
		cfg_->loadFromParamServer(private_nh);

		// local planner
		planner_ = std::make_shared<HuberoPlanner>(name, planner_util_);
		planner_->initialize(cfg_);

		// visualization
		vis_.initialize(private_nh);
		// TODO: make it param
		vis_.reconfigure(cfg_->sfm.max_force, 1.30);

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
	return planner_->isGoalReached();
}

// based on TEB's `computeVelocityCommands`
bool HuberoPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
	if(!isInitialized()) {
		ROS_ERROR("hubero_local_planner has not been initialized, please call initialize() before using this planner");
		return false;
	}
	ROS_INFO("[HuberoPlannerROS] computeVelocityCommands()");

	// retrieve environment information and pass them to the HuBeRo planner

	// initialize with dummy data
	cmd_vel.linear.x = 0.25;
	cmd_vel.linear.y = 0;
	cmd_vel.angular.z = 0;

	// Get robot pose
	tf::Stamped<tf::Pose> robot_pose;
	costmap_ros_->getRobotPose(robot_pose);
	printf("\n\npose: x %2.4f / y %2.4f / z %2.4f / Rx %2.4f, Ry %2.4f, Rz %2.4f, Rw %2.4f \r\n",
		robot_pose.getOrigin().x(),
		robot_pose.getOrigin().y(),
		robot_pose.getOrigin().z(),
		robot_pose.getRotation().x(),
		robot_pose.getRotation().y(),
		robot_pose.getRotation().z(),
		robot_pose.getRotation().w()
	);

	// Get robot velocity
	tf::Stamped<tf::Pose> robot_vel_tf;
	odom_helper_.getRobotVel(robot_vel_tf);
	geometry_msgs::Twist robot_vel_;
	robot_vel_.linear.x = robot_vel_tf.getOrigin().getX();
	robot_vel_.linear.y = robot_vel_tf.getOrigin().getY();
	robot_vel_.angular.z = tf::getYaw(robot_vel_tf.getRotation());
	printf("vel: x %2.4f / y %2.4f / yaw %2.4f \n\n\n",
		robot_vel_.linear.x,
		robot_vel_.linear.y,
		robot_vel_.angular.z
	);

//	// prune global plan to cut off parts of the past (spatially before the robot)
//	base_local_planner::prunePlan(robot_pose, pla)
//
//	// Transform global plan to the frame of interest (w.r.t. the local costmap)
//	std::vector<geometry_msgs::PoseStamped> transformed_plan;
//	int goal_idx;
//	tf::StampedTransform tf_plan_to_global;
//	if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist,
//	transformed_plan, &goal_idx, &tf_plan_to_global))
//	{
//	ROS_WARN("Could not transform the global plan to the frame of the controller");
//	return false;
//	}
//
//	// Return false if the transformed global plan is empty
//	if (transformed_plan.empty()) {
//		ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
//		return false;
//	}

	updateObstacleContainerWithCostmapConverter();

	printf("pre-compute \r\n");
	Eigen::Vector3f force;
	planner_->compute(robot_pose, robot_vel_, obstacles_, force);

	tf::StampedTransform transform;
	tf_->lookupTransform("base_link", "map", ros::Time(0), transform);
	tf::Matrix3x3 rot(transform.getRotation());
	double roll, pitch, yaw;
	rot.getRPY(roll, pitch, yaw);
	std::cout << "\t 1) transform: x=" << transform.getOrigin().x() << " y=" << transform.getOrigin().y() << " z=" << transform.getOrigin().z() << std::endl;
	std::cout << "\t 2) transform: yaw=" << yaw << " pitch=" << pitch << " roll=" << roll << std::endl;

	// orientation of the force vector
	Vector3 force_v(force[0], force[1], force[2]);
	Angle angle_force_v(std::atan2(force_v.Normalized().Y(), force_v.Normalized().X()));
	angle_force_v.Normalize();

	Angle rel_angle(angle_force_v - Angle(yaw));
	std::cout << "rel_angle: " << rel_angle.Radian() << std::endl;

	// publishes TF with force
	tf::StampedTransform tf_force_vector;
	//tf_->lookupTransform("map", "base_link", ros::Time(0), tf_force_vector);
	tf_force_vector.stamp_ = ros::Time::now();
	tf_force_vector.frame_id_ = "base_link"; // "map";
	tf_force_vector.child_frame_id_ = "force";
//	tf::Matrix3x3 rot_f(tf_force_vector.getRotation());
//	double roll_force, pitch_force, yaw_force;
//	rot_f.getRPY(roll_force, pitch_force, yaw_force);
//	std::cout << "force TF vector: Y=" << yaw_force << " P=" << pitch_force << " R=" << roll_force << std::endl;
//	Angle force_angle_combined;
//	force_angle_combined.Radian(roll_force + rel_angle.Radian());
//	force_angle_combined.Normalize();
	tf_force_vector.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	tf_force_vector.setRotation(tf::Quaternion(0.0, 0.0, rel_angle.Radian()));
	tf_broadcaster_.sendTransform(tf_force_vector);

	//std::cout << "force vector: Y=" << force_angle_combined.Radian() << " P=" << pitch_force << " R=" << roll_force << std::endl;

	printf("post-compute \r\n");
	cmd_vel.linear.x = 0.15;

	// visualization
	auto vis_data = planner_->getMotionData();
	Vector3 pos(robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), robot_pose.getOrigin().z());
	vis_.publishForceInternal(pos, vis_data.force_internal);
	vis_.publishForceInteraction(pos, vis_data.force_interaction);
	vis_.publishForceSocial(pos, vis_data.force_social);
	vis_.publishForceCombined(pos, vis_data.force_combined);
	vis_.publishBehaviourActive(pos, vis_data.behaviour_active);
	vis_.publishClosestPoints(vis_data.closest_points);

	return true;
}

// protected
void HuberoPlannerROS::reconfigureCB(HuberoPlannerConfig &config, uint32_t level) {
	cfg_->reconfigure(config);
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
			obstacles_->push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
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

}; // namespace hubero_local_planner
