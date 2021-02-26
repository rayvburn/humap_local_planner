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
		costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
		dsrv_(nullptr),
		cfg_(nullptr),
		tf_(nullptr),
		odom_helper_("odom") {
}

HuberoPlannerROS::~HuberoPlannerROS(){
	//make sure to clean things up
	delete dsrv_;
}

void HuberoPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) {
	printf("[HuberoPlannerROS::initialize] start\r\n");

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
		obstacles_.reserve(500);

		// get parameters of HuberoConfig via the node handle and override the default config
		cfg_ = std::make_shared<HuberoConfigROS>();
		cfg_->loadFromParamServer(private_nh);

		// local planner
		printf("[HuberoPlannerROS::initialize] before planner\r\n");
		planner_ = std::make_shared<HuberoPlanner>(name, planner_util_);
		planner_->initialize(cfg_);

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
	return true;
}

bool HuberoPlannerROS::isGoalReached() {
	return false;
}

void HuberoPlannerROS::reconfigureCB(HuberoPlannerConfig &config, uint32_t level) {
	printf("HuberoPlannerROS::reconfigureCB \r\n");
	cfg_->reconfigure(config);
}

void HuberoPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
	base_local_planner::publishPlan(path, l_plan_pub_);
}

void HuberoPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
	base_local_planner::publishPlan(path, g_plan_pub_);
}

bool HuberoPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
	cmd_vel.linear.x = 0.25;
	return true;
}

}; // namespace hubero_local_planner
