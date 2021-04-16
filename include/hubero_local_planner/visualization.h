/*
 * visualization.h
 *
 *  Created on: Mar 3, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_HUBERO_LOCAL_PLANNER_VISUALIZATION_H_
#define INCLUDE_HUBERO_LOCAL_PLANNER_VISUALIZATION_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

#include <hubero_local_planner/vis/arrow.h>
#include <hubero_local_planner/vis/grid.h>
#include <hubero_local_planner/vis/grid_force.h>
#include <hubero_local_planner/vis/line_list.h>
#include <hubero_local_planner/vis/text.h>
#include <hubero_local_planner/vis/footprint.h>
#include <hubero_local_planner/vis/point.h>

#include <hubero_local_planner/hubero_planner.h>

namespace hubero_local_planner {

class Visualization {
public:
	/**
	 *
	 * @param frame
	 * @param marker_stack_height: TODO: make it param
	 */
	Visualization(const std::string& frame, const double& marker_stack_height = 1.30);
	void initialize(ros::NodeHandle& nh);
	void reconfigure(const double& max_force);
	bool pubBounding();

	/// @section Force vector marker publishing
	bool publishForceInternal(const Vector3& pos, const Vector3& force);
	bool publishForceInteraction(const Vector3& pos, const Vector3& force);
	bool publishForceSocial(const Vector3& pos, const Vector3& force);
	bool publishForceCombined(const Vector3& pos, const Vector3& force);

	bool publishBehaviourActive(const Vector3& pos, const std::string& description);
	bool publishClosestPoints(const std::vector<Pose3>& pts);
	bool publishVelocity(const Vector3& pos_start,
			const double& angle_lin,
			const double& linear_x,
			const Vector3& pos_lin_end,
			const double& angle_ang,
			const double& angular_z
	);

	bool publishPath(const Pose3& new_pos);
	void resetPath();

	bool publishGrid(
			const Pose3& pos_current,
			HuberoPlanner& planner
	);

	bool publishRobotFootprint(
			const Pose3& pos_current,
			const RobotFootprintModelConstPtr footprint
	);

	bool publishGoalLocal(const Vector3& pos);
	bool publishGoal(const Vector3& pos);
	virtual ~Visualization() = default;

private:
	/// \brief Gap between markers along Z axis
	static constexpr double MARKER_GAP = 0.07;

	ros::Publisher pub_marker_;
	ros::Publisher pub_marker_array_;
	ros::Publisher pub_path_;
	ros::Publisher pub_closest_dist_;

	vis::Arrow marker_force_;
	vis::Text marker_behaviour_;
	vis::LineList marker_closest_pts_;
	nav_msgs::Path marker_path_;
	std_msgs::Float32 marker_closest_dist_;
	vis::GridForce marker_force_grid_;

	double marker_stack_height_;

	vis::Footprint marker_footprint_;
	vis::Point marker_point_;
};

} /* namespace hubero_local_planner */

#endif /* INCLUDE_HUBERO_LOCAL_PLANNER_VISUALIZATION_H_ */
