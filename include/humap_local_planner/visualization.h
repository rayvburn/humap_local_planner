/*
 * visualization.h
 *
 *  Created on: Mar 3, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_HUMAP_LOCAL_PLANNER_VISUALIZATION_H_
#define INCLUDE_HUMAP_LOCAL_PLANNER_VISUALIZATION_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

#include <humap_local_planner/vis/arrow.h>
#include <humap_local_planner/vis/grid.h>
#include <humap_local_planner/vis/grid_force.h>
#include <humap_local_planner/vis/line_list.h>
#include <humap_local_planner/vis/text.h>
#include <humap_local_planner/vis/footprint.h>
#include <humap_local_planner/vis/point.h>

#include <humap_local_planner/humap_planner.h>

namespace humap_local_planner {

using namespace geometry;
// FIXME: lack of using namespace humap::geometry, but class got included properly
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
	bool publishForceInternal(const Vector& pos, const Vector& force);
	bool publishForceInteraction(const Vector& pos, const Vector& force);
	bool publishForceSocial(const Vector& pos, const Vector& force);
	bool publishForceCombined(const Vector& pos, const Vector& force);

	bool publishBehaviourActive(const Vector& pos, const std::string& description);
	bool publishClosestPoints(const std::vector<Pose>& pts_static, const std::vector<Pose>& pts_dynamic);
	bool publishVelocity(const Vector& pos_start,
			const double& angle_lin,
			const double& linear_x,
			const Vector& pos_lin_end,
			const double& angle_ang,
			const double& angular_z
	);

	bool publishPath(const Pose& new_pos);
	void resetPath();

	bool publishGrid(
			const Pose& pos_current,
			HumapPlanner& planner
	);

	bool publishRobotFootprint(
			const Pose& pos_current,
			const RobotFootprintModelConstPtr footprint
	);

	bool publishGoalInitiation(const Vector& pos);
	bool publishGoalLocal(const Vector& pos);
	bool publishGoal(const Vector& pos);
	bool publishGoalRecoveryRotateAndRecede(const Vector& pos);
	bool publishGoalRecoveryLookAround(const Vector& pos);
	bool publishPlannerState(const Vector& pos, const std::string& state);
	virtual ~Visualization() = default;

private:
	/// Evaluates whether the position can be assigned to a marker
	bool isPositionValid(const Vector& pos) const;

	/// \brief Gap between markers along Z axis
	static constexpr double MARKER_GAP = 0.07;

	ros::Publisher pub_marker_;
	ros::Publisher pub_marker_array_;
	/// \brief Separate publisher to @ref pub_marker_array_ as grid computations are quite heavy and
	/// theirs execution is not disabled by simply unticking `grid` markers namespace in rViz (this would disable
	/// all MarkerArray publications)
	ros::Publisher pub_grid_;
	ros::Publisher pub_path_;
	ros::Publisher pub_closest_dist_;

	vis::Arrow marker_force_;
	vis::Text marker_behaviour_;
	vis::LineList marker_closest_pts_static_;
	vis::LineList marker_closest_pts_dynamic_;
	nav_msgs::Path marker_path_;
	std_msgs::Float32 marker_closest_dist_;
	vis::GridForce marker_force_grid_;

	double marker_stack_height_;

	vis::Footprint marker_footprint_;
	vis::Point marker_point_;
	vis::Text marker_state_;
};

} /* namespace humap_local_planner */

#endif /* INCLUDE_HUMAP_LOCAL_PLANNER_VISUALIZATION_H_ */
