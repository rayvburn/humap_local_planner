/*
 * robot_footprint_model.h
 *
 *  Created on: Mar 24, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_HUBERO_LOCAL_PLANNER_ROBOT_FOOTPRINT_MODEL_H_
#define INCLUDE_HUBERO_LOCAL_PLANNER_ROBOT_FOOTPRINT_MODEL_H_

#include <teb_local_planner/robot_footprint_model.h>
#include <hubero_local_planner/obstacles.h>

namespace hubero_local_planner {
/**
 * @details Extension of TEB's footprint representation. Luckily, that there is no copying of code, but the main
 * drawback is that each class must implement virtual methods separately (they are not "linked" from non-virtual
 * class, which stands as a base of the specific footprint)
 *
 * What's strange is that not all classes required
 * ```cpp
 * 	public:
 * 		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 * ```
 * in fact only `LineRobotFootprint` did.
 */
class BaseRobotFootprintModel: public teb_local_planner::BaseRobotFootprintModel {
public:
	BaseRobotFootprintModel(): teb_local_planner::BaseRobotFootprintModel() {
	}
	virtual ~BaseRobotFootprintModel() {
	}

	// new obstacle representation class
	virtual double calculateDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const = 0;
	// new obstacle representation class
	virtual double estimateSpatioTemporalDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle, double t) const = 0;

	// new method
	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const = 0;
	// new method
	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle) const = 0;
}; // class BaseRobotFootprintModel

//! Abbrev. for shared obstacle pointers
typedef std::shared_ptr<BaseRobotFootprintModel> RobotFootprintModelPtr;
//! Abbrev. for shared obstacle const pointers
typedef std::shared_ptr<const BaseRobotFootprintModel> RobotFootprintModelConstPtr;


class PointRobotFootprint: public BaseRobotFootprintModel, public teb_local_planner::PointRobotFootprint {
public:
	PointRobotFootprint(): teb_local_planner::PointRobotFootprint() {
	}
	virtual ~PointRobotFootprint() {
	}

	virtual double calculateDistance(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle) const {
		return teb_local_planner::PointRobotFootprint::calculateDistance(current_pose, obstacle);
	}
	virtual double calculateDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const {
		return calculateDistance(current_pose, obstacle);
	}

	virtual double estimateSpatioTemporalDistance(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle, double t) const {
		return teb_local_planner::PointRobotFootprint::estimateSpatioTemporalDistance(current_pose, obstacle, t);
	}
	virtual double estimateSpatioTemporalDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle, double t) const {
		return estimateSpatioTemporalDistance(current_pose, obstacle, t);
	}

	virtual double getInscribedRadius() {
		return teb_local_planner::PointRobotFootprint::getInscribedRadius();
	}

	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle) const {
		return Eigen::Vector2d();
	}
	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const {
		return calculateShortestVector(current_pose, obstacle);
	}
}; // class PointRobotFootprint


class CircularRobotFootprint: public BaseRobotFootprintModel, public teb_local_planner::CircularRobotFootprint {
public:
	CircularRobotFootprint(double radius): teb_local_planner::CircularRobotFootprint(radius) {
	}
	virtual ~CircularRobotFootprint() {
	}

	virtual double calculateDistance(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle) const {
		return teb_local_planner::CircularRobotFootprint::calculateDistance(current_pose, obstacle);
	}
	virtual double calculateDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const {
		return calculateDistance(current_pose, obstacle);
	}

	virtual double estimateSpatioTemporalDistance(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle, double t) const {
		return teb_local_planner::CircularRobotFootprint::estimateSpatioTemporalDistance(current_pose, obstacle, t);
	}
	virtual double estimateSpatioTemporalDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle, double t) const {
		return estimateSpatioTemporalDistance(current_pose, obstacle, t);
	}

	virtual double getInscribedRadius() {
		return teb_local_planner::CircularRobotFootprint::getInscribedRadius();
	}

	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle) const {
		return Eigen::Vector2d();
	}
	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const {
		return calculateShortestVector(current_pose, obstacle);
	}
}; // class CircularRobotFootprint


class TwoCirclesRobotFootprint: public BaseRobotFootprintModel, public teb_local_planner::TwoCirclesRobotFootprint {
public:
	TwoCirclesRobotFootprint(double front_offset, double front_radius, double rear_offset, double rear_radius)
		: teb_local_planner::TwoCirclesRobotFootprint(front_offset, front_radius, rear_offset, rear_radius) {
	}
	virtual ~TwoCirclesRobotFootprint() {
	}

	virtual double calculateDistance(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle) const {
		return teb_local_planner::TwoCirclesRobotFootprint::calculateDistance(current_pose, obstacle);
	}
	virtual double calculateDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const {
		return calculateDistance(current_pose, obstacle);
	}

	virtual double estimateSpatioTemporalDistance(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle, double t) const {
		return teb_local_planner::TwoCirclesRobotFootprint::estimateSpatioTemporalDistance(current_pose, obstacle, t);
	}
	virtual double estimateSpatioTemporalDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle, double t) const {
		return estimateSpatioTemporalDistance(current_pose, obstacle, t);
	}

	virtual double getInscribedRadius() {
		return teb_local_planner::TwoCirclesRobotFootprint::getInscribedRadius();
	}

	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle) const {
		return Eigen::Vector2d();
	}
	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const {
		return calculateShortestVector(current_pose, obstacle);
	}
}; // class TwoCirclesRobotFootprint


class LineRobotFootprint: public BaseRobotFootprintModel, public teb_local_planner::LineRobotFootprint {
public:
	LineRobotFootprint(const geometry_msgs::Point& line_start, const geometry_msgs::Point& line_end)
		: teb_local_planner::LineRobotFootprint(line_start, line_end) {
	}
	LineRobotFootprint(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end)
		: teb_local_planner::LineRobotFootprint(line_start, line_end) {
	}
	virtual ~LineRobotFootprint() {
	}

	virtual double calculateDistance(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle) const {
		return teb_local_planner::LineRobotFootprint::calculateDistance(current_pose, obstacle);
	}
	virtual double calculateDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const {
		return calculateDistance(current_pose, obstacle);
	}

	virtual double estimateSpatioTemporalDistance(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle, double t) const {
		return teb_local_planner::LineRobotFootprint::estimateSpatioTemporalDistance(current_pose, obstacle, t);
	}
	virtual double estimateSpatioTemporalDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle, double t) const {
		return estimateSpatioTemporalDistance(current_pose, obstacle, t);
	}

	virtual double getInscribedRadius() {
		return teb_local_planner::LineRobotFootprint::getInscribedRadius();
	}

	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle) const {
		return Eigen::Vector2d();
	}
	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const {
		return calculateShortestVector(current_pose, obstacle);
	}
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // class LineRobotFootprint


class PolygonRobotFootprint: public BaseRobotFootprintModel, public teb_local_planner::PolygonRobotFootprint {
public:
	PolygonRobotFootprint(const teb_local_planner::Point2dContainer& vertices)
		: teb_local_planner::PolygonRobotFootprint(vertices) {
	}
	virtual ~PolygonRobotFootprint() {
	}

	virtual double calculateDistance(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle) const {
		return teb_local_planner::PolygonRobotFootprint::calculateDistance(current_pose, obstacle);
	}
	virtual double calculateDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const {
		return calculateDistance(current_pose, obstacle);
	}

	virtual double estimateSpatioTemporalDistance(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle, double t) const {
		return teb_local_planner::PolygonRobotFootprint::estimateSpatioTemporalDistance(current_pose, obstacle, t);
	}
	virtual double estimateSpatioTemporalDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle, double t) const {
		return estimateSpatioTemporalDistance(current_pose, obstacle, t);
	}

	virtual double getInscribedRadius() {
		return teb_local_planner::PolygonRobotFootprint::getInscribedRadius();
	}

	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const teb_local_planner::Obstacle* obstacle) const {
		return Eigen::Vector2d();
	}
	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const {
		return calculateShortestVector(current_pose, obstacle);
	}
}; // class PolygonRobotFootprint


}; // namespace hubero_local_planner

#endif /* INCLUDE_HUBERO_LOCAL_PLANNER_ROBOT_FOOTPRINT_MODEL_H_ */
