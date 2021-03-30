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
#include <hubero_local_planner/utils/vector_calculations.h>

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
	struct ClosestPoints {
		teb_local_planner::PoseSE2 robot;
		teb_local_planner::PoseSE2 obstacle;
	};

	BaseRobotFootprintModel(): teb_local_planner::BaseRobotFootprintModel() {
	}
	virtual ~BaseRobotFootprintModel() {
	}

	// new obstacle representation class
	virtual double calculateDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) const = 0;
	// new obstacle representation class
	virtual double estimateSpatioTemporalDistance(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle, double t) const = 0;

	// new method, non-const due to the `getInscribedRadius` call (which is not marked as const, but could be)
	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) = 0;

	// new method, non-const due to the `getInscribedRadius` call (which is not marked as const, but could be)
	virtual ClosestPoints calculateClosestPoints(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) = 0;
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

	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) {
		return obstacle->getShortestVector(current_pose.position());
	}

	virtual ClosestPoints calculateClosestPoints(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) {
		ClosestPoints pts;
		pts.robot = current_pose;
		pts.obstacle = teb_local_planner::PoseSE2(obstacle->getClosestPoint(current_pose.position()), 0.0);
		return pts;
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

	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) {
		auto v = obstacle->getShortestVector(current_pose.position());
		return v - v.normalized() * getInscribedRadius();
	}

	virtual ClosestPoints calculateClosestPoints(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) {
		ClosestPoints pts;
		pts.obstacle = teb_local_planner::PoseSE2(obstacle->getClosestPoint(current_pose.position()), 0.0);
		Eigen::Vector2d v = vector_point_to_point(current_pose.position(), pts.obstacle.position());
		// move `current_pose`, considering circular footprint's radius along vector that connects robot and obstacle representations
		pts.robot = teb_local_planner::PoseSE2(current_pose.position() + v.normalized() * this->getInscribedRadius(), current_pose.theta());
		return pts;
	}
}; // class CircularRobotFootprint


class TwoCirclesRobotFootprint: public BaseRobotFootprintModel, public teb_local_planner::TwoCirclesRobotFootprint {
public:
	TwoCirclesRobotFootprint(double front_offset, double front_radius, double rear_offset, double rear_radius)
		: teb_local_planner::TwoCirclesRobotFootprint(front_offset, front_radius, rear_offset, rear_radius),
		  // workaround
		  front_offset__(front_offset),
		  front_radius__(front_radius),
		  rear_offset__(rear_offset),
		  rear_radius__(rear_radius) {
	}
	virtual ~TwoCirclesRobotFootprint() {
	}

	// workaround
	void setParameters(double front_offset, double front_radius, double rear_offset, double rear_radius) {
		teb_local_planner::TwoCirclesRobotFootprint::setParameters(front_offset, front_radius, rear_offset, rear_radius);
		front_offset__ = front_offset;
		front_radius__ = front_radius;
		rear_offset__ = rear_offset;
		rear_radius__ = rear_radius;
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

	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) {
		// based on teb_local_planner::TwoCirclesRobotFootprint::calculateDistance
	    Eigen::Vector2d dir = current_pose.orientationUnitVec();
	    Eigen::Vector2d v_front = obstacle->getShortestVector(current_pose.position() + front_offset__ * dir);
	    v_front -= v_front.normalized() * front_radius__;
	    Eigen::Vector2d v_rear = obstacle->getShortestVector(current_pose.position() + rear_offset__ * dir);
	    v_rear -= v_rear.normalized() * rear_radius__;
	    return (v_front.norm() >= v_rear.norm()) ? v_rear : v_front;
	}

	virtual ClosestPoints calculateClosestPoints(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) {
		ClosestPoints pts;
		// based on teb_local_planner::TwoCirclesRobotFootprint::calculateDistance
		Eigen::Vector2d dir = current_pose.orientationUnitVec();
		Eigen::Vector2d obs_pt_front = obstacle->getClosestPoint(current_pose.position() + front_offset__ * dir);
		Eigen::Vector2d obs_pt_rear = obstacle->getClosestPoint(current_pose.position() + rear_offset__ * dir);

		// compute robot bounds and then select closest pair of points
		Eigen::Vector2d v = vector_point_to_point(current_pose.position() + front_offset__ * dir, obs_pt_front);
		Eigen::Vector2d robot_pt_front_front = v - v.normalized() * front_radius__;
		v = vector_point_to_point(current_pose.position() + front_offset__ * dir, obs_pt_rear);
		Eigen::Vector2d robot_pt_front_rear = v - v.normalized() * front_radius__;
		v = vector_point_to_point(current_pose.position() + rear_offset__ * dir, obs_pt_front);
		Eigen::Vector2d robot_pt_rear_front = v - v.normalized() * rear_radius__;
		v = vector_point_to_point(current_pose.position() + rear_offset__ * dir, obs_pt_rear);
		Eigen::Vector2d robot_pt_rear_rear = v - v.normalized() * rear_radius__;

		typedef std::tuple<double, Eigen::Vector2d, Eigen::Vector2d> ClosestPointsHypothesis;
		std::vector<ClosestPointsHypothesis> v_norms;
		v_norms.push_back(std::make_tuple(robot_pt_front_front.norm(), robot_pt_front_front, obs_pt_front));
		v_norms.push_back(std::make_tuple(robot_pt_front_rear.norm(), robot_pt_front_rear, obs_pt_rear));
		v_norms.push_back(std::make_tuple(robot_pt_rear_front.norm(), robot_pt_rear_front, obs_pt_front));
		v_norms.push_back(std::make_tuple(robot_pt_rear_rear.norm(), robot_pt_rear_rear, obs_pt_rear));

		std::sort(
				v_norms.begin(),
				v_norms.end(),
				[](const ClosestPointsHypothesis &left, const ClosestPointsHypothesis &right) {
					return std::get<0>(left) < std::get<0>(right);
				}
		);

		pts.obstacle = teb_local_planner::PoseSE2(std::get<2>(v_norms.at(0)), 0.0);
		pts.robot = teb_local_planner::PoseSE2(std::get<1>(v_norms.at(0)), current_pose.theta());
		return pts;
	}

protected:
	// NOTE: these were marked as private in the original implementation
	double front_offset__;
	double front_radius__;
	double rear_offset__;
	double rear_radius__;
}; // class TwoCirclesRobotFootprint


// TODO: new features are not implemented perfectly, needs verification and improvements
class LineRobotFootprint: public BaseRobotFootprintModel, public teb_local_planner::LineRobotFootprint {
public:
	LineRobotFootprint(const geometry_msgs::Point& line_start, const geometry_msgs::Point& line_end)
		: teb_local_planner::LineRobotFootprint(line_start, line_end) {
		setLine(line_start, line_end);
	}
	LineRobotFootprint(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end)
		: teb_local_planner::LineRobotFootprint(line_start, line_end) {
		setLine(line_start, line_end);
	}
	virtual ~LineRobotFootprint() {
	}

	void setLine(const geometry_msgs::Point& line_start, const geometry_msgs::Point& line_end) {
		teb_local_planner::LineRobotFootprint::setLine(line_start, line_end);
		line_start__.x() = line_start.x;
		line_start__.y() = line_start.y;
		line_end__.x() = line_end.x;
		line_end__.y() = line_end.y;
	}

	void setLine(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) {
		teb_local_planner::LineRobotFootprint::setLine(line_start, line_end);
		line_start__ = line_start;
		line_end__ = line_end;
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

	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) {
	    // based on teb_local_planner::LineRobotFootprint::calculateDistance
		Eigen::Vector2d line_start_world;
	    Eigen::Vector2d line_end_world;
	    transformToWorld(current_pose, line_start_world, line_end_world);
		return obstacle->getShortestVector(line_start_world, line_end_world);
	}

	virtual ClosestPoints calculateClosestPoints(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) {
		ClosestPoints pts;

		// TODO: this is a far-from-great solution
		Eigen::Vector2d line_start_world;
		Eigen::Vector2d line_end_world;
		transformToWorld(current_pose, line_start_world, line_end_world);
		pts.robot = current_pose;
		pts.obstacle = teb_local_planner::PoseSE2(current_pose.position() - obstacle->getShortestVector(line_start_world, line_end_world), 0.0);

		return pts;
	}
protected:
	// workaround - lack of access
	// this is a copy of teb_local_planner::LineRobotFootprint::transformToWorld private method
	void transformToWorld(const teb_local_planner::PoseSE2& current_pose, Eigen::Vector2d& line_start_world, Eigen::Vector2d& line_end_world) const {
		double cos_th = std::cos(current_pose.theta());
		double sin_th = std::sin(current_pose.theta());
		line_start_world.x() = current_pose.x() + cos_th * line_start__.x() - sin_th * line_start__.y();
		line_start_world.y() = current_pose.y() + sin_th * line_start__.x() + cos_th * line_start__.y();
		line_end_world.x() = current_pose.x() + cos_th * line_end__.x() - sin_th * line_end__.y();
		line_end_world.y() = current_pose.y() + sin_th * line_end__.x() + cos_th * line_end__.y();
	}
	// workaround - lack of access
	Eigen::Vector2d line_start__;
	Eigen::Vector2d line_end__;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // class LineRobotFootprint


class PolygonRobotFootprint: public BaseRobotFootprintModel, public teb_local_planner::PolygonRobotFootprint {
public:
	PolygonRobotFootprint(const teb_local_planner::Point2dContainer& vertices)
		: teb_local_planner::PolygonRobotFootprint(vertices) {
		setVertices(vertices);
	}
	virtual ~PolygonRobotFootprint() {
	}

	// workaround for lack of access
	void setVertices(const teb_local_planner::Point2dContainer& vertices) {
		teb_local_planner::PolygonRobotFootprint::setVertices(vertices);
		vertices__ = vertices;
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

	virtual Eigen::Vector2d calculateShortestVector(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) {
		return obstacle->getShortestVector(vertices__);
	}

	virtual ClosestPoints calculateClosestPoints(const teb_local_planner::PoseSE2& current_pose, const Obstacle* obstacle) {
		ClosestPoints pts;
		pts.robot = current_pose;
		pts.obstacle = teb_local_planner::PoseSE2(obstacle->getShortestVector(vertices__), 0.0);
		return pts;
	}

protected:
	void transformToWorld(const teb_local_planner::PoseSE2& current_pose, teb_local_planner::Point2dContainer& polygon_world) const {
		double cos_th = std::cos(current_pose.theta());
		double sin_th = std::sin(current_pose.theta());
		for (std::size_t i=0; i<vertices__.size(); ++i)
		{
			polygon_world[i].x() = current_pose.x() + cos_th * vertices__[i].x() - sin_th * vertices__[i].y();
			polygon_world[i].y() = current_pose.y() + sin_th * vertices__[i].x() + cos_th * vertices__[i].y();
		}
	}
	teb_local_planner::Point2dContainer vertices__;

}; // class PolygonRobotFootprint


}; // namespace hubero_local_planner

#endif /* INCLUDE_HUBERO_LOCAL_PLANNER_ROBOT_FOOTPRINT_MODEL_H_ */
