/*
 * obstacles.h
 *
 *  Created on: Mar 17, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_HUBERO_LOCAL_PLANNER_OBSTACLES_H_
#define INCLUDE_HUBERO_LOCAL_PLANNER_OBSTACLES_H_

// obtacles representation (polygons) helper
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/distance_calculations.h>

#include <hubero_local_planner/utils/vector_calculations.h>

// @brief Extension of the teb's classes to allow shortest vector calculation (instead of pure distances)
namespace hubero_local_planner {

class Obstacle: public teb_local_planner::Obstacle {
public:
	Obstacle(): teb_local_planner::Obstacle() {}
	virtual ~Obstacle() { };

	virtual Eigen::Vector2d getShortestVector(const Eigen::Vector2d& position) const = 0;
	virtual Eigen::Vector2d getShortestVector(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const = 0;
	virtual Eigen::Vector2d getShortestVector(const teb::Point2dContainer& polygon) const = 0;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // class Obstacle

//! Abbrev. for shared obstacle pointers
typedef std::shared_ptr<Obstacle> ObstaclePtr;
//! Abbrev. for shared obstacle const pointers
typedef std::shared_ptr<const Obstacle> ObstacleConstPtr;
//! Abbrev. for containers storing multiple obstacles
typedef std::vector<ObstaclePtr> ObstContainer;

// Container
typedef std::shared_ptr<ObstContainer> ObstContainerPtr;
typedef std::shared_ptr<const ObstContainer> ObstContainerConstPtr;


class PointObstacle: public Obstacle, public teb_local_planner::PointObstacle {
public:
	PointObstacle(): teb_local_planner::PointObstacle() {
	}
	PointObstacle(const Eigen::Ref< const Eigen::Vector2d>& position): teb_local_planner::PointObstacle(position) {
	}
	PointObstacle(double x, double y): teb_local_planner::PointObstacle(x, y) {
	}
	virtual ~PointObstacle() {
	};

	virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const {
		return teb_local_planner::PointObstacle::checkCollision(point, min_dist);
	}
	virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const {
		return teb_local_planner::PointObstacle::checkLineIntersection(line_start, line_end, min_dist);
	}
	virtual double getMinimumDistance(const Eigen::Vector2d& position) const {
		return teb_local_planner::PointObstacle::getMinimumDistance(position);
	}
	virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const {
		return teb_local_planner::PointObstacle::getMinimumDistance(line_start, line_start);
	}
	virtual double getMinimumDistance(const teb_local_planner::Point2dContainer& polygon) const {
		return teb_local_planner::PointObstacle::getMinimumDistance(polygon);
	}
	virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const {
		return teb_local_planner::PointObstacle::getClosestPoint(position);
	}
	virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const {
		return teb_local_planner::PointObstacle::getMinimumSpatioTemporalDistance(position, t);
	}
	virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const {
		return teb_local_planner::PointObstacle::getMinimumSpatioTemporalDistance(line_start, line_end, t);
	}
	virtual double getMinimumSpatioTemporalDistance(const teb_local_planner::Point2dContainer& polygon, double t) const {
		return teb_local_planner::PointObstacle::getMinimumSpatioTemporalDistance(polygon, t);
	}
	virtual void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const {
		return teb_local_planner::PointObstacle::predictCentroidConstantVelocity(t, position);
	}
	virtual const Eigen::Vector2d& getCentroid() const {
		return teb_local_planner::PointObstacle::getCentroid();
	}
	virtual std::complex<double> getCentroidCplx() const {
		return teb_local_planner::PointObstacle::getCentroidCplx();
	}
	virtual void toPolygonMsg(geometry_msgs::Polygon& polygon) {
		teb_local_planner::PointObstacle::toPolygonMsg(polygon);
	}

	virtual Eigen::Vector2d getShortestVector(const Eigen::Vector2d& position) const {
		return vector_point_to_point(pos_, position);
	}
	virtual Eigen::Vector2d getShortestVector(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const {
		return vector_point_to_segment_2d(pos_, line_start, line_end);
	}
	virtual Eigen::Vector2d getShortestVector(const teb::Point2dContainer& polygon) const {
		return vector_point_to_polygon_2d(pos_, polygon);
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // class PointObstacle



class CircularObstacle: public Obstacle, public teb_local_planner::CircularObstacle {
public:
	CircularObstacle(): teb_local_planner::CircularObstacle() {
	}
	CircularObstacle(const Eigen::Ref< const Eigen::Vector2d>& position, double radius):
		teb_local_planner::CircularObstacle(position, radius) {
	}
	CircularObstacle(double x, double y, double radius):
		teb_local_planner::CircularObstacle(x, y, radius) {
	}
	virtual ~CircularObstacle() {
	};

	virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const {
		return teb_local_planner::CircularObstacle::checkCollision(point, min_dist);
	}
	virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const {
		return teb_local_planner::CircularObstacle::checkLineIntersection(line_start, line_end, min_dist);
	}
	virtual double getMinimumDistance(const Eigen::Vector2d& position) const {
		return teb_local_planner::CircularObstacle::getMinimumDistance(position);
	}
	virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const {
		return teb_local_planner::CircularObstacle::getMinimumDistance(line_start, line_start);
	}
	virtual double getMinimumDistance(const teb_local_planner::Point2dContainer& polygon) const {
		return teb_local_planner::CircularObstacle::getMinimumDistance(polygon);
	}
	virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const {
		return teb_local_planner::CircularObstacle::getClosestPoint(position);
	}
	virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const {
		return teb_local_planner::CircularObstacle::getMinimumSpatioTemporalDistance(position, t);
	}
	virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const {
		return teb_local_planner::CircularObstacle::getMinimumSpatioTemporalDistance(line_start, line_end, t);
	}
	virtual double getMinimumSpatioTemporalDistance(const teb_local_planner::Point2dContainer& polygon, double t) const {
		return teb_local_planner::CircularObstacle::getMinimumSpatioTemporalDistance(polygon, t);
	}
	virtual void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const {
		return teb_local_planner::CircularObstacle::predictCentroidConstantVelocity(t, position);
	}
	virtual const Eigen::Vector2d& getCentroid() const {
		return teb_local_planner::CircularObstacle::getCentroid();
	}
	virtual std::complex<double> getCentroidCplx() const {
		return teb_local_planner::CircularObstacle::getCentroidCplx();
	}
	virtual void toPolygonMsg(geometry_msgs::Polygon& polygon) {
		teb_local_planner::CircularObstacle::toPolygonMsg(polygon);
	}

	virtual Eigen::Vector2d getShortestVector(const Eigen::Vector2d& position) const {
		auto v = vector_point_to_point(pos_, position);
		return v - v.normalized() * radius_;
	}
	virtual Eigen::Vector2d getShortestVector(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const {
		auto v = vector_point_to_segment_2d(pos_, line_start, line_end);
		return v - v.normalized() * radius_;
	}
	virtual Eigen::Vector2d getShortestVector(const teb::Point2dContainer& polygon) const {
		auto v = vector_point_to_polygon_2d(pos_, polygon);
		return v - v.normalized() * radius_;
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // class CircularObstacle


class LineObstacle: public Obstacle, public teb_local_planner::LineObstacle {
public:
	LineObstacle(): teb_local_planner::LineObstacle() {
	}
	LineObstacle(const Eigen::Ref< const Eigen::Vector2d>& line_start, const Eigen::Ref< const Eigen::Vector2d>& line_end)
		: teb_local_planner::LineObstacle(line_start, line_end) {
	}
	LineObstacle(double x1, double y1, double x2, double y2): teb_local_planner::LineObstacle(x1, y1, x2, y2) {
	}
	virtual ~LineObstacle() {
	};

	virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const {
		return teb_local_planner::LineObstacle::checkCollision(point, min_dist);
	}
	virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const {
		return teb_local_planner::LineObstacle::checkLineIntersection(line_start, line_end, min_dist);
	}
	virtual double getMinimumDistance(const Eigen::Vector2d& position) const {
		return teb_local_planner::LineObstacle::getMinimumDistance(position);
	}
	virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const {
		return teb_local_planner::LineObstacle::getMinimumDistance(line_start, line_start);
	}
	virtual double getMinimumDistance(const teb_local_planner::Point2dContainer& polygon) const {
		return teb_local_planner::LineObstacle::getMinimumDistance(polygon);
	}
	virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const {
		return teb_local_planner::LineObstacle::getClosestPoint(position);
	}
	virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const {
		return teb_local_planner::LineObstacle::getMinimumSpatioTemporalDistance(position, t);
	}
	virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const {
		return teb_local_planner::LineObstacle::getMinimumSpatioTemporalDistance(line_start, line_end, t);
	}
	virtual double getMinimumSpatioTemporalDistance(const teb_local_planner::Point2dContainer& polygon, double t) const {
		return teb_local_planner::LineObstacle::getMinimumSpatioTemporalDistance(polygon, t);
	}
	virtual void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const {
		return teb_local_planner::LineObstacle::predictCentroidConstantVelocity(t, position);
	}
	virtual const Eigen::Vector2d& getCentroid() const {
		return teb_local_planner::LineObstacle::getCentroid();
	}
	virtual std::complex<double> getCentroidCplx() const {
		return teb_local_planner::LineObstacle::getCentroidCplx();
	}
	virtual void toPolygonMsg(geometry_msgs::Polygon& polygon) {
		teb_local_planner::LineObstacle::toPolygonMsg(polygon);
	}

	virtual Eigen::Vector2d getShortestVector(const Eigen::Vector2d& position) const {
		return vector_point_to_segment_2d(position, start(), end());
	}
	virtual Eigen::Vector2d getShortestVector(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const {
		return vector_segment_to_segment_2d(start(), end(), line_start, line_end);
	}
	virtual Eigen::Vector2d getShortestVector(const teb::Point2dContainer& polygon) const {
		return vector_segment_to_polygon_2d(start(), end(), polygon);
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // class LineObstacle


class PolygonObstacle: public Obstacle, public teb_local_planner::PolygonObstacle {
public:
	PolygonObstacle(): teb_local_planner::PolygonObstacle() {
	}
	PolygonObstacle(const teb_local_planner::Point2dContainer& vertices): teb_local_planner::PolygonObstacle(vertices) {
	}
	virtual ~PolygonObstacle() {
	};

	virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const {
		return teb_local_planner::PolygonObstacle::checkCollision(point, min_dist);
	}
	virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist=0) const {
		return teb_local_planner::PolygonObstacle::checkLineIntersection(line_start, line_end, min_dist);
	}
	virtual double getMinimumDistance(const Eigen::Vector2d& position) const {
		return teb_local_planner::PolygonObstacle::getMinimumDistance(position);
	}
	virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const {
		return teb_local_planner::PolygonObstacle::getMinimumDistance(line_start, line_start);
	}
	virtual double getMinimumDistance(const teb_local_planner::Point2dContainer& polygon) const {
		return teb_local_planner::PolygonObstacle::getMinimumDistance(polygon);
	}
	virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const {
		return teb_local_planner::PolygonObstacle::getClosestPoint(position);
	}
	virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const {
		return teb_local_planner::PolygonObstacle::getMinimumSpatioTemporalDistance(position, t);
	}
	virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const {
		return teb_local_planner::PolygonObstacle::getMinimumSpatioTemporalDistance(line_start, line_end, t);
	}
	virtual double getMinimumSpatioTemporalDistance(const teb_local_planner::Point2dContainer& polygon, double t) const {
		return teb_local_planner::PolygonObstacle::getMinimumSpatioTemporalDistance(polygon, t);
	}
	virtual void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const {
		return teb_local_planner::PolygonObstacle::predictCentroidConstantVelocity(t, position);
	}
	virtual const Eigen::Vector2d& getCentroid() const {
		return teb_local_planner::PolygonObstacle::getCentroid();
	}
	virtual std::complex<double> getCentroidCplx() const {
		return teb_local_planner::PolygonObstacle::getCentroidCplx();
	}
	virtual void toPolygonMsg(geometry_msgs::Polygon& polygon) {
		teb_local_planner::PolygonObstacle::toPolygonMsg(polygon);
	}

	virtual Eigen::Vector2d getShortestVector(const Eigen::Vector2d& position) const {
		return vector_point_to_polygon_2d(position, vertices_);
	}
	virtual Eigen::Vector2d getShortestVector(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const {
		return vector_segment_to_polygon_2d(line_start, line_end, vertices_);
	}
	virtual Eigen::Vector2d getShortestVector(const teb::Point2dContainer& polygon) const {
		return vector_polygon_to_polygon_2d(polygon, vertices_);
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; // class PolygonObstacle

}; // namespace hubero_local_planner

#endif /* INCLUDE_HUBERO_LOCAL_PLANNER_OBSTACLES_H_ */
