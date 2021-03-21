/*
 * distance_calculations.h
 *
 *  Created on: Mar 17, 2021
 *      Author: rayvburn
 */

#ifndef INCLUDE_HUBERO_LOCAL_PLANNER_EXTERNAL_DISTANCE_CALCULATIONS_H_
#define INCLUDE_HUBERO_LOCAL_PLANNER_EXTERNAL_DISTANCE_CALCULATIONS_H_

#include <hubero_local_planner/utils/teb_utils.h> // ns alias
#include <teb_local_planner/distance_calculations.h>

namespace hubero_local_planner {

// NOTE: these are modified teb_local_planner functions from `distance_calculations.h`

inline Eigen::Vector2d vector_point_to_segment_2d(
		const Eigen::Ref<const Eigen::Vector2d>& point,
		const Eigen::Ref<const Eigen::Vector2d>& line_start,
		const Eigen::Ref<const Eigen::Vector2d>& line_end
) {
	return (point - teb::closest_point_on_line_segment_2d(point, line_start, line_end));
}

inline Eigen::Vector2d vector_segment_to_segment_2d(
		const Eigen::Ref<const Eigen::Vector2d>& line1_start,
		const Eigen::Ref<const Eigen::Vector2d>& line1_end,
        const Eigen::Ref<const Eigen::Vector2d>& line2_start,
		const Eigen::Ref<const Eigen::Vector2d>& line2_end
) {
	// TODO more efficient implementation

	// check if segments intersect
	if (teb::check_line_segments_intersection_2d(line1_start, line1_end, line2_start, line2_end)) {
		return Eigen::Vector2d();
	}

	// check all 4 combinations
	std::array<Eigen::Vector2d, 4> vectors;

	vectors[0] = vector_point_to_segment_2d(line1_start, line2_start, line2_end);
	vectors[1] = vector_point_to_segment_2d(line1_end, line2_start, line2_end);
	vectors[2] = vector_point_to_segment_2d(line2_start, line1_start, line1_end);
	vectors[3] = vector_point_to_segment_2d(line2_end, line1_start, line1_end);

	int index_shortest = 0;
	double shortest = vectors[0].norm();
	for (unsigned int i = 1; i < 4; i++) {
		double len = vectors[i].norm();
		if (len < shortest) {
			shortest = len;
			index_shortest = i;
		}
	}
	return vectors[index_shortest];
}

inline Eigen::Vector2d vector_point_to_polygon_2d(const Eigen::Vector2d& point, const teb::Point2dContainer& vertices) {
	double dist = HUGE_VAL;
	Eigen::Vector2d vec;

	// the polygon is a point
	if (vertices.size() == 1) {
		return (point - vertices.front());
	}

	// check each polygon edge
	for (int i = 0; i < (int)vertices.size() - 1; ++i) {
		Eigen::Vector2d new_vec = vector_point_to_segment_2d(point, vertices.at(i), vertices.at(i+1));
		double new_vec_dist = new_vec.norm();
		if (new_vec_dist < dist) {
			dist = new_vec_dist;
			vec = new_vec;
		}
	}

	// if not a line - close polygon
	if (vertices.size() > 2) {
		Eigen::Vector2d new_vec = vector_point_to_segment_2d(point, vertices.back(), vertices.front()); // check last edge
		double new_vec_dist = new_vec.norm();
		if (new_vec_dist < dist) {
			return new_vec;
		}
	}

	return vec;
}

inline Eigen::Vector2d vector_segment_to_polygon_2d(
		const Eigen::Vector2d& line_start,
		const Eigen::Vector2d& line_end,
		const teb::Point2dContainer& vertices
) {
	double dist = HUGE_VAL;
	Eigen::Vector2d vec;

	// the polygon is a point
	if (vertices.size() == 1) {
		return vector_point_to_segment_2d(vertices.front(), line_start, line_end);
	}

	// check each polygon edge
	for (int i = 0; i < (int)vertices.size() - 1; ++i) {
		Eigen::Vector2d new_vec = vector_segment_to_segment_2d(line_start, line_end, vertices.at(i), vertices.at(i+1));
		double new_vec_dist = new_vec.norm();
		if (new_vec_dist < dist) {
			dist = new_vec_dist;
			vec = new_vec;
		}
	}

	// if not a line - close polygon
	if (vertices.size() > 2) {
		Eigen::Vector2d new_vec = vector_segment_to_segment_2d(line_start, line_end, vertices.back(), vertices.front()); // check last edge
		double new_vec_dist = new_vec.norm();
		if (new_vec_dist < dist) {
			dist = new_vec_dist;
			vec = new_vec;
		}
	}

	return vec;
}

inline Eigen::Vector2d vector_polygon_to_polygon_2d(const teb::Point2dContainer& vertices1, const teb::Point2dContainer& vertices2)
{
	double dist = HUGE_VAL;
	Eigen::Vector2d vec;

	// the polygon1 is a point
	if (vertices1.size() == 1) {
		return vector_point_to_polygon_2d(vertices1.front(), vertices2);
	}

	// check each edge of polygon1
	for (int i = 0; i < (int)vertices1.size() - 1; ++i) {
		Eigen::Vector2d new_vec = vector_segment_to_polygon_2d(vertices1[i], vertices1[i+1], vertices2);
		double new_vec_dist = new_vec.norm();
		if (new_vec_dist < dist) {
			dist = new_vec_dist;
			vec = new_vec;
		}
	}

	// if not a line - close polygon1
	if (vertices1.size() > 2) {
		Eigen::Vector2d new_vec = vector_segment_to_polygon_2d(vertices1.back(), vertices1.front(), vertices2); // check last edge
		double new_vec_dist = new_vec.norm();
		if (new_vec_dist < dist) {
			dist = new_vec_dist;
			vec = new_vec;
		}
	}

	return vec;
}

}; // namespace hubero_local_planner

#endif /* INCLUDE_HUBERO_LOCAL_PLANNER_EXTERNAL_DISTANCE_CALCULATIONS_H_ */
