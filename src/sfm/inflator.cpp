/*
 * Inflator.cpp
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#include <humap_local_planner/sfm/inflator.h>
#include <algorithm> // std::min_element

#include <humap_local_planner/sfm/sfm_debug.h>

namespace humap_local_planner {
namespace sfm {

// ------------------------------------------------------------------- //

Inflator::Inflator() { }

// ------------------------------------------------------------------- //

ignition::math::Vector3d Inflator::findClosestPointsModelBox(const ignition::math::Pose3d &actor_pose,
		const ignition::math::Pose3d &object_pose, const inflation::Border &bb)
{

	// TODO: check operation
	// it doesn't take ACTOR's BOUNDING BOX into consideration (only object's BB)

	/* Assuming axis-aligned bounding box - closest point search algorithm:
	 * 1st case:
	 * 		o 	check whether actor's y-coordinate-defined line intersects the bounding box - if yes, then the y-coordinate
	 * 			of the closest point is already known and x-coordinate will be located on the closest to actor
	 * 			edge of a bounding box
	 * 2nd case:
	 * 		o 	analogical to 1st one but first check is connected with x-coordinate intersection with bounding box
	 * 3rd case:
	 * 		o 	none of actor's coordinates intersect the bounding box - let's check 4 vertices (assuming on-plane
	 * 			check) and choose the closest one
	 */

	// inf has an object with no bounding box defined (for example - actor)
	if ( std::fabs(bb.getCenter().X()) > 1e+300 ) {
		// another actor met
		return ( ignition::math::Vector3d(object_pose.Pos()) );
	}

	// BB's Intersect() method returns a tuple
	bool does_intersect = false;
	double dist_intersect = 0.0;
	ignition::math::Vector3d point_intersect;

	// 0 case  -------------------------------------------------------------------
	// if actor stepped into some obstacle then force its central point to be the closest - WIP
	// otherwise actor will not see the obstacle (BEHIND flag will be raised)
	if ( bb.doesContain(actor_pose.Pos()) ) {
		return (bb.getCenter());
	}

	// 1st case -------------------------------------------------------------------
	ignition::math::Line3d line;
	// create a line of which intersection with a bounding box will be checked, syntax: x1, y1, x2, y2, z_common
//	line.Set(-1e+50, actor_pose.Pos().Y(), +1e+50, actor_pose.Pos().Y(), bb.Center().Z() );
	line.Set(actor_pose.Pos().X(), actor_pose.Pos().Y(), bb.getCenter().X(), actor_pose.Pos().Y(), bb.getCenter().Z() );
	std::tie(does_intersect, point_intersect) = bb.doesIntersect(line);

	if ( does_intersect ) {
		return (point_intersect);
	}

	// 2nd case -------------------------------------------------------------------
//	line.Set(actor_pose.Pos().X(), -1e+50, actor_pose.Pos().X(), +1e+50, bb.Center().Z() );
	line.Set(actor_pose.Pos().X(), actor_pose.Pos().Y(), actor_pose.Pos().X(), bb.getCenter().Y(), bb.getCenter().Z() );
	std::tie(does_intersect, point_intersect) = bb.doesIntersect(line);

	if ( does_intersect ) {
		return (point_intersect);
	}

	// 3rd case -------------------------------------------------------------------
	return (findClosestBoundingBoxVertex(actor_pose.Pos(), bb));

}

// ------------------------------------------------------------------- //

std::vector<ignition::math::Vector3d> Inflator::createVerticesVector(const ignition::math::Box &bb) {

	// 4 vertices only (planar)
	std::vector<ignition::math::Vector3d> temp_container;
	ignition::math::Vector3d temp_vector;

	// planar objects considered, height does not matter
	// as long as it is the same for both vector points
	temp_vector.Z(bb.Center().Z());

	temp_vector.X(bb.Min().X()); 	temp_vector.Y(bb.Min().Y());
	temp_container.push_back(temp_vector);

	temp_vector.X(bb.Min().X()); 	temp_vector.Y(bb.Max().Y());
	temp_container.push_back(temp_vector);

	temp_vector.X(bb.Max().X()); 	temp_vector.Y(bb.Min().Y());
	temp_container.push_back(temp_vector);

	temp_vector.X(bb.Max().X()); 	temp_vector.Y(bb.Max().Y());
	temp_container.push_back(temp_vector);

	// emplace_back() makes system stuck completely
	return (temp_container);

}

// ------------------------------------------------------------------- //

std::vector<double> Inflator::calculateLengthToVertices(const ignition::math::Vector3d &actor_pos,
		const std::vector<ignition::math::Vector3d> &vertices_pts)
{

	std::vector<double> temp_containter;
	for ( size_t i = 0; i < vertices_pts.size(); i++ ) {
		// planar
		ignition::math::Vector3d v_plane = vertices_pts.at(i) - actor_pos; 	v_plane.Z(0.0);
		temp_containter.push_back(v_plane.Length());
	}
	return (temp_containter);

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> Inflator::findClosestPointsBoxes(
		const ignition::math::Pose3d &actor_pose,  const inflation::Border &actor_box,
		const ignition::math::Pose3d &object_pose, const inflation::Border &object_box,
		const std::string &_object_name /* debug only */ )
{

	// method used for `d_alpha_beta` calculation
	BoxesSurfaceIntersection intersection_status = BOXES_SURFACE_INTERSECTION_NONE;
	BoxIntersectionType x_box_intersection, y_box_intersection = BOX_INTERSECTION_TYPE_UNKNOWN;

	ignition::math::Vector3d pt_actor = actor_pose.Pos();
	ignition::math::Pose3d pose_actor_new = actor_pose;
	ignition::math::Vector3d pt_obstacle = object_pose.Pos();

	std::tie(intersection_status, x_box_intersection, y_box_intersection) = findIntersectionTypeOfBB(actor_box.getBox(), object_box.getBox());

	switch (intersection_status) {

	case(BOXES_SURFACE_INTERSECTION_NONE):
			// choose a vertex that is the closest to the obstacle
			std::tie(pt_actor, pt_obstacle) = findClosestVerticesIntersectedBoxes(actor_box.getBox(), object_box.getBox());
			return (std::make_tuple(ignition::math::Pose3d(pt_actor.X(), pt_actor.Y(), pt_actor.Z(), 0.0, 0.0, 0.0), pt_obstacle));
			break;
	case(BOXES_SURFACE_INTERSECTION_PARTIAL):
			// actor has stepped into the obstacle space;
			// this will find a sub-optimal (in terms of length) vector which connects actor and obstacle centers
			std::tie(pose_actor_new, pt_obstacle) = findClosestPointsActorBox(actor_pose, actor_box, object_pose, object_box, _object_name);
			return (std::make_tuple(pose_actor_new, pt_obstacle));
			break;
	case(BOXES_SURFACE_INTERSECTION_SPECIAL):
			// the case related to the configuration in which the shortest vector
			// connecting obstacle and actor spaces is not generated from 2 vertices
			// but rather from 2 points lying somewhere along the border side
			std::tie(pt_actor, pt_obstacle) = findVectorSpecialIntersectionBB(x_box_intersection, y_box_intersection, actor_box.getBox(), object_box.getBox());
			return (std::make_tuple(ignition::math::Pose3d(pt_actor.X(), pt_actor.Y(), pt_actor.Z(), 0.0, 0.0, 0.0), pt_obstacle));
			break;

	}
	return (std::make_tuple(actor_pose, object_pose.Pos()));

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> Inflator::findIntersectedModelsClosestPoints(
		const ignition::math::Vector3d &actor_pos, const ignition::math::Vector3d &pt_intersect,
		const IntersectionType &type)
{

	/*
	 * APPLIES ESPECIALLY to 2 actors case:
	 * While actors are moving towards each other the distance between their extreme points
	 * (those which intersect bounding boxes/circles) gets smaller. At some point it will
	 * become so small that 2 bounding boxes/circles will intersect. This creates some
	 * trouble connected with calculating the distance between closest points.
	 *
	 * The main issue there will be related with RelativeLocation of an object - it will likely
	 * switch from (for example) RIGHT to LEFT which in turn will create a switch in perpendicular
	 * vector direction too (RelativeLocation is calculated based on d_alpha_beta which relies on
	 * extreme points position - bingo!)
	 *
	 * This function prevents from accidental switch of Relative Location which produces sticking
	 * of 2 objects located near to each other. It will calculate the closest points (extreme)
	 * position as settled just around the half distance between 2 objects (their centers -
	 * to be specific).
	 */

#ifdef DEBUG_ACTORS_BOUNDING_CIRCLES_LENGTH_FIX_BB
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\nWARNING - " << SfmDebugGetCurrentActorName() << "'s bounding circle IS INTERSECTING " << SfmDebugGetCurrentObjectName() << "'s bounding BOX!" << std::endl;
	}
#endif

	// line from the actor's center to the point of intersection
	ignition::math::Line3d line_actor_intersection;
	line_actor_intersection.Set(ignition::math::Vector3d(actor_pos.X(),    actor_pos.Y(),    0.00),
			 	 	 	 	 	ignition::math::Vector3d(pt_intersect.X(), pt_intersect.Y(), 0.00));

	// calculate the slope of the created line
	ignition::math::Angle line_slope( std::atan2( line_actor_intersection.Direction().Y(), line_actor_intersection.Direction().X() ) );
	line_slope.Normalize(); // just in case

	/* locate the actor_shifted just around the intersection point (this will force
	 * a very short distance between the actor and the object) */
	ignition::math::Vector3d actor_shifted = actor_pos;


#ifdef DEBUG_ACTORS_BOUNDING_CIRCLES_LENGTH_FIX_BB
	if ( SfmDebugGetCurrentActorName() == "actor1" ) {
		std::cout << "\tLINE    slope: " << line_slope.Radian() << "\tdir: " << line_actor_intersection.Direction() << std::endl;
		std::cout << "\t\tlength: " << line_actor_intersection.Length() << "\tsin: " << sin(line_slope.Radian()) << "\tcos: " << cos(line_slope.Radian()) << std::endl;
		//std::cout << "\tINITIAL shift\tactor: " << actor_pose_shifted.Pos() << "\tobject: " <<  object_pos_shifted << std::endl;
		std::cout << "\tMODDED  shift\tactor: " << actor_shifted << "\tobject: " <<  _bb_intersection_pt << std::endl;
		std::cout << "\tDIST VECTOR: " << _bb_intersection_pt-actor_shifted << "\tlen: " << (_bb_intersection_pt-actor_shifted).Length() << std::endl;
		std::cout << std::endl;
	}
#endif

	// needed for 2 actors case
	ignition::math::Vector3d object_shifted = pt_intersect;

	// save length from actor center to intersection point (with bounding box)
	double length = line_actor_intersection.Length();

	// depending on `type` another repulsion distance will be considered
	// to maximize repulsion strength
	double max_repulsion_dist = 0.0;

	switch(type) {

	case(INTERSECTION_ACTORS):

			max_repulsion_dist = 0.1;
			/* Try to keep some threshold distance between intersected models,
			 * this is just a hack for a smoother SFM operation */
			if ( length > max_repulsion_dist ) {
				// keep distance long enough
				length = (length - max_repulsion_dist / length) / 2.0;
			} else {
				// unable to keep the distance long enough
				length *= 0.495;
			}

			/* Below is OK under assumption that both bounding `boxes` have the same shape and dimensions
			 * 1) 	create a line which divides the connection into 2 parts
			 * 2) 	re-assign shifted points - place then just around the center of the line
			 * 		to force very small distance between objects to strengthen their repulsion */
			actor_shifted.X( actor_pos.X() + length * cos(line_slope.Radian() ));
			actor_shifted.Y( actor_pos.Y() + length * sin(line_slope.Radian() ));
			object_shifted.X( pt_intersect.X() - length * cos(line_slope.Radian() ));
			object_shifted.Y( pt_intersect.Y() - length * sin(line_slope.Radian() ));
			return (std::make_tuple(actor_shifted, object_shifted));
			break;

	case(INTERSECTION_ACTOR_OBJECT):

			/* initially a factor was 0.97 but the smaller the distance between actor and an obstacle
			 * the smaller repulsion is produced; when the repulsion in small distances from an obstacle
			 * is too weak then the factor should be a little smaller  */

			/* NOTE1: `line_actor_intersection` is a vector connecting actor center position
			 * and object's extreme point.
			 * NOTE2: the `actor_shifted` vector is related to a point within actor bounding.
			 * A vector generated by a difference between this point and object's extreme point
			 * creates a new d_alpha_beta vector (effect of the `inflation` procedure).
			 * NOTE3: the bigger the coefficient near `line_actor_intersection.Length()`
			 * the shorter vector (d_alpha_beta) is generated.
			 * NOTE4: In general, a shorter vector not necessarily provides a stronger repulsion.
			 * The coefficient near `line_actor_intersection.Length()` is set to provide the strongest
			 * repulsion possible when models intersecting slightly.
			 * Exemplary LOG (last value is the coefficient):
			 * 	chair_1_clone: 	21.273 110.973 0	len: 112.993	dist: 0.0799756	model_type: 3	0.90
			 *	chair_1_clone: 	89.7493 505.829 0	len: 513.729	dist: 0.319737	model_type: 3	0.60
			 *	chair_1_clone: 	83.1285 496.072 0	len: 502.989	dist: 0.479551	model_type: 3	0.45
			 *	chair_1_clone: 	82.5894 365.857 0	len: 375.064	dist: 0.199734	model_type: 3	0.75
			 *	chair_1_clone: 	89.3611 495.877 0	len: 503.865	dist: 0.30368	model_type: 3	0.62
			 *	chair_1_clone: 	104.151 517.634 0	len: 528.008	dist: 0.35971	model_type: 3	0.55
			 *	chair_1_clone: 	88.3657 522.637 0	len: 530.055	dist: 0.391876	model_type: 3	0.51
			 * This applies to the circular bounding with radius of 0.8 m. For different size the coefficient
			 * will not be optimal but still should work properly.
			 */

			// based on logged data (see above) a distance with a strongest repulsion
			// is maintained (if possible).
			// NOTE: this `case` is connected with another way of interaction calculation
			// thus `length` is different than in the previous `case`
			//
			// FIXME: Experimental version, aim is to generate the strongest repulsion
			// possible when even slightly stepped into an obstacle.
			//
			max_repulsion_dist = 0.3; // 0.4

			if ( length > max_repulsion_dist ) {
				// keep distance long enough
				length = (length - max_repulsion_dist / length) / 2.0;
				actor_shifted.X( actor_pos.X() + length * cos(line_slope.Radian() ));
				actor_shifted.Y( actor_pos.Y() + length * sin(line_slope.Radian() ));
			} else {
				// Unable to keep the distance long enough.
				// Calculate how far should be shifted to maintain given distance.
				length = max_repulsion_dist - length;
				// Shift actor outwards object position (see sign below).
				actor_shifted.X( actor_pos.X() - length * cos(line_slope.Radian() ));
				actor_shifted.Y( actor_pos.Y() - length * sin(line_slope.Radian() ));
			}

//			// FIXME: just debugging
//			std::cout << "\tINTERSECTION_ACTOR_OBJECT | actor_pos: " << actor_pos << "\t\tobject_edge_pos: " << pt_intersect << std::endl;
//			std::cout << "\tINTERSECTION_ACTOR_OBJECT | actor_center-object_edge len: " << line_actor_intersection.Length() << "\t0.1x: " << 0.1 * line_actor_intersection.Length() << "\t\t0.9x: " << 0.9 * line_actor_intersection.Length() << std::endl;
//			std::cout << "\tINTERSECTION_ACTOR_OBJECT | actor_shifted-object_edge len: " << (pt_intersect - actor_shifted).Length() << std::endl;
//			std::cout << "\tINTERSECTION_ACTOR_OBJECT | actor_shifted: " << actor_shifted << std::endl;
			return (std::make_tuple(actor_shifted, pt_intersect));
			break;

	default:
		// this should never happen!
		return (std::make_tuple(actor_shifted, pt_intersect));

	}

}

// ------------------------------------------------------------------- //

std::tuple<bool, ignition::math::Vector3d> Inflator::isWithinRangeOfBB(
		const ignition::math::Vector3d &actor_center,
		const inflation::Box &object_bb)
{

	/* Using the fact that in Gazebo, the Bounding Box is axis-aligned
	 * so it is never rotated around the world Z axis.
	 * NOTE: actor's both X and Y coordinates will be within the BB range
	 * only if actor's center is located within object bounds. */

	// stores the end point of the `intersection line`
	ignition::math::Vector3d intersection_end = object_bb.getCenter();

	bool within_x = false;
	if ( actor_center.X() >= object_bb.getMin().X() && actor_center.X() <= object_bb.getMax().X() ) {
		within_x = true;
		intersection_end.X(actor_center.X());
	}

	bool within_y = false;
	if ( actor_center.Y() >= object_bb.getMin().Y() && actor_center.Y() <= object_bb.getMax().Y() ) {
		within_y = true;
		intersection_end.Y(actor_center.Y());
	}

	if ( (within_x && within_y) || (!within_x && !within_y) ) {
		// actor stepped onto an obstacle,
		// a procedure for this case is already implemented;
		// return unmodified center position of the object's BB
		return (std::make_tuple(false, intersection_end));
	}

	// some gap between actor and object is present - desired configuration
	return (std::make_tuple(true, intersection_end));

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Inflator::calculateBoxIntersection(const ignition::math::Vector3d &object_pos,
		const inflation::Border &box,
		const ignition::math::Vector3d &box_pos) {

	/* The algorithm of searching the closest points of 2 bounding boxes (presented below)
	 * consists of creation of a line starting in the 1st bounding box's center,
	 * ending in the 2nd bb's center; then the Intersects method of the Box class
	 * will be used to find 2 points creating the shortest distance between boxes */

	/* no error handling as there should always be an intersection found
	 * when 2 center points from each bounding box are connected by a line */

	/* the only exception is calculating the social force for visualization -
	 * in such a case there is a very high possibility of being unable to
	 * finding intersection */

	// will be likely updated later (return value)
	ignition::math::Vector3d box_intersection(box.getCenter().X(), box.getCenter().Y(), 0.0);

	// by default use box center position as line end point
	ignition::math::Vector3d box_pt = box.getCenter();

	// check if box_pos is valid number or NaN (default, see header file)
	if ( !std::isnan(box_pos.X()) ) {
		if ( box.doesContain(box_pos) ) {
			box_pt = box_pos;
		}
	}

	/* create a line from the object center to the box position and check
	 * the intersection point of that line with the object's bounding box */

	/* NOTE1: some models have their centers not matched with bounding box'es center
	 * which may produce false `intersects` flag. Let's stick to bounding
	 * box'es center (instead of `object_pose` */

	/* NOTE2: line direction is important. The line given by `A` point (start)
	 * and `B` point (end point) must cross the BoundingBox so the end
	 * point is located within BB bounds (direction INTO the box). */
	ignition::math::Line3d line;
	line.Set(object_pos.X(), object_pos.Y(), box_pt.X(), box_pt.Y(), box_pt.Z() );

	// find intersection point
	bool intersects = false;
	ignition::math::Vector3d point_intersect;
	std::tie(intersects, point_intersect) = box.doesIntersect(line);

	// check if intersection point was found - it always should
	// if `object_pos` is not located within BB bounds
	if ( intersects ) {
		box_intersection = point_intersect;
	}

	return (box_intersection);

}

// ------------------------------------------------------------------- //

ignition::math::Vector3d Inflator::findClosestBoundingBoxVertex(const ignition::math::Vector3d &actor_pos,
		const inflation::Border &object_box) {

	std::vector<ignition::math::Vector3d> vertices_vector = createVerticesVector(object_box.getBox());
	std::vector<double> lengths_vector = calculateLengthToVertices(actor_pos, vertices_vector);
	auto shortest = std::min_element(lengths_vector.begin(), lengths_vector.end());

	if ( shortest == lengths_vector.end() ) {
		std::cout << "[ERROR] sfm::core::Inflator::findClosestBoundingBoxVertex" << std::endl;
	}

	// convert iterator to vector index
	size_t index = std::distance(lengths_vector.begin(), shortest);
	return (vertices_vector.at(index));

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> Inflator::findClosestPointsCircles(
		const ignition::math::Pose3d &actor_pose,
		const inflation::Border &actor_circle,
		const ignition::math::Pose3d &object_pose,
		const inflation::Border &object_circle,
		const std::string &_object_name /* debug only */)
{

	/* BoundingCircle and BoundingCircle -> 2 actors */
	// intersection of the 1st actor's circle (currently processed)
	ignition::math::Pose3d actor_pose_shifted = actor_pose;
	std::tie(std::ignore, actor_pose_shifted.Pos()) = actor_circle.doesIntersect(object_pose.Pos());

	// intersection of the 2nd actor's circle (another one)
	ignition::math::Vector3d object_pos_shifted;
	std::tie(std::ignore, object_pos_shifted) = object_circle.doesIntersect(actor_pose.Pos());

	/* Let's check whether the bounding circles are not intruding each other -
	 * compare the length of a vector from actor's center to the object's position
	 * shifted. When it's longer than radius then both bounding circles are safe -
	 * they are not intersecting each other. */
	if ( actor_circle.doesContain(object_pos_shifted) ) {
		std::tie(actor_pose_shifted.Pos(), object_pos_shifted) = findIntersectedModelsClosestPoints(actor_pose.Pos(), object_pose.Pos(), INTERSECTION_ACTORS);
	}

	return ( std::make_tuple(actor_pose_shifted, object_pos_shifted) );

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Pose3d, ignition::math::Vector3d> Inflator::findClosestPointsEllipses(
		const ignition::math::Pose3d &actor_pose,
		const inflation::Border &actor_ellipse,
		const ignition::math::Pose3d &object_pose,
		const inflation::Border &object_ellipse,
		const std::string &object_name /* debug only */ )
{

	/* BoundingEllipse and BoundingEllipse -> 2 actors */
	// intersection of the 1st actor's circle (currently processed)
	ignition::math::Pose3d actor_pose_shifted = actor_pose;

	std::tie(std::ignore, actor_pose_shifted.Pos()) = actor_ellipse.doesIntersect(object_pose.Pos());

	// intersection of the 2nd actor's circle (another one)
	ignition::math::Vector3d object_pos_shifted;

	// current actor ellipse's shifted center ( = actor's pos) is connected with object ellipse's center; the connector is a line
	std::tie(std::ignore, object_pos_shifted) = object_ellipse.doesIntersect(actor_pose.Pos());

	/* Let's check whether the bounding circles are not intruding each other -
	 * compare the length of a vector from actor's center to the object's position
	 * shifted. When it's longer than radius then both bounding circles are safe -
	 * they are not intersecting each other. */

	if ( actor_ellipse.doesContain(object_pos_shifted) ) {
		std::tie(actor_pose_shifted.Pos(), object_pos_shifted) = findIntersectedModelsClosestPoints(actor_pose.Pos(), object_pose.Pos(), INTERSECTION_ACTORS);
	}

	return ( std::make_tuple(actor_pose_shifted, object_pos_shifted) );

}

// ------------------------------------------------------------------- //

std::tuple<Inflator::BoxesSurfaceIntersection, Inflator::BoxIntersectionType, Inflator::BoxIntersectionType>
Inflator::findIntersectionTypeOfBB(
		const inflation::Box &actor_bb,
		const inflation::Box &object_bb)
{

	// note: remake of the `isWithinRangeOfBB` method
	// note: all boxes are axis aligned

	bool case_known = false;
	ignition::math::Vector3d pt_actor, pt_obstacle, pt_temp_actor, pt_temp_obstacle;
	BoxIntersectionType x_intersection_type = findIntersectionType(actor_bb.getMin().X(), actor_bb.getMax().X(), object_bb.getMin().X(), object_bb.getMax().X());
	BoxIntersectionType y_intersection_type = findIntersectionType(actor_bb.getMin().Y(), actor_bb.getMax().Y(), object_bb.getMin().Y(), object_bb.getMax().Y());

	// NOTE: `horizontal line` - along the Y axis
	// reasoning block based on the box intersection type
	if ( (x_intersection_type == BOX_INTERSECTION_TYPE_NONE_UPPER || x_intersection_type == BOX_INTERSECTION_TYPE_NONE_LOWER) &&
		 (y_intersection_type == BOX_INTERSECTION_TYPE_NONE_UPPER || y_intersection_type == BOX_INTERSECTION_TYPE_NONE_LOWER) )
	{

		// no `intersection` at all
		return (std::make_tuple(BOXES_SURFACE_INTERSECTION_NONE, x_intersection_type, y_intersection_type));

	} else if ( (x_intersection_type == BOX_INTERSECTION_TYPE_EXTERNAL && (y_intersection_type == BOX_INTERSECTION_TYPE_NONE_LOWER || y_intersection_type == BOX_INTERSECTION_TYPE_NONE_UPPER)) 		// 1 case (horizontal)
			 || (y_intersection_type == BOX_INTERSECTION_TYPE_EXTERNAL && (x_intersection_type == BOX_INTERSECTION_TYPE_NONE_LOWER || x_intersection_type == BOX_INTERSECTION_TYPE_NONE_UPPER)) 		// 1 case (vertical)
			 || (x_intersection_type == BOX_INTERSECTION_TYPE_INTERNAL && (y_intersection_type == BOX_INTERSECTION_TYPE_NONE_LOWER || y_intersection_type == BOX_INTERSECTION_TYPE_NONE_UPPER)) 		// 2 case (horizontal)
			 || (y_intersection_type == BOX_INTERSECTION_TYPE_INTERNAL && (x_intersection_type == BOX_INTERSECTION_TYPE_NONE_LOWER || x_intersection_type == BOX_INTERSECTION_TYPE_NONE_UPPER)) 		// 2 case (vertical)
			 || (x_intersection_type == BOX_INTERSECTION_TYPE_PARTIAL_LOWER && (y_intersection_type == BOX_INTERSECTION_TYPE_NONE_LOWER || y_intersection_type == BOX_INTERSECTION_TYPE_NONE_UPPER)) 	// 3 case (horizontal)
			 || (y_intersection_type == BOX_INTERSECTION_TYPE_PARTIAL_LOWER && (x_intersection_type == BOX_INTERSECTION_TYPE_NONE_LOWER || x_intersection_type == BOX_INTERSECTION_TYPE_NONE_UPPER)) 	// 3 case (vertical)
			 || (x_intersection_type == BOX_INTERSECTION_TYPE_PARTIAL_UPPER && (y_intersection_type == BOX_INTERSECTION_TYPE_NONE_LOWER || y_intersection_type == BOX_INTERSECTION_TYPE_NONE_UPPER)) 	// 4 case (horizontal) - equal to the 3rd case variation, for readability stands as a separate condition
			 || (y_intersection_type == BOX_INTERSECTION_TYPE_PARTIAL_UPPER && (x_intersection_type == BOX_INTERSECTION_TYPE_NONE_LOWER || x_intersection_type == BOX_INTERSECTION_TYPE_NONE_UPPER))	// 4 case (vertical)   -  				- '' -
			  )
	{

		// 'special' configuration occurs
		return (std::make_tuple(BOXES_SURFACE_INTERSECTION_SPECIAL, x_intersection_type, y_intersection_type)); // FIXME: temporary

	} else if ( x_intersection_type != BOX_INTERSECTION_TYPE_UNKNOWN && y_intersection_type != BOX_INTERSECTION_TYPE_UNKNOWN ) {

		//
		return (std::make_tuple(BOXES_SURFACE_INTERSECTION_PARTIAL, x_intersection_type, y_intersection_type));

	} else {

		// this should not happen
		return (std::make_tuple(BOXES_SURFACE_INTERSECTION_NONE, BOX_INTERSECTION_TYPE_UNKNOWN, BOX_INTERSECTION_TYPE_UNKNOWN));

	}

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Vector3d, ignition::math::Vector3d>
Inflator::findVectorSpecialIntersectionBB(const Inflator::BoxIntersectionType &x_intersection_type,
								const Inflator::BoxIntersectionType &y_intersection_type,
								const inflation::Box &actor_bb,
								const inflation::Box &object_bb)
{

	// routine related to the `BOXES_SURFACE_INTERSECTION_SPECIAL` case
	ignition::math::Vector3d pt_actor, pt_obstacle;

	if ( x_intersection_type == BOX_INTERSECTION_TYPE_EXTERNAL ) {

		// take the actor center and corresponding obstacle point (horizontal line)
		std::tie(pt_actor, pt_obstacle) = findShortestVectorIntersectedBoxes('x', actor_bb.getCenter().X(), actor_bb, object_bb);

	} else if ( x_intersection_type == BOX_INTERSECTION_TYPE_INTERNAL ) {

		// take the obstacle center and corresponding actor point (horizontal line)
		std::tie(pt_actor, pt_obstacle) = findShortestVectorIntersectedBoxes('x', object_bb.getCenter().X(), actor_bb, object_bb);

	} else if ( x_intersection_type == BOX_INTERSECTION_TYPE_PARTIAL_LOWER ) {

		// mean of the `intersection range`
		double x_mean = (actor_bb.getMin().X() + object_bb.getMax().X()) / 2.0;
		std::tie(pt_actor, pt_obstacle) = findShortestVectorIntersectedBoxes('x', x_mean, actor_bb, object_bb);

	} else if ( x_intersection_type == BOX_INTERSECTION_TYPE_PARTIAL_UPPER ) {

		// mean of the `intersection range`
		double x_mean = (actor_bb.getMax().X() + object_bb.getMin().X()) / 2.0;
		std::tie(pt_actor, pt_obstacle) = findShortestVectorIntersectedBoxes('x', x_mean, actor_bb, object_bb);

	} else if ( y_intersection_type == BOX_INTERSECTION_TYPE_EXTERNAL ) {

		// take the actor center and corresponding obstacle point (vertical line)
		std::tie(pt_actor, pt_obstacle) = findShortestVectorIntersectedBoxes('y', actor_bb.getCenter().Y(), actor_bb, object_bb);

	} else if ( y_intersection_type == BOX_INTERSECTION_TYPE_INTERNAL ) {

		// take the obstacle center and corresponding actor point (vertical line)
		std::tie(pt_actor, pt_obstacle) = findShortestVectorIntersectedBoxes('y', object_bb.getCenter().Y(), actor_bb, object_bb);

	} else if ( y_intersection_type == BOX_INTERSECTION_TYPE_PARTIAL_LOWER ) {

		// mean of the `intersection range`
		double y_mean = (object_bb.getMax().Y() + actor_bb.getMin().Y()) / 2.0;
		std::tie(pt_actor, pt_obstacle) = findShortestVectorIntersectedBoxes('y', y_mean, actor_bb, object_bb);

	} else if ( y_intersection_type == BOX_INTERSECTION_TYPE_PARTIAL_UPPER ) {

		// mean of the `intersection range`
		double y_mean = (actor_bb.getMax().Y() + object_bb.getMin().Y()) / 2.0;
		std::tie(pt_actor, pt_obstacle) = findShortestVectorIntersectedBoxes('y', y_mean, actor_bb, object_bb);

	}
	return (std::make_tuple(pt_actor, pt_obstacle));

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> Inflator::findClosestVerticesIntersectedBoxes(const inflation::Box &actor_box, const inflation::Box &object_box) {

	ignition::math::Vector3d pt_actor, pt_obstacle, pt_temp_actor, pt_temp_obstacle;

	// 1) find actor's box vertex that is the closest to the obstacle box center
	pt_temp_actor = findClosestBoundingBoxVertex(object_box.getCenter(), actor_box);
	// 2) find obstacle's box vertex that is the closest to the `pt_temp_actor`
	pt_temp_obstacle = findClosestBoundingBoxVertex(pt_temp_actor, object_box);

	return (std::make_tuple(pt_temp_actor, pt_temp_obstacle));

}

// ------------------------------------------------------------------- //

std::tuple<ignition::math::Vector3d, ignition::math::Vector3d> Inflator::findShortestVectorIntersectedBoxes(
		const char &axis, const double &coord_common, const inflation::Box &actor_box,
		const inflation::Box &object_box)
{

	ignition::math::Vector3d pt_actor, pt_obstacle, pt_temp_actor, pt_temp_obstacle;

	if ( axis == 'x' ) {
		pt_actor.X(coord_common);
		pt_obstacle.X(coord_common);
	} else if ( axis == 'y' ) {
		pt_actor.Y(coord_common);
		pt_obstacle.Y(coord_common);
	}

	// find the second coordinate of the points defined above
	std::tie(pt_temp_actor, pt_temp_obstacle) = findClosestVerticesIntersectedBoxes(actor_box, object_box);

	// update the second coordinat
	if ( axis == 'x' ) {
		pt_actor.Y(pt_temp_actor.Y());
		pt_obstacle.Y(pt_temp_obstacle.Y());
	} else if ( axis == 'y' ) {
		pt_actor.X(pt_temp_actor.X());
		pt_obstacle.X(pt_temp_obstacle.X());
	}

	return (std::make_tuple(pt_actor, pt_obstacle));

}
// ------------------------------------------------------------------- //

Inflator::BoxIntersectionType Inflator::findIntersectionType(const double &actor_coord_min, const double &actor_coord_max,
		const double &object_coord_min, const double &object_coord_max) {

	// TODO: add some reference (picture)
//	// configuration							diagram number
//	bool intersection_partial_lower = false; 	// 3
//	bool intersection_partial_upper = false; 	// 4
//	bool intersection_internal = false;		 	// 1
//	bool intersection_external = false;		 	// 5
//	bool no_intersection_lower = false;			// 2
//	bool no_intersection_upper = false;			// 6

	// check whether a vertex of the obstacle is within the `range` of actor box x coordinates
	if ( 		actor_coord_min <= object_coord_max && actor_coord_max >= object_coord_max &&
				actor_coord_min >= object_coord_min && actor_coord_max >= object_coord_min )
	{
		return (BoxIntersectionType::BOX_INTERSECTION_TYPE_PARTIAL_LOWER);

	} else if ( actor_coord_min <= object_coord_max && actor_coord_max <= object_coord_max &&
				actor_coord_min <= object_coord_min && actor_coord_max >= object_coord_min)
	{
		return (BoxIntersectionType::BOX_INTERSECTION_TYPE_PARTIAL_UPPER);

	} else if ( actor_coord_min <= object_coord_max && actor_coord_max >= object_coord_max &&
				actor_coord_min <= object_coord_min && actor_coord_max >= object_coord_min)
	{
		return (BoxIntersectionType::BOX_INTERSECTION_TYPE_INTERNAL);

	} else if ( actor_coord_min <= object_coord_max && actor_coord_max <= object_coord_max &&
				actor_coord_min >= object_coord_min && actor_coord_max >= object_coord_min)
	{
		return (BoxIntersectionType::BOX_INTERSECTION_TYPE_EXTERNAL);

	} else if ( actor_coord_min >= object_coord_max && actor_coord_max >= object_coord_max &&
				actor_coord_min >= object_coord_min && actor_coord_max >= object_coord_min)
	{
		return (BoxIntersectionType::BOX_INTERSECTION_TYPE_NONE_LOWER);

	} else if ( actor_coord_min <= object_coord_max && actor_coord_max <= object_coord_max &&
				actor_coord_min <= object_coord_min && actor_coord_max <= object_coord_min)
	{
		return (BoxIntersectionType::BOX_INTERSECTION_TYPE_NONE_UPPER);
	}

	return (BoxIntersectionType::BOX_INTERSECTION_TYPE_UNKNOWN);

}

// ------------------------------------------------------------------- //

Inflator::~Inflator() { }

// ------------------------------------------------------------------- //

} /* namespace sfm */
} /* namespace humap_local_planner */
