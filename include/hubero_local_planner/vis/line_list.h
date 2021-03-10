/*
 * LineList.h
 *
 *  Created on: Sep 27, 2019
 *      Author: rayvburn
 */

#pragma once

#include <vector>
#include <hubero_common/typedefs.h>
#include <hubero_local_planner/vis/marker_base.h>
#include <visualization_msgs/MarkerArray.h>

namespace vis {

/// \brief LineList class extends Base's functionality
/// with a LineList management (may be developed further)
/// http://wiki.ros.org/rviz/DisplayTypes/Marker#Line_List_.28LINE_LIST.3D5.29
class LineList : public MarkerBase {

public:

	/// \brief Default constructor
	LineList();

	/// \brief Method which wraps createLineList()
	visualization_msgs::MarkerArray createArray(const std::vector<Pose3> &poses);
	visualization_msgs::Marker create(const std::vector<Pose3> &poses);

	/// \brief Method that passes position components
	/// of a poses to a createLineList() method which
	/// takes Vector3
	visualization_msgs::Marker create(const Pose3 &p1, const Pose3 &p2, const unsigned int &line_id) const;

	/// \brief Function that creates a line list;
	/// each line could be created out of 2 points
	visualization_msgs::Marker create(const Vector3 &p1, const Vector3 &p2, const unsigned int &line_id) const;

	/// \brief Default destructor
	virtual ~LineList();

protected:

	/// \brief Stores a max ID of a line
	/// which allows to delete old lines
	unsigned int line_id_max_;

};

} /* namespace vis */
