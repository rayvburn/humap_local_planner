/*
 * Arrow.h
 *
 *  Created on: Sep 27, 2019
 *      Author: rayvburn
 */

#pragma once

#include <hubero_common/typedefs.h>
#include <hubero_local_planner/vis/marker_base.h>

namespace vis {

class Arrow : public MarkerBase {

public:

	/// \brief Default constructor
	Arrow();

	/// \brief Sets arrow parameters
	/// \param[in] Max length of an arrow expressed in meters
	/// \param[in] SFM max force is a max allowable force
	/// which SFM algorithm could return; used to scale
	/// arrow's length
	virtual void setParameters(const float &length_meters, const float &sfm_max_force);

	/// \brief Method that creates a simple arrow
	/// of a previously set color
	/// \param[in] Foothold of an arrow
	/// \param[in] Force vector which is used to
	/// determine a marker's orientation and length
	virtual visualization_msgs::Marker create(const Vector3 &pos, const Vector3 &vector) const;

	/// \brief Default destructor
	virtual ~Arrow();

protected:

	/// \brief Max length of an arrow (in meters]
	float max_length_;

	/// \brief Max allowable force which SFM
	/// algorithm could return
	double sfm_max_force_;

};

} /* namespace vis */
