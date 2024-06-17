/*
 * Cell.h
 *
 *  Created on: Dec 27, 2019
 *      Author: rayvburn
 */

#pragma once

#include <humap_local_planner/vis/marker_base.h>
#include <vector>

namespace humap_local_planner {
namespace vis {

using namespace geometry;
class HeatCell : public MarkerBase {

public:

	/// \brief Default constructor
	HeatCell();

	/// \brief Cell parameters setter method
	virtual void setParameters(const double& min_force_magnitude, const double& max_force_magnitude, const double& resolution);

	/// \brief Creates a Marker configured as a cube with a minimal height (in fact a square)
	virtual visualization_msgs::Marker create(const Vector& pos, const double& force_magnitude) const;

	/// \brief Destructor
	virtual ~HeatCell();

private:

	/// \brief Defines a maximum magnitude of the force vector
	double max_force_magnitude_;
	/// \brief Defines a minimum magnitude of the force vector
	double min_force_magnitude_;
	/// \brief Defines a heatmap cell side length
	double resolution_;

	/// \section Reference: https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
	typedef struct {
	    unsigned char r;
	    unsigned char g;
	    unsigned char b;
	} RgbColor;

	typedef struct {
	    unsigned char h;
	    unsigned char s;
	    unsigned char v;
	} HsvColor;

	/// \brief Converts a color from HSV-domain color to RGB-domain
	RgbColor HsvToRgb(const HsvColor& hsv) const;

	/// \brief Converts a color from RGB-domain color to HSV-domain
	HsvColor RgbToHsv(const RgbColor& rgb) const;

	/// \brief Converts force magnitude to the HSV-color value
	HsvColor convertMagnitudeToHSV(const double& magnitude) const;

};

} /* namespace vis */
} /* namespace humap_local_planner */
