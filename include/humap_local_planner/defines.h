#pragma once

namespace humap_local_planner {

typedef enum {
	LOCATION_FRONT = 0,
	LOCATION_RIGHT,
	LOCATION_LEFT,
	LOCATION_BEHIND,
	LOCATION_UNSPECIFIED
} RelativeLocation;

} // namespace humap_local_planner