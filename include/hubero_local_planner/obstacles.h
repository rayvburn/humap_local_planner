#pragma once

// TEB obstacle representation
#include <teb_local_planner/obstacles.h>

namespace hubero_local_planner {

//! Abbrev. for shared obstacle pointers
typedef teb_local_planner::ObstaclePtr ObstaclePtr;
//! Abbrev. for shared obstacle const pointers
typedef teb_local_planner::ObstacleConstPtr ObstacleConstPtr;
//! Abbrev. for containers storing multiple obstacles
typedef teb_local_planner::ObstContainer ObstContainer;

typedef std::shared_ptr<ObstContainer> ObstContainerPtr;
typedef std::shared_ptr<const ObstContainer> ObstContainerConstPtr;

}; // hubero_local_planner
