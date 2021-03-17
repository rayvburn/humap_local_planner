#pragma once

// obtacles representation (polygons)
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/distance_calculations.h>

// robot model representation
#include <teb_local_planner/robot_footprint_model.h>

// getRobotFootprintFromParamServer
#include <teb_local_planner/teb_local_planner_ros.h>

// typedef
namespace teb_local_planner {
typedef TebLocalPlannerROS LocalPlannerROS;
typedef std::shared_ptr<ObstContainer> ObstContainerPtr;
typedef std::shared_ptr<const ObstContainer> ObstContainerConstPtr;
}

// namespace alias
namespace teb = teb_local_planner;
