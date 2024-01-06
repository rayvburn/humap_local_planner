/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Original Author: TKruse
 * Extended/modified by Jarosław Karwowski, 2024
 *********************************************************************/

#pragma once

#include <base_local_planner/trajectory_cost_function.h>

#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/map_grid.h>

namespace humap_local_planner {

/**
 * when scoring a trajectory according to the values in mapgrid, we can take
 *return the value of the last point (if no of the earlier points were in
 * return collision), the sum for all points, or the product of all (non-zero) points
 */
enum CostAggregationType { Last, Sum, Product};

/**
 * This class provides cost based on a map_grid of a small area of the world.
 * The map_grid covers a the costmap, the costmap containing the information
 * about sensed obstacles. The map_grid is used by setting
 * certain cells to distance 0, and then propagating distances around them,
 * filling up the area reachable around them.
 *
 * The approach using grid_maps is used for computational efficiency, allowing to
 * score hundreds of trajectories very quickly.
 *
 * @rayvburn: The original cost function has been modified in terms of searching of a valid cost cell among neighbors
 * of a cell that is marked as "unreachable" (even if it is perfectly reachable, but simply located in an area
 * of higher costs). This issue should be addressed directly in the base_local_planner::MapGrid, but modifying
 * the cell costs in MapGridCostFunction is an easier way to accomplish the same goal.
 *
 * This can be used to favor trajectories which stay on a given path, or which
 * approach a given goal.
 * @param costmap_ros Reference to object giving updates of obstacles around robot
 * @param xshift where the scoring point is with respect to robot center pose
 * @param yshift where the scoring point is with respect to robot center pose
 * @param is_local_goal_function, scores for local goal rather than whole path
 * @param aggregationType how to combine costs along trajectory
 */
class MapGridCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	MapGridCostFunction(
		costmap_2d::Costmap2D* costmap,
		double xshift = 0.0,
		double yshift = 0.0,
		bool is_local_goal_function = false,
		CostAggregationType aggregationType = Last
	);

	/**
	 * set line segments on the grid with distance 0, resets the grid
	 */
	void setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses);

	inline void setXShift(double xshift) {
		xshift_ = xshift;
	}

	inline void setYShift(double yshift) {
		yshift_ = yshift;
	}

	/** @brief If true, failures along the path cause the entire path to be rejected.
	 *
	 * Default is true. */
	inline void setStopOnFailure(bool stop_on_failure) {
		stop_on_failure_ = stop_on_failure;
	}

	// @rayvburn added this method
	inline void setKernelSize(unsigned int n_kernel_size) {
		n_kernel_size_ = n_kernel_size;
	}

	// @rayvburn added this method
	inline void setNeighborCellCostMultiplier(double n_cost_multiplier) {
		n_cost_multiplier_ = n_cost_multiplier;
	}

	/**
	 * propagate distances
	 */
	bool prepare();

	double scoreTrajectory(base_local_planner::Trajectory& traj);

	/**
	 * return a value that indicates cell is in obstacle
	 */
	double obstacleCosts() {
		return map_.obstacleCosts();
	}

	/**
	 * returns a value indicating cell was not reached by wavefront
	 * propagation of set cells. (is behind walls, regarding the region covered by grid)
	 */
	double unreachableCellCosts() {
		return map_.unreachableCellCosts();
	}

	// used for easier debugging
	double getCellCosts(unsigned int cx, unsigned int cy);

	// @rayvburn added this method
	std::vector<geometry_msgs::PoseStamped> getTargetPoses() const {
		return target_poses_;
	}

protected:
	std::vector<geometry_msgs::PoseStamped> target_poses_;
	costmap_2d::Costmap2D* costmap_;

	base_local_planner::MapGrid map_;
	CostAggregationType aggregationType_;
	// xshift and yshift allow scoring for different points of robots than center, like front or back this can help
	// with alignment or keeping specific wheels on tracks both default to 0
	double xshift_;
	double yshift_;
	/// If true, we look for a suitable local goal on path, else we use the full path for costs
	bool is_local_goal_function_;
	bool stop_on_failure_;

	// @rayvburn added this
	/// Defines a distance offset from a cell marked as unreachable to search a valid cost among its neighbors
	unsigned int n_kernel_size_;
	/// The multiplier of a cost when the resultant cost is obtained based on the neighbors
	double n_cost_multiplier_;
};

} // namespace humap_local_planner
