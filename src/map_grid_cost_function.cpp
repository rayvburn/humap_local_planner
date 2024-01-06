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

#include <humap_local_planner/map_grid_cost_function.h>

namespace humap_local_planner {

MapGridCostFunction::MapGridCostFunction(
	costmap_2d::Costmap2D* costmap,
	double xshift,
	double yshift,
	bool is_local_goal_function,
	CostAggregationType aggregationType
):
	costmap_(costmap),
	map_(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()),
	aggregationType_(aggregationType),
	xshift_(xshift),
	yshift_(yshift),
	is_local_goal_function_(is_local_goal_function),
	stop_on_failure_(true),
	n_kernel_size_(3),
	n_cost_multiplier_(3.0)
{}

void MapGridCostFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses) {
	target_poses_ = target_poses;
}

bool MapGridCostFunction::prepare() {
	map_.resetPathDist();

	if (is_local_goal_function_) {
		map_.setLocalGoal(*costmap_, target_poses_);
	} else {
		map_.setTargetCells(*costmap_, target_poses_);
	}
	return true;
}

double MapGridCostFunction::getCellCosts(unsigned int px, unsigned int py) {
	double grid_dist = map_(px, py).target_dist;

	// check if some processing related to cell's neighbours is reasonable
	if (grid_dist != map_.unreachableCellCosts() || n_kernel_size_ <= 0) {
		return grid_dist;
	}

	// neighboring points
	std::vector<double> npts = {grid_dist};
	int offset = 1;
	while (offset <= n_kernel_size_) {
		// positive offset
		bool px_poff_wbounds = (px + offset) <= map_.size_x_;
		bool py_poff_wbounds = (py + offset) <= map_.size_y_;
		// negative offset
		bool px_noff_wbounds = (px - offset) >= 0;
		bool py_noff_wbounds = (py - offset) >= 0;

		if (px_poff_wbounds && py_poff_wbounds) {
			npts.push_back(map_(px + offset, py + offset).target_dist);
		}
		if (px_poff_wbounds) {
			npts.push_back(map_(px + offset, py + 0).target_dist);
		}
		if (py_poff_wbounds) {
			npts.push_back(map_(px + 0, py + offset).target_dist);
		}
		if (py_noff_wbounds) {
			npts.push_back(map_(px - 0, py - offset).target_dist);
		}
		if (px_noff_wbounds) {
			npts.push_back(map_(px - offset, py - 0).target_dist);
		}
		if (px_noff_wbounds && py_noff_wbounds) {
			npts.push_back(map_(px - offset, py - offset).target_dist);
		}
		if (px_poff_wbounds && py_noff_wbounds) {
			npts.push_back(map_(px + offset, py - offset).target_dist);
		}
		if (px_noff_wbounds && py_poff_wbounds) {
			npts.push_back(map_(px - offset, py + offset).target_dist);
		}
		offset++;
	}

	double max_cost = *std::max_element(npts.cbegin(), npts.cend());

	if (max_cost != map_.obstacleCosts() && max_cost != map_.unreachableCellCosts()) {
		// return min_cost
		return *std::min_element(npts.cbegin(), npts.cend()) * n_cost_multiplier_;
	}
	// return unchanged cost value
	return grid_dist;
}

double MapGridCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	double cost = 0.0;
	if (aggregationType_ == Product) {
		cost = 1.0;
	}
	double px, py, pth;
	unsigned int cell_x, cell_y;
	double grid_dist;

	for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
		traj.getPoint(i, px, py, pth);

		// translate point forward if specified
		if (xshift_ != 0.0) {
			px = px + xshift_ * cos(pth);
			py = py + xshift_ * sin(pth);
		}
		// translate point sideways if specified
		if (yshift_ != 0.0) {
			px = px + yshift_ * cos(pth + M_PI_2);
			py = py + yshift_ * sin(pth + M_PI_2);
		}

		//we won't allow trajectories that go off the map... shouldn't happen that often anyways
		if (!costmap_->worldToMap(px, py, cell_x, cell_y)) {
			//we're off the map
			ROS_WARN("Off Map %f, %f", px, py);
			return -4.0;
		}
		grid_dist = getCellCosts(cell_x, cell_y);
		//if a point on this trajectory has no clear path to the goal... it may be invalid
		if (stop_on_failure_) {
			if (grid_dist == map_.obstacleCosts()) {
				return -3.0;
			} else if (grid_dist == map_.unreachableCellCosts()) {
				return -2.0;
			}
		}

		switch(aggregationType_) {
		case Last:
			cost = grid_dist;
			break;
		case Sum:
			cost += grid_dist;
			break;
		case Product:
			if (cost > 0) {
				cost *= grid_dist;
			}
			break;
		}
	}
	return cost;
}

} // namespace humap_local_planner
