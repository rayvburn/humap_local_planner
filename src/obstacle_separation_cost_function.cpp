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
 * Extended/modified by Jarosław Karwowski, 2023
 *********************************************************************/
#include <hubero_local_planner/obstacle_separation_cost_function.h>

#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

namespace hubero_local_planner {

ObstacleSeparationCostFunction::ObstacleSeparationCostFunction(
	costmap_2d::Costmap2D* costmap
):
	costmap_(costmap),
	sum_scores_(false)
{
	if (costmap != NULL) {
		world_model_ = new base_local_planner::CostmapModel(*costmap_);
	}
}

ObstacleSeparationCostFunction::~ObstacleSeparationCostFunction() {
	if (world_model_ != NULL) {
		delete world_model_;
	}
}

void ObstacleSeparationCostFunction::setParams(
	double max_trans_vel,
	double max_scaling_factor,
	double scaling_speed,
	double min_separation_dist,
	unsigned short int separation_kernel
) {
	max_trans_vel_ = max_trans_vel;
	max_scaling_factor_ = max_scaling_factor;
	scaling_speed_ = scaling_speed;
	min_separation_dist_ = min_separation_dist;
	separation_kernel_ = static_cast<SeparationKernel>(separation_kernel);
}

void ObstacleSeparationCostFunction::setFootprint(std::vector<geometry_msgs::Point> footprint_spec) {
  	footprint_spec_ = footprint_spec;
}

bool ObstacleSeparationCostFunction::prepare() {
  	return true;
}

double ObstacleSeparationCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
	double cost = 0;
	double scale = getScalingFactor(traj, scaling_speed_, max_trans_vel_, max_scaling_factor_);
	double px, py, pth;
	if (footprint_spec_.size() == 0) {
		// Bug, should never happen
		ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
		return -9;
	}

	for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
		traj.getPoint(i, px, py, pth);
		double f_cost = footprintCost(
			px, py, pth,
			scale, min_separation_dist_, separation_kernel_, footprint_spec_,
			costmap_, world_model_
		);

		if (f_cost < 0) {
			return f_cost;
		}

		if (sum_scores_) {
			cost +=  f_cost;
		} else {
			cost = std::max(cost, f_cost);
		}
	}
	return cost;
}

double ObstacleSeparationCostFunction::getScalingFactor(
	base_local_planner::Trajectory &traj,
	double scaling_speed,
	double max_trans_vel,
	double max_scaling_factor
) {
	double vmag = hypot(traj.xv_, traj.yv_);

	//if we're over a certain speed threshold, we'll scale the robot's
	//footprint to make it either slow down or stay further from walls
	double scale = 1.0;
	if (vmag > scaling_speed) {
		//scale up to the max scaling factor linearly... this could be changed later
		double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
		scale = max_scaling_factor * ratio + 1.0;
	}
	return scale;
}

double ObstacleSeparationCostFunction::getFootprintCost(unsigned int px, unsigned int py) const {
	// convert inputs (map cells) to world coordinates
	double x, y = 0.0;
	costmap_->mapToWorld(px, py, x, y);

	double f_cost = footprintCost(
		x, y, 0.0,
		1.0, min_separation_dist_, separation_kernel_, footprint_spec_,
		costmap_, world_model_
	);
	return f_cost;
}

double ObstacleSeparationCostFunction::getFootprintCost(double x, double y, double th) const {
	return footprintCost(
		x, y, th,
		1.0, min_separation_dist_, separation_kernel_, footprint_spec_,
		costmap_, world_model_
	);
}

double ObstacleSeparationCostFunction::getFootprintCost(double x, double y, double th, double separation_dist) const {
	return footprintCost(
		x, y, th,
		1.0, separation_dist, separation_kernel_, footprint_spec_,
		costmap_, world_model_
	);
}

double ObstacleSeparationCostFunction::footprintCost(
	const double& x,
	const double& y,
	const double& th,
	double scale,
	double separation_dist,
	SeparationKernel separation_kernel,
	std::vector<geometry_msgs::Point> footprint_spec,
	costmap_2d::Costmap2D* costmap,
	base_local_planner::WorldModel* world_model
) {
	//check if the footprint is legal
	// TODO: Cache inscribed radius
	// TODO: Resize footprint according to @ref scale
	double footprint_cost = world_model->footprintCost(x, y, th, footprint_spec);

	bool compute_without_separation_dist = std::abs(separation_dist) < 1e-03;
	if (!compute_without_separation_dist) {
		// helper lambda to check footprint cost around the robot center
		auto footprintCostKernel = [=](double offset, double angle) -> double {
			// based on MapGridCostFunction
			double xk = x + offset * cos(th + angle);
			double yk = y + offset * sin(th + angle);
			return world_model->footprintCost(xk, yk, th, footprint_spec);
		};

		std::vector<double> costs;
		// cost of the footprint at the center
		costs.push_back(footprint_cost);

		switch (separation_kernel) {
			case (SeparationKernel::CROSS):
				// check 4 surrounding points
				costs.push_back(footprintCostKernel(separation_dist, 0.0));
				costs.push_back(footprintCostKernel(separation_dist, +M_PI_2));
				costs.push_back(footprintCostKernel(separation_dist, +M_PI));
				costs.push_back(footprintCostKernel(separation_dist, -M_PI_2));
				break;
			case (SeparationKernel::RECTANGLE):
				// check 8 surrounding points
				costs.push_back(footprintCostKernel(separation_dist, 0.0));
				costs.push_back(footprintCostKernel(separation_dist, +M_PI_4));
				costs.push_back(footprintCostKernel(separation_dist, +M_PI_2));
				costs.push_back(footprintCostKernel(separation_dist, +3.0 * M_PI_4));
				costs.push_back(footprintCostKernel(separation_dist, +M_PI));
				costs.push_back(footprintCostKernel(separation_dist, -3.0 * M_PI_4));
				costs.push_back(footprintCostKernel(separation_dist, -M_PI_2));
				costs.push_back(footprintCostKernel(separation_dist, -M_PI_4));
				break;
			default:
				// nothing
				break;
		}

		double max_cost = *std::max_element(costs.cbegin(), costs.cend());
		double min_cost = *std::min_element(costs.cbegin(), costs.cend());
		// choosing min negative or max positive
		(min_cost < 0.0) ? (footprint_cost = min_cost) : (footprint_cost = max_cost);
	}

	if (footprint_cost < 0) {
		return -6.0;
	}
	unsigned int cell_x, cell_y;

	//we won't allow trajectories that go off the map... shouldn't happen that often anyways
	if (!costmap->worldToMap(x, y, cell_x, cell_y)) {
		return -7.0;
	}

	double occ_cost = std::max(
		std::max(
			0.0,
			footprint_cost
		), double(costmap->getCost(cell_x, cell_y))
	);

	return occ_cost;
}

} // namespace hubero_local_planner
