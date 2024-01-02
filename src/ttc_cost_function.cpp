#include <humap_local_planner/ttc_cost_function.h>
#include <humap_local_planner/utils/transformations.h>

#include <cmath>
#include <limits>

namespace humap_local_planner {

TTCCostFunction::TTCCostFunction(const World& world_model):
	world_model_(world_model),
	max_sim_time_(3.0),
	collision_distance_(0.05) {
}

void TTCCostFunction::setParameters(const double& max_sim_time, const double& collision_distance) {
	max_sim_time_ = max_sim_time;
	collision_distance_ = collision_distance;
}

void TTCCostFunction::reset() {
	robot_dyn_obj_v_.clear();
	robot_stat_obj_v_.clear();
}

bool TTCCostFunction::prepare() {
	return true;
}

double TTCCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	// check error conditions first
	if (traj.getPointsSize() == 0) {
		return -10.0;
	}

	const double dt = traj.time_delta_;
	// we can proceed only when `dt` is properly defined
	if (dt <= 1e-12 || std::isinf(dt) || std::isnan(dt)) {
		return -12.0;
	}

	/*
	 * How TTC is computed?
	 * We were given trajectory that consists of seed velocity (initial) and consecutive points (without velocities).
	 * We create World model from the location of the last robot trajectory point.
	 * Dynamic obstacles state is predicted N steps forward, where N is number of trajectory points.
	 */
	// initialize vector for visualization data
	std::vector<std::vector<Distance>> state_prediction_static;
	std::vector<std::vector<Distance>> state_prediction_dynamic;

	// recreate robot trajectory
	Trajectory robot_traj(traj);
	// predict world states sequence based on robot trajectory
	std::vector<World> world_sequence = world_model_.predict(robot_traj);

	double timestamp = 0.0;
	double traj_time = traj.getPointsSize() * dt;

	// store the smallest distance to static or dynamic obstacle in a whole trajectory
	double dist_min_static = std::numeric_limits<double>::max();
	double dist_min_dynamic = std::numeric_limits<double>::max();

	// simulate forward
	for (const auto& world: world_sequence) {
		// save state for a specific timestamp
		collectWorldStateData(
			world,
			state_prediction_static,
			state_prediction_dynamic
		);

		// check distances from robot and obstacles
		dist_min_static = std::min(dist_min_static, world.getDistanceClosestStaticObject());
		dist_min_dynamic = std::min(dist_min_dynamic, world.getDistanceClosestDynamicObject());

		if (checkForCollision(
			world,
			dist_min_static,
			dist_min_dynamic)
		) {
			// we'll score and stop here
			// save data for a specific trajectory
			robot_stat_obj_v_.push_back(state_prediction_static);
			robot_dyn_obj_v_.push_back(state_prediction_dynamic);
			return computeCost(timestamp, traj_time + max_sim_time_);
		}
		timestamp += dt;
	}

	/* Starting velocity computation for simulation forward beyond */
	// now, investigate further state (beyond trajectory bounds)
	// stick to trajectory time delta and maintain last velocity of the trajectory
	geometry::Vector robot_global_vel = robot_traj.getVelocities().back();

	/* Starting simulation further beyond */
	// predict further beyond trajectory simulation time, velocity doesn't change here
	for (double t = 0; t < max_sim_time_; t += dt) {
		auto world_model_pred = world_sequence.back();

		// save state for a specific timestamp
		collectWorldStateData(
			world_model_pred,
			state_prediction_static,
			state_prediction_dynamic
		);

		// check distances from robot and obstacles
		dist_min_static = std::min(dist_min_static, world_model_pred.getDistanceClosestStaticObject());
		dist_min_dynamic = std::min(dist_min_dynamic, world_model_pred.getDistanceClosestDynamicObject());

		if (checkForCollision(
			world_model_pred,
			dist_min_static,
			dist_min_dynamic)
		) {
			// we'll score and stop here
			// save data for a specific trajectory
			robot_stat_obj_v_.push_back(state_prediction_static);
			robot_dyn_obj_v_.push_back(state_prediction_dynamic);
			return computeCost(timestamp, traj_time + max_sim_time_);
		}

		world_model_pred.predict(robot_global_vel, dt);
		timestamp += dt;

		// save world state
		world_sequence.push_back(world_model_pred);
	}

	// save data for a specific trajectory
	robot_stat_obj_v_.push_back(state_prediction_static);
	robot_dyn_obj_v_.push_back(state_prediction_dynamic);

	/* Finished simulations - no TTC defined */
	// TTC is assumed to be infinity - cost is zero
	return 0.0;
}

void TTCCostFunction::collectWorldStateData(
	const World& world_model,
	std::vector<std::vector<Distance>>& prediction_data_static,
	std::vector<std::vector<Distance>>& prediction_data_dynamic
) {
	auto objects_static = world_model.getStaticObjectsData();
	std::vector<Distance> dists_static;
	for (const auto& obj_stat: objects_static) {
		Distance dist {};
		// NOTE: robot pose that is the closest to the obstacle is used here
		dist.robot = obj_stat.robot;
		dist.object = obj_stat.object;
		dists_static.push_back(dist);
	}
	prediction_data_static.push_back(dists_static);

	auto objects_dyn = world_model.getDynamicObjectsData();
	std::vector<Distance> dists_dynamic;
	for (const auto& obj_dyn: objects_dyn) {
		Distance dist {};
		// NOTE: robot pose that is the closest to the obstacle is used here
		dist.robot = obj_dyn.robot;
		dist.object = obj_dyn.object;
		dists_dynamic.push_back(dist);
	}
	prediction_data_dynamic.push_back(dists_dynamic);
}

bool TTCCostFunction::checkForCollision(
	const World& world_model,
	const double& dist_min_static,
	const double& dist_min_dynamic
) {
	if (dist_min_static <= collision_distance_) {
		return true;
	}
	if (dist_min_dynamic <= collision_distance_) {
		return true;
	}
	return false;
}

double TTCCostFunction::computeCost(double ttc, double total_prediction_time) const {
	// avoid inf costs since those trajectories are hard to compare
	if (ttc <= 0.0) {
		ttc = 1e-04;
	}
	// only TTCs < max_sim_time will be bigger than 0
	return 1.0 / (ttc / total_prediction_time);
}

} // namespace humap_local_planner
