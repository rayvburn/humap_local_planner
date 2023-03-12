#include <hubero_local_planner/ttc_cost_function.h>
#include <hubero_local_planner/utils/transformations.h>

#include <cmath>
#include <limits>

namespace hubero_local_planner {

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

	// recreate world model sequence
	std::vector<World> world_sequence;
	// create a copy of initial world model for pose predictions
	World world_model_pred = world_model_;

	// save world initial state
	world_sequence.push_back(world_model_pred);
	double timestamp = 0.0;

	double traj_time = traj.getPointsSize() * dt;

	// store the smallest distance to static or dynamic obstacle in a whole trajectory
	double dist_min_static = std::numeric_limits<double>::max();
	double dist_min_dynamic = std::numeric_limits<double>::max();

	/* Starting trajectory recreation */
	// simulate forward
	for (unsigned int sim_step = 0; sim_step < traj.getPointsSize(); sim_step++) {
		// retrieve 2d pose from trajectory
		double traj_x, traj_y, traj_th = 0.0;
		traj.getPoint(sim_step, traj_x, traj_y, traj_th);
		auto traj_pose = geometry::Pose(
			traj_x,
			traj_y,
			world_model_pred.getRobotData().centroid.getZ(),
			world_model_pred.getRobotData().centroid.getRoll(),
			world_model_pred.getRobotData().centroid.getPitch(),
			traj_th
		);

		// eval robot displacement
		auto robot_displacement = subtractPoses(traj_pose, world_model_pred.getRobotData().centroid);

		// eval robot velocity
		geometry::Vector robot_vel;
		if (sim_step == 0) {
			// use trajectory seed converted to global coordinates
			computeVelocityGlobal(
				geometry::Vector(traj.xv_, traj.yv_, traj.thetav_),
				traj_pose,
				robot_vel
			);
		} else {
			// compute from pose difference
			robot_vel = geometry::Vector(
				robot_displacement.getX() / dt,
				robot_displacement.getY() / dt,
				robot_displacement.getYaw() / dt
			);
		}

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

		// recreate world prediction (from trajectory)
		world_model_pred.predict(robot_vel, dt);
		timestamp += dt;

		// save world state
		world_sequence.push_back(world_model_pred);
	}

	/* Starting velocity computation for simulation forward beyond */
	// now, investigate further state (beyond trajectory bounds)
	// stick to trajectory time delta and maintain last velocity of the trajectory
	geometry::Vector base_vel_last;
	if (traj.getPointsSize() >= 2) {
		// compute velocity based on pose difference
		// last point
		double x_l, y_l, th_l = 0.0;
		traj.getPoint(traj.getPointsSize() - 1, x_l, y_l, th_l);
		// second to last point
		double x_stl, y_stl, th_stl = 0.0;
		traj.getPoint(traj.getPointsSize() - 2, x_stl, y_stl, th_stl);
		// difference between poses
		geometry::Vector pose_diff(
			x_l - x_stl,
			y_l - y_stl,
			th_l - th_stl
		);
		// velocity at the end of the trajectory
		base_vel_last = geometry::Vector(
			pose_diff.getX() / dt,
			pose_diff.getY() / dt,
			pose_diff.getZ() / dt
		);
	} else {
		// velocity is stored in as trajectory seed (only 1 point)
		base_vel_last = geometry::Vector(traj.xv_, traj.yv_, traj.thetav_);
	}

	// convert to global velocity (used by world model representation)
	geometry::Vector robot_global_vel;
	computeVelocityGlobal(
		base_vel_last,
		world_model_pred.getRobotData().centroid,
		robot_global_vel
	);

	/* Starting simulation further beyond */
	// predict further beyond trajectory simulation time, velocity doesn't change here
	for (double t = 0; t <= max_sim_time_; t += dt) {
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

} // namespace hubero_local_planner
