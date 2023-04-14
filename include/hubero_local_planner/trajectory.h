#pragma once

#include <hubero_local_planner/geometry/pose.h>
#include <hubero_local_planner/utils/transformations.h>

#include <base_local_planner/trajectory.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace hubero_local_planner {

/**
 * Wrapper class for trajectories
 *
 * @note Class definition has been put into header since for some reason classes from bigger libraries (e.g.,
 * `hubero_local_planner` in CMakeLists) can't be unit-tested anymore:
 * ```
 *   terminate called after throwing an instance of 'ros::TimeNotInitializedException'
 *   what():  Cannot use ros::Time::now() before the first NodeHandle has been created or ros::start() has been called.
 *   If this is a standalone app or test that just uses ros::Time and does not communicate over ROS, you may also call
 *   ros::Time::init()
 * ```
 * Creating separate library in CMakeLists may solve the issue, however, no time to investigate that deeper though.
 * Class with separate header/source has been successfully created in the main class of the planner.
 */
class Trajectory {
public:
	/// Default ctor
	Trajectory(): dt_(0.0) {}

	/**
	 * Wraps @ref base_local_planner::Trajectory to simplify retrieving velocities along the trajectory
	 *
	 * Since @ref base_local_planner::Trajectory is commonly used to express robot trajectory with velocities
	 * represented in the body frame, it converts vels to the global frame.
	 */
	explicit Trajectory(const base_local_planner::Trajectory& traj):
		dt_(traj.time_delta_)
	{
		// check all robot trajectory points ...
		for (unsigned int i = 0; i < traj.getPointsSize(); i++) {
			// retrieve poses
			double x, y, th = 0.0;
			traj.getPoint(i, x, y, th);
			// retrieve robot velocity
			double vx = 0.0;
			double vy = 0.0;
			double vth = 0.0;
			if (i == 0) {
				// velocity is explicitly given in base coordinates -> convert to global coords
				geometry::Vector vel;
				computeVelocityGlobal(
					geometry::Vector(traj.xv_, traj.yv_, traj.thetav_),
					geometry::Pose(x, y, th),
					vel
				);
				vx = vel.getX();
				vy = vel.getY();
				vth = vel.getZ();
			} else {
				// eval robot displacement
				double x_prev, y_prev, th_prev = 0.0;
				traj.getPoint(i - 1, x_prev, y_prev, th_prev);
				auto displacement = subtractPoses(
					geometry::Pose(x, y, th),
					geometry::Pose(x_prev, y_prev, th_prev)
				);
				// compute vel from pose difference
				auto vel = geometry::Vector(
					displacement.getX() / dt_,
					displacement.getY() / dt_,
					displacement.getYaw() / dt_
				);
				vx = vel.getX();
				vy = vel.getY();
				vth = vel.getZ();
			}
			poses_.push_back(geometry::Pose(x, y, th));
			vels_.push_back(geometry::Vector(vx, vy, vth));
		}
	}

	/**
	 * Uses constant velocity assumption to compute expected trajectory of @ref object
	 *
	 * @tparam T supports @ref people_msgs_utils::People and @ref people_msgs_utils::Group types.
	 * Assuming that object's getVelocity method returns velocity in the global coordinate system (not in body frame's).
	 *
	 * @param object
	 * @param dt time delta between steps
	 * @param steps number of total steps of the trajectory, i.e. steps = 5 will make predictions for 4 steps forward
	 *
	 * Implementation of the template method must be placed inside the header
	 */
	template <typename T>
	explicit Trajectory(
		const T& object,
		double dt,
		unsigned int steps
	): dt_(dt) {
		// current pose
		poses_.push_back(geometry::Pose(object.getPose()));
		// current velocity
		geometry::Vector vel(object.getVelocityX(), object.getVelocityY(), object.getVelocityTheta());
		vels_.push_back(vel);

		// create future pose set for each detected object (assuming constant velocity)
		for (unsigned int i = 1; i < steps; i++) {
			// to predict, refer to the newest available pose and initial velocity
			poses_.push_back(computeNextPose(poses_.back(), vel, dt_));
			vels_.push_back(vel);
		}
	}

	size_t getSteps() const {
		return poses_.size();
	}

	double getTime(const size_t& idx) const {
		if (!requestedValidIndex(idx)) {
			throw std::runtime_error("Wrong index requested in hubero_local_planner::Trajectory::getTime");
		}
		return dt_ * idx;
	}

	double getTimeDelta() const {
		return dt_;
	}

	std::vector<geometry::Pose> getPoses() const {
		return poses_;
	}

	geometry::Pose getPose(const size_t& idx) const {
		if (!requestedValidIndex(idx)) {
			throw std::runtime_error("Wrong index requested in hubero_local_planner::Trajectory::getPose");
		}
		return poses_.at(idx);
	}

	std::vector<geometry::Vector> getVelocities() const {
		return vels_;
	}

	geometry::Vector getVelocity(const size_t& idx) const {
		if (!requestedValidIndex(idx)) {
			throw std::runtime_error("Wrong index requested in hubero_local_planner::Trajectory::getVelocity");
		}
		return vels_.at(idx);
	}

	std::vector<std::pair<geometry::Pose, geometry::Vector>> getStates() const {
		if (poses_.size() != vels_.size()) {
			throw std::runtime_error("Wrong index requested in hubero_local_planner::Trajectory::getState");
		}

		std::vector<std::pair<geometry::Pose, geometry::Vector>> states;
		for (size_t i = 0; i < poses_.size(); i++) {
			states.push_back(std::make_pair(poses_.at(i), vels_.at(i)));
		}
		return states;
	}

	std::pair<geometry::Pose, geometry::Vector> getState(const size_t& idx) const {
		if (!requestedValidIndex(idx)) {
			throw std::runtime_error("Wrong index requested in hubero_local_planner::Trajectory::getState");
		}
		return std::make_pair(poses_.at(idx), vels_.at(idx));
	}

protected:
	bool requestedValidIndex(const size_t& idx) const {
		return getSteps() >= (idx + 1);
	}

	double dt_;
	std::vector<geometry::Pose> poses_;
	std::vector<geometry::Vector> vels_;
};

} // namespace hubero_local_planner
