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
 * In each case, a trajectory consists of N poses and N-1 velocities. First velocity enables to move the object
 * from pose0 to pose1 and so on.
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
	/// Default ctor creating dummy trajectory
	Trajectory(): dt_(0.0) {}

	/**
	 * Wraps @ref base_local_planner::Trajectory to simplify retrieving velocities along the trajectory
	 *
	 * Since @ref base_local_planner::Trajectory is commonly used to express robot trajectory with velocities
	 * represented in the body frame. When @ref convert_to_global_velocities is true, velocities will be expressed
	 * in the global frame.
	 */
	explicit Trajectory(
		const base_local_planner::Trajectory& traj,
		bool convert_to_global_velocities = true
	):
		dt_(traj.time_delta_)
	{
		// check all robot trajectory points ...
		for (unsigned int i = 0; i < traj.getPointsSize(); i++) {
			// retrieve pose
			double x, y, th = 0.0;
			traj.getPoint(i, x, y, th);
			auto pose_curr = geometry::Pose(x, y, th);

			// first velocity can be computed from base_local_planner::Trajectory seed or from poses(0-1);
			// omitting vel. calc. here provides that first velocity will not be repeated in the vels_ vector
			if (i == 0) {
				poses_.push_back(pose_curr);
				continue;
			}

			// store robot velocity
			double vx = 0.0;
			double vy = 0.0;
			double vth = 0.0;

			// eval robot displacement
			double x_prev, y_prev, th_prev = 0.0;
			traj.getPoint(i - 1, x_prev, y_prev, th_prev);
			auto pose_prev = geometry::Pose(x_prev, y_prev, th_prev);

			if (convert_to_global_velocities) {
				// convert to global coordinate system
				auto vel = computeVelocityFromPoses(pose_prev, pose_curr, dt_);
				vx = vel.getX();
				vy = vel.getY();
				vth = vel.getZ();
			} else {
				// compute velocities in local coordinate system
				auto vel = computeBaseVelocityFromPoses(pose_prev, pose_curr, dt_);
				vx = vel.getX();
				vy = vel.getY();
				vth = vel.getZ();
			}
			poses_.push_back(pose_curr);
			vels_.push_back(geometry::Vector(vx, vy, vth));
		}
	}

	/**
	 * Uses constant velocity model to expect trajectory initiated at @ref pose with velocity @ref vel
	 *
	 * @param pose pose expressed in the **global** coordinate system
	 * @param vel velocity expressed in the **global** coordinate system
	 * @param dt time delta
	 * @param steps how many steps to predict forward
	 * @param convert_to_local_velocities whether object getters should return velocities in global c.s.
	 */
	explicit Trajectory(
		const geometry::Pose& pose,
		const geometry::Vector& vel,
		double dt,
		unsigned int steps,
		bool convert_to_local_velocities = false
	): dt_(dt) {
		// save initial
		poses_.push_back(pose);

		if (!convert_to_local_velocities) {
			vels_.push_back(vel);
		} else {
			vels_.push_back(computeVelLocal(vel, pose));
		}

		for (unsigned int i = 1; i < steps; i++) {
			// to predict, refer to the newest available pose and initial velocity
			poses_.push_back(computeNextPose(poses_.back(), vel, dt_));

			// omit saving vel. during the last iteration (see class' doxygen)
			if (i == steps - 1) {
				continue;
			}

			if (!convert_to_local_velocities) {
				vels_.push_back(vel);
			} else {
				vels_.push_back(computeVelLocal(vel, poses_.back()));
			}
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
	 * @param convert_to_local_velocities whether object getters should return velocities in global c.s.
	 *
	 * Implementation of the template method must be placed inside the header
	 */
	template <typename T>
	explicit Trajectory(
		const T& object,
		double dt,
		unsigned int steps,
		bool convert_to_local_velocities = false
	): dt_(dt) {
		// current pose
		poses_.push_back(geometry::Pose(object.getPose()));
		// current global velocity (see ctor doc for assumption)
		geometry::Vector vel_global(object.getVelocityX(), object.getVelocityY(), object.getVelocityTheta());

		if (convert_to_local_velocities) {
			vels_.push_back(computeVelLocal(vel_global, poses_.back()));
		} else {
			vels_.push_back(vel_global);
		}

		// create future pose set for each detected object (assuming constant velocity)
		for (unsigned int i = 1; i < steps; i++) {
			// to predict, refer to the newest available pose and global velocity
			poses_.push_back(computeNextPose(poses_.back(), vel_global, dt_));

			// omit saving vel. during the last iteration (see class' doxygen)
			if (i == steps - 1) {
				continue;
			}

			if (convert_to_local_velocities) {
				vels_.push_back(computeVelLocal(vel_global, poses_.back()));
			} else {
				vels_.push_back(vel_global);
			}
		}
	}

	/**
	 * Similar to the templated constructor but this supports not dynamic objects to have a proper trajectory assigned
	 */
	template <typename T>
	explicit Trajectory(
		const T& object,
		unsigned int steps
	): dt_(1e-03) {
		// current pose
		poses_.push_back(geometry::Pose(object.getPose()));
		// current velocity
		geometry::Vector vel(0.0, 0.0, 0.0);
		vels_.push_back(vel);

		// create future pose set for each detected object (assuming constant velocity)
		for (unsigned int i = 1; i < steps; i++) {
			// to predict, refer to the newest available pose and initial velocity
			poses_.push_back(computeNextPose(poses_.back(), vel, dt_));

			// skip saving vel. during the last iteration (see class' doxygen)
			if (i == steps - 1) {
				continue;
			}

			vels_.push_back(vel);
		}
	}

	/// Returns number of stored poses. Note that the number of velocities is smaller by 1.
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

	size_t getVelocitiesNum() const {
		return vels_.size();
	}

	geometry::Vector getVelocity(const size_t& idx) const {
		if (!requestedValidVelocityIndex(idx)) {
			throw std::runtime_error("Wrong index requested in hubero_local_planner::Trajectory::getVelocity");
		}
		return vels_.at(idx);
	}

protected:
	bool requestedValidIndex(const size_t& idx) const {
		return getSteps() >= (idx + 1);
	}

	bool requestedValidVelocityIndex(const size_t& idx) const {
		return getVelocitiesNum() >= (idx + 1);
	}

	// Helper
	geometry::Vector computeVelLocal(const geometry::Vector& vel_glob, const geometry::Pose& pose) const {
		geometry::Vector vel_local;
		computeVelocityLocal(vel_glob, pose, vel_local);
		return vel_local;
	};

	double dt_;
	std::vector<geometry::Pose> poses_;
	std::vector<geometry::Vector> vels_;
};

} // namespace hubero_local_planner
