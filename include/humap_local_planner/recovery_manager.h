#pragma once

#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/trajectory.h>

#include <humap_local_planner/obstacle_separation_cost_function.h>

// custom implementation of the CircularBuffer
#include <deque>
#include <stdexcept>

namespace humap_local_planner {

template <typename T>
class CircularBuffer {
public:
	explicit CircularBuffer(size_t capacity = 0):
		buffer_(), capacity_(capacity), size_(0) {}

	T& at(size_t index) {
		return buffer_.at(index);
	}

	void push_back(const T& value) {
		if (size_ == capacity_) {
			// Remove the oldest element if the buffer is full
			buffer_.pop_front();
			size_--;
		}
		buffer_.push_back(value);
		size_++;
	}

	T& front() {
		if (empty()) {
			throw std::out_of_range("Circular buffer is empty");
		}
		return buffer_.front();
	}

	void pop_front() {
		if (empty()) {
			throw std::out_of_range("Circular buffer is empty");
		}
		buffer_.pop_front();
		size_--;
	}

	bool empty() const {
		return size_ == 0;
	}

	size_t size() const {
		return size_;
	}

	void set_capacity(size_t new_capacity) {
		capacity_ = new_capacity;
		while (size_ > capacity_) {
			buffer_.pop_front();
			size_--;
		}
	}

	void clear() {
		buffer_.clear();
	}

	size_t capacity() const {
		return capacity_;
	}

	const std::deque<T>& get_deque() const {
		return buffer_;
	}

protected:
	std::deque<T> buffer_;
	size_t capacity_;
	size_t size_;
};

/**
 * @brief
 *
 * This class is partially based on TEB's FailureDetector by Christoph Rösmann
 */
class RecoveryManager {
public:
	/// How many points around the robot should be checked as the recovery goal pose candidates
	static constexpr int RECOVERY_ROTATE_AND_RECEDE_SURROUNDING_PTS = 16;
	/// Defines length of the path whose poses must be collision-free to choose it for recovery
	static constexpr double RECOVERY_ROTATE_AND_RECEDE_DISTANCE = 0.3;

	RecoveryManager();

	/**
	 * @brief Set buffer length (measurement history) for oscillation detection
	 * @param length number of measurements to be kept
	 */
	void setOscillationBufferLength(int length);

	/**
	 * @brief Adds a new twist measurement to the internal buffer and computes a new decision
	 *
	 * @param v_x linear X component of the current velocity command
	 * @param v_th angular Z component of the current velocity command
	 * @param v_max maximum forward translational velocity
	 * @param v_backwards_max maximum backward translational velocity
	 * @param omega_max maximum angular velocity
	 * @param v_eps Threshold for the average normalized linear velocity in (0,1) that must not be exceeded (e.g. 0.1)
	 * @param omega_eps Threshold for the average normalized angular velocity in (0,1) that must not be exceeded (e.g. 0.1)
	 */
	void checkOscillation(
		double v_x,
		double v_th,
		double v_max,
		double v_backwards_max,
		double omega_max,
		double v_eps,
		double omega_eps
	);

	/**
	 * @brief Checks whether the robot's trajectory deviates from the gradient-based cost functions
	 *
	 * Deviation may occur, e.g., when obstacles suddenly appear in front of the robot of nearby
	 *
	 * @param traj robot trajectory to be evaluated
	 * @param alignment_costfun non-const only because scoreTrajectory method is non-const
	 * @param goal_front_costfun non-const only because scoreTrajectory method is non-const
	 */
	void checkDeviation(
		base_local_planner::Trajectory traj,
		base_local_planner::MapGridCostFunction& goal_front_costfun,
		base_local_planner::MapGridCostFunction& alignment_costfun
	);

	/**
	 * @brief Checks whether the robot's footprint is in collision with the environment according to the costmap
	 *
	 * @param traj robot trajectory to be evaluated
	 * @param obstacle_costfun non-const only because scoreTrajectory method is non-const
	 */
	void checkCollision(
		base_local_planner::Trajectory traj,
		ObstacleSeparationCostFunction& obstacle_costfun
	);

	/**
	 * @brief Searches for a safe pose near the robot to recover from being stuck
	 *
	 * @param x x-component of the current robot pose
	 * @param y y-component of the current robot pose
	 * @param th theta-component of the current robot pose
	 * @param obstacle_costfun cost function that evaluates whether a pose in world coordinates is collision-free
	 * @return true If recovery goal pose is found (recovery is feasible)
	 * @return false If recovery goal pose was not found
	 */
	bool planRecoveryRotateAndRecede(
		double x,
		double y,
		double th,
		const ObstacleSeparationCostFunction& obstacle_costfun
	);

	/**
	 * @brief Clear the current internal state
	 *
	 * This call also resets the internal buffer
	 */
	void clear();

	/**
	 * @brief Check if the robot is oscillating (and possibly stuck totally soon)
	 */
	inline bool isOscillating() const {
		return oscillating_;
	}

	inline bool isDeviatingFromGlobalPath() const {
		return deviating_from_global_plan_;
	}

	inline bool isStuck() const {
		return stuck_;
	}

	inline bool isNearCollision() const {
		return prev_near_collision_ || near_collision_;
	}

	inline bool canRecover() const {
		return can_recover_from_collision_;
	}

	inline double getRecoveryGoalX() const {
		return recovery_goal_x_;
	}

	inline double getRecoveryGoalY() const {
		return recovery_goal_y_;
	}

	inline double getRecoveryGoalYaw() const {
		return recovery_goal_th_;
	}

	static bool isPoseCollisionFree(double costmap_cost);

protected:
	/** Variables to be monitored */
	struct VelMeasurement {
		double v = 0;
		double omega = 0;
	};

	/**
	 * @brief Detect if the robot got stucked based on the current buffer content
	 *
	 * @param v_eps Threshold for the average normalized linear velocity in (0,1) that must not be exceded (e.g. 0.1)
	 * @param omega_eps Threshold for the average normalized angular velocity in (0,1) that must not be exceded (e.g. 0.1)
	 */
	void detectOscillation(double v_eps, double omega_eps);

	/// Sets all coordinates of the recovery goal to NAN
	void resetRecoveryGoal();

	/// Circular buffer to store the last measurements @see setOscillationBufferLength
	CircularBuffer<VelMeasurement> buffer_;

	/// Current state: true if robot is oscillating
	bool oscillating_;

	/// True if the robot is stuck (based on the data in the buffer)
	bool stuck_;

	bool deviating_from_global_plan_;

	bool near_collision_;
	// Escaping the recovery might be problematic (switching between recovery and normal mode in subsequent iterations)
	bool prev_near_collision_;
	bool can_recover_from_collision_;

	/// Coordinates of the recovery goal
	double recovery_goal_x_;
	double recovery_goal_y_;
	double recovery_goal_th_;

	/// Stores previous linear velocity of the velocity command send to the mobile base
	double prev_cmd_vx;
	double prev_cmd_omega;
};

} // namespace humap_local_planner
