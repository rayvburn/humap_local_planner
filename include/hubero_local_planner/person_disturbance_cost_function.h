#pragma once

#include <base_local_planner/trajectory_cost_function.h>
#include <people_msgs_utils/person.h>

#include <vector>

namespace hubero_local_planner {

/**
 * @brief Trajectory cost function to penalize disturbances introduced by robot's motion to nearby people
 *
 * Penalizes robot motion that e.g. drives it towards person
 */
class PersonDisturbanceCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
	PersonDisturbanceCostFunction();

	/**
	 * @brief Updates dataset containing people detections
	 */
	void setPeopleDetections(const std::vector<people_msgs_utils::Person>& people);

	/**
	 * @brief Set the Parameters object
	 *
	 * @param fov_person person's field of view (full) that is used in disturbance calculations
	 * @param person_model_radius radius that defines circle that models physical area occupied by a person
	 * @param spatial_factor_exp defines smoothing rate of how disturbance weakens with a distance increase
	 */
	void setParameters(double fov_person, double person_model_radius, double spatial_factor_exp);

	/**
	 * @brief General updating of context values if required.
	 * Subclasses may overwrite. Return false in case there is any error.
	 */
	virtual bool prepare() override;

	/**
	 * @brief Returns a score for trajectory @ref traj
	 */
	virtual double scoreTrajectory(base_local_planner::Trajectory& traj) override;

	/**
	 * @brief Calculates value of person disturbance induced by robot motion (its direction in particular) in vicinity of people
	 *
	 * Disturbance is modeled by a Gaussian function. Its values are computed by arguments given in domain of angles.
	 * For further details check `dirCross` concept (location of the intersection point of i and j direction rays
	 * in relation to the i centre) in `hubero_local_planner`. Here, `i` is the person and `j` is the robot.
	 *
	 * @param x_robot
	 * @param y_robot
	 * @param yaw_robot
	 * @param vx_robot
	 * @param vy_robot
	 * @param x_person
	 * @param y_person
	 * @param yaw_person
	 * @param fov_person total angular field of view of the person
	 * @param person_model_radius assuming that person is modelled as a circle, choose a radius of that circle (in meters);
	 * determines, how wide the region of, cross_center direction angles will be with the circular model of the person
	 * @return Normalized (0-1) disturbance score
	 */
	static double calculateDirectionDisturbance(
		double x_robot,
		double y_robot,
		double yaw_robot,
		double vx_robot,
		double vy_robot,
		double x_person,
		double y_person,
		double yaw_person,
		double fov_person = 3.31613, // 190 degrees
		double person_model_radius = 0.4,
		double spatial_factor_exp = -0.8 // exponent provides approx 0.5 @ 1 m between centers of robot and person
	);

	/**
	 * @brief Computes value of 1D Gaussian probability density function
	 * @url https://en.wikipedia.org/wiki/Gaussian_function
	 *
	 * @param x
	 * @param mean mu
	 * @param variance
	 * @param normalize
	 * @return double
	 */
	static double calculateGaussian(double x, double mean, double variance, bool normalize = false);

	/**
	 * @brief Computes value of 1D Gaussian including wrapped regions of a `bell` curve (shifted -2pi and +2pi)
	 *
	 * Note that with @ref normalize set to true, it does no longer compute probability density function
	 * as its integral won't sum up to 1
	 *
	 * @param x
	 * @param mean mu
	 * @param variance
	 * @param normalize
	 * @return double
	 */
	static double calculateGaussianAngle(double x, double mean, double variance, bool normalize = false);

protected:
	std::vector<people_msgs_utils::Person> people_;
	double fov_person_;
	double person_model_radius_;
	double disturbance_spatial_factor_exp_;
};

} // namespace hubero_local_planner
