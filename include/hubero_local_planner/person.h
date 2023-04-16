#pragma once

#include <people_msgs_utils/person.h>
#include <hubero_local_planner/trajectory.h>

namespace hubero_local_planner {

/**
 * Extension of the @ref people_msgs_utils::Person providing trajectory prediction of the person
 */
class Person: public people_msgs_utils::Person {
public:
	/**
	 * Constructor extending basic @ref person with trajectory prediction each @ref dt for @ref steps_num steps forward
	 */
	Person(const people_msgs_utils::Person& person, double dt, unsigned int steps_num):
		people_msgs_utils::Person(person),
		traj_prediction_(Trajectory(person, dt, steps_num))
	{}

	Trajectory getTrajectoryPrediction() const {
		return traj_prediction_;
	}

protected:
	Trajectory traj_prediction_;
};

// Basic container
typedef std::vector<Person> People;

} // namespace hubero_local_planner
