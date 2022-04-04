/*
 * SocialConductor.h
 *
 *  Created on: Aug 6, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_FUZZ_SOCIALCONDUCTOR_H_
#define INCLUDE_FUZZ_SOCIALCONDUCTOR_H_

#include <hubero_local_planner/hubero_config.h>
#include <hubero_local_planner/fuzz/processor.h>
#include <hubero_local_planner/geometry/vector.h>

#include <vector>
#include <string>
#include <tuple>

namespace hubero_local_planner {
namespace fuzz {

using namespace geometry;

/**
 * Aggregates @ref Processor outputs (force directions) to create a resultant force vector
 * that represents behaviour force. Uses additivity rule of vectors.
 */
class SocialConductor {
public:
	/// \brief Default constructor
	SocialConductor() = default;

	/// \brief Updates internal state according to a given structure's content
	void initialize(std::shared_ptr<const hubero_local_planner::FisParams> cfg);

	/**
	 * Calculates behaviour force using additivity rule; normalizes vector if norm of sum exceeds unit vector length
	 * @param pose_agent: pose of the agent that computations are performed for; needed to convert behavior vector
	 * to global coordinates from local c.s. (direction in fis_outputs_v)
	 * @param fis_outputs_v: vector of partial FIS outputs (i.e. inference for each dynamic object)
	 * @param dist_v: vector of distances between ego agent and each dynamic object;
	 * must be the same size as fis_outputs_v vector - assuming certain `dist` corresponds to `fis_output`
	 * with the same vector index
	 */
	bool computeBehaviourForce(
		const Pose& pose_agent,
		const std::vector<Processor::FisOutput>& fis_outputs_v,
		const std::vector<double>& dist_v
	);

	/// \brief Sets social vector and active behaviour to default values
	void reset();

	/// \brief Returns the superposed social force vector.
	/// \return Superposed social force vector
	Vector getSocialVector() const;

	/// \brief Returns the last active behaviour expressed in verbal way
	/// \return
	std::string getBehaviourActive() const;

	/// \brief Destructor
	virtual ~SocialConductor() = default;

private:
	/// \brief Social force vector (the actual `social`; after `superposition` procedure)
	Vector behaviour_force_;

	/// \brief Stores active behaviour list as a string (each activated
	/// behaviour is placed in a new line).
	std::string behaviour_active_str_;

	/// \brief Stores i.a. maximum magnitudes of the force generated by the behaviour
	/// as well as resultant force multiplier
	std::shared_ptr<const hubero_local_planner::FisParams> cfg_;

	/// \brief Updates `behaviour_active_str_` according to its length
	/// and given behaviour name (method argument)
	void updateActiveBehaviour(const std::string& beh_name);

	/// \brief Defines strength of the behaviour, based on distance to agent
	double computeBehaviourStrength(const double& dist_to_agent);
};

} /* namespace fuzz */
} /* namespace hubero_local_planner */

#endif /* INCLUDE_FUZZ_SOCIALCONDUCTOR_H_ */
