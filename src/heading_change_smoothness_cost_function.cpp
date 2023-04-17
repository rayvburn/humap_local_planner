#include <hubero_local_planner/heading_change_smoothness_cost_function.h>
#include <hubero_local_planner/trajectory.h>

#include <cmath>

namespace hubero_local_planner {

bool HeadingChangeSmoothnessCostFunction::prepare() {
	return true;
}

double HeadingChangeSmoothnessCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	if (traj.getPointsSize() == 0) {
		// no changes in heading as there is only 1 sample
		return 0.0;
	}

	// convert trajectory to global poses and velocities
	Trajectory t(traj);

	// heading change smoothness sum storage
	double hcs = 0.0;
	for (size_t i = 1; i < t.getPoses().size(); i++) {
		hcs += std::abs(t.getVelocity(i).getZ() - t.getVelocity(i - 1).getZ());
	}

	// mean
	return hcs / t.getPoses().size();
}

} // namespace hubero_local_planner
