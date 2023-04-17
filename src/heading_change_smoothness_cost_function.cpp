#include <hubero_local_planner/heading_change_smoothness_cost_function.h>
#include <hubero_local_planner/trajectory.h>

#include <cmath>

namespace hubero_local_planner {

bool HeadingChangeSmoothnessCostFunction::prepare() {
	return true;
}

double HeadingChangeSmoothnessCostFunction::scoreTrajectory(base_local_planner::Trajectory& traj) {
	if (traj.getPointsSize() <= 1) {
		// too few samples
		return 0.0;
	}

	// convert trajectory to global poses and velocities
	Trajectory t(traj);

	// heading change smoothness sum storage
	double hcs = 0.0;
	for (size_t i = 1; i < t.getPoses().size(); i++) {
		double domega = t.getVelocity(i).getZ() - t.getVelocity(i - 1).getZ();
        hcs += (std::abs(domega) / t.getTimeDelta());
	}

	// mean
	return hcs / static_cast<double>((t.getPoses().size() - 1));
}

} // namespace hubero_local_planner
