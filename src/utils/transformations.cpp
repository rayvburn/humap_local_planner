#include <hubero_local_planner/utils/transformations.h>
#include <hubero_local_planner/utils/debug.h>

// debugging macros
#define DEBUG_BASIC 0
#define DEBUG_VERBOSE 0
#define debug_print_basic(fmt, ...) _template_debug_print_basic_(DEBUG_BASIC, fmt, ##__VA_ARGS__)
#define debug_print_verbose(fmt, ...) _template_debug_print_basic_(DEBUG_VERBOSE, fmt, ##__VA_ARGS__)
#define debug_print_warn(fmt, ...) _template_debug_print_warn_(1, fmt, ##__VA_ARGS__)

namespace hubero_local_planner {

using namespace geometry;

geometry::Pose computeNextPoseBaseVel(const geometry::Pose& pose, const geometry::Vector& vel, double dt) {
	// NOTE: calculations based on base_local_planner::SimpleTrajectoryGenerator::computeNewPositions
	geometry::Pose pose_new;
	double new_x = pose.getX() + (vel.getX() * cos(pose.getYaw()) + vel.getY() * cos(M_PI_2 + pose.getYaw())) * dt;
	double new_y = pose.getY() + (vel.getX() * sin(pose.getYaw()) + vel.getY() * sin(M_PI_2 + pose.getYaw())) * dt;
	double new_yaw = pose.getYaw() + vel.getZ() * dt;

	pose_new.setPosition(new_x, new_y, pose.getZ());
	pose_new.setOrientation(pose.getRoll(), pose.getPitch(), Angle(new_yaw).getRadian());
	return pose_new;
}

geometry::Pose computeNextPose(const geometry::Pose& pose, const geometry::Vector& vel, double dt) {
	geometry::Pose pose_new;
	double new_x = pose.getX() + vel.getX() * dt;
	double new_y = pose.getY() + vel.getY() * dt;
	double new_yaw = pose.getYaw() + vel.getZ() * dt;

	pose_new.setPosition(new_x, new_y, pose.getZ());
	pose_new.setOrientation(pose.getRoll(), pose.getPitch(), Angle(new_yaw).getRadian());
	return pose_new;
}

void computeTwistNonholonomic(
	const geometry::Pose& pose,
	const geometry::Vector& force,
	const geometry::Vector& robot_vel_glob,
	const double& sim_period,
	const double& robot_mass,
	const double& min_vel_x,
	const double& max_vel_x,
	const double& max_rot_vel,
	geometry::Vector& cmd_vel
) {
	std::cout << "hubero_local_planner::computeTwistNonholonomic not implemented yet" << std::endl;
}

void computeTwist(
	const Pose& pose,
	const Vector& force,
	const Vector& robot_vel_glob,
	const double& sim_period,
	const double& robot_mass,
	const double& min_vel_x,
	const double& max_vel_x,
	const double& max_rot_vel,
	const double& twist_rotation_compensation,
	Vector& cmd_vel
) {
	// no force - no movement (strain physical laws)
	if (force.calculateLength() <= 1e-08) {
		cmd_vel = Vector(0.0, 0.0, 0.0);
		return;
	}

	// convert 2D forces into robot forces with non-holonomic contraints
	double yaw = pose.getYaw();

	// alias
	double dt = sim_period;

	// conversion
	Angle force_dir(force);

	// global force affects global acceleration directly
	Vector acc_v = force / robot_mass;
	// compute how much force affects global velocity of the robot
	Vector vel_v_new = acc_v * dt;

	// inverted rotation matrix expressed as 2 eqn
	// @url https://github.com/yinzixuan126/modified_dwa/blob/b379c01e37adc1f6414005750633b05e1a024ae5/iri_navigation/iri_akp_local_planner_companion/local_lib/src/scene_elements/robot.cpp#L91
	// projection to robot pose (dot product)
	double vv = +cos(yaw) * vel_v_new.getX() + sin(yaw) * vel_v_new.getY();
	// cross product theta x f
	double vw = -sin(yaw) * vel_v_new.getX() + cos(yaw) * vel_v_new.getY();

	// EXPERIMENTAL
	Angle ang_z_force_diff(force_dir.getRadian() - yaw);
    // strengthen rotation
	vw += twist_rotation_compensation * ang_z_force_diff.getRadian();

	// logging section
	debug_print_verbose("force - x: %2.3f, y: %2.3f, th: %2.3f, mass - %2.3f\r\n",
		force.getX(), force.getY(), force.getZ(), robot_mass
	);

	debug_print_verbose("accel - x: %2.3f, y: %2.3f, th: %2.3f, vel - x: %2.3f, y: %2.3f, th: %2.3f\r\n",
		acc_v.getX(), acc_v.getY(), acc_v.getZ(),
		vel_v_new.getX(), vel_v_new.getY(), vel_v_new.getZ()
	);

	debug_print_verbose("cmd_vel 'init'  - lin.x: %2.3f, ang.z: %2.3f\r\n", vv, vw);

	// hard non-linearities section:
	// check if within limits: try to maintain path, ignoring trajectory
	if (vv > max_vel_x || vw > max_rot_vel) {
		double excess_x = vv / max_vel_x;
		double excess_z = vw / max_rot_vel;
		double shortening_factor = 1 / std::max(excess_x, excess_z);
		// shorten proportionally
		vv *= shortening_factor;
		vw *= shortening_factor;
		debug_print_warn(
			"Requested velocity not within limits! excess_x %2.3f, excess_z %2.3f, multiplier %2.3f "
			"cmd_vel 'modded' lin.x: %2.3f, ang.z: %2.3f\r\n",
			excess_x, excess_z, shortening_factor, vv, vw
		);
	}

	// trim minimum velocity to `min_vel_x`
	if (vv < min_vel_x) {
		vv = min_vel_x;
		debug_print_warn("Requested velocity is smaller than min! 'modded' lin.x %2.3f\r\n", vv);
	}

	// assign final values
	cmd_vel.setX(vv);
	cmd_vel.setY(0.0);
	cmd_vel.setZ(vw);

	debug_print_verbose(
		"cmd_vel 'final' - lin.x: %2.3f (min %2.3f, max %2.3f), ang.z: %2.3f (max %2.3f)\r\n",
		cmd_vel.getX(), min_vel_x, max_vel_x, cmd_vel.getZ(), max_rot_vel
	);
}

void computeVelocityGlobal(
	const Vector& vel_local,
	const Pose& pose,
	Vector& vel_global
) {
	// slide 38 at https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture3-RobotMotion.pdf
	double yaw = pose.getYaw();
	Eigen::Matrix3d rotation_yaw_inv;
	// create a rotation matrix
	rotation_yaw_inv <<
		cos(yaw), 0, 0,
		sin(yaw), 0, 0,
		0, 0, 1;
	// compute global velocity vector
	Eigen::Vector3d vel_global_eigen = rotation_yaw_inv * vel_local.getAsEigen<Eigen::Vector3d>();
	// prepare twist expressed in global coordinates
	vel_global = Vector(vel_global_eigen);

	debug_print_verbose("velocity local  : x: %2.4f, y: %2.4f, ang_z: %2.4f   (yaw: %2.4f,  sin(yaw): %2.4f,  cos(yaw): %2.4f) \r\n",
		vel_local.getX(),
		vel_local.getY(),
		vel_local.getZ(),
		yaw,
		sin(yaw),
		cos(yaw)
	);

	debug_print_verbose("velocity global: x: %2.4f, y: %2.4f, ang_z: %2.4f \r\n",
		vel_global.getX(),
		vel_global.getY(),
		vel_global.getZ()
	);
}

geometry::Pose subtractPoses(const geometry::Pose& pose_ref, const geometry::Pose& pose_other) {
	return Pose(
		pose_ref.getX() - pose_other.getX(),
		pose_ref.getY() - pose_other.getY(),
		pose_ref.getZ() - pose_other.getZ(),
		pose_ref.getRoll() - pose_other.getRoll(),
		pose_ref.getPitch() - pose_other.getPitch(),
		pose_ref.getYaw() - pose_other.getYaw()
	);
}

geometry::Pose addPoses(const geometry::Pose& pose_ref, const geometry::Pose& pose_other) {
	return Pose(
		pose_ref.getX() + pose_other.getX(),
		pose_ref.getY() + pose_other.getY(),
		pose_ref.getZ() + pose_other.getZ(),
		pose_ref.getRoll() + pose_other.getRoll(),
		pose_ref.getPitch() + pose_other.getPitch(),
		pose_ref.getYaw() + pose_other.getYaw()
	);
}

} // namespace hubero_local_planner
