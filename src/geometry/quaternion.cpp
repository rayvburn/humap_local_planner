#include <hubero_local_planner/geometry/quaternion.h>

namespace hubero_local_planner {
namespace geometry {

Quaternion::Quaternion(const double& yaw): quat_(0.0, 0.0, yaw) {}
Quaternion::Quaternion(const ignition::math::Quaterniond& quaternion): quat_(quaternion) {}
Quaternion::Quaternion(const double& roll, const double& pitch, const double& yaw): quat_(roll, pitch, yaw) {}
Quaternion::Quaternion(const double& x, const double& y, const double& z, const double& w): quat_(w, x, y, z) {}

}; // namespace geometry
}; // namespace hubero_local_planner
