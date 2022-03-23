#include <hubero_local_planner/geometry/vector.h>
#include <ignition/math/Pose3.hh> // Quaternion

namespace hubero_local_planner {
namespace geometry {

Vector::Vector(const double& x, const double& y, const double& z): v_(x, y, z) {}

Vector::Vector(const ignition::math::Vector3d& vector): v_(vector) {}

Vector::Vector(const Eigen::Vector2d& v): Vector(v[0], v[1], 0.0) {}

Vector::Vector(const Eigen::Vector3d& v): Vector(v[0], v[1], v[2]) {}

Vector::Vector(const geometry_msgs::Twist& twist): Vector(twist.linear.x, twist.linear.y, twist.angular.z) {}

Vector::Vector(const Angle& angle): Vector::Vector(1.0, 0.0, 0.0) {
    rotate(angle);
}

// FIXME
//Vector::Vector(const Pose& pose): Vector(pose.getX(), pose.getY(), pose.getYaw()) {}

Vector::Vector(const tf::Stamped<tf::Pose>& pose):
    Vector::Vector(pose.getOrigin().getX(), pose.getOrigin().getY(), tf::getYaw(pose.getRotation())) {}

Vector::Vector(const geometry_msgs::Pose& pose) {
    ignition::math::Vector3d pos(pose.position.x, pose.position.y, pose.position.z);
    ignition::math::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    v_ = ignition::math::Vector3d(pos.X(), pos.Y(), quat.Yaw());
}

Vector::Vector(const geometry_msgs::PoseStamped& pose): Vector(pose.pose) {}

Angle Vector::calculateDirection() const {
    ignition::math::Vector3d v_norm = v_.Normalized();
    return Angle(std::atan2(v_norm.Y(), v_norm.X()));
}

Vector Vector::normalized() const {
    return Vector(v_.Normalized());
}

Vector Vector::normalize() {
    v_.Normalize();
    return *this;
}

Vector Vector::rotate(const double& angle_rad) {
    double x = getX();
    double y = getY();
    double z = getZ();
    setX(x * cos(angle_rad) - y * sin(angle_rad) + z * 0.0);
	setY(x * sin(angle_rad) + y * cos(angle_rad) + z * 0.0);
    setZ(x * 0.0            + y * 0.0            + z * 1.0);
    return *this;
}

Vector Vector::rotate(const Angle& angle) {
    return rotate(angle.normalized().getRadian());
}

Vector Vector::rotated(const double& angle_rad) const {
    Vector v_copy = *this;
    v_copy.rotate(angle_rad);
    return v_copy;
}

Vector Vector::rotated(const Angle& angle) const {
    return rotated(angle.normalized().getRadian());
}

// Vector Vector::rotateInverse(const double& angle_rad) {
// }

// Vector Vector::rotateInverse(const Angle& angle) {
// }

// Vector Vector::rotatedInverse(const double& angle_rad) const {
// }

// Vector Vector::rotatedInverse(const Angle& angle) const {
// }

geometry_msgs::Pose Vector::getAsMsgPose() const {
    geometry_msgs::Pose pose;
    pose.position.x = getX();
    pose.position.y = getY();
    pose.position.z = getZ();
    return pose;
}

geometry_msgs::Twist Vector::getAsTwist() const {
    geometry_msgs::Twist twist;
    twist.linear.x = getX();
    twist.linear.y = getY();
    twist.angular.z = getZ();
    return twist;
}

}; // namespace geometry
}; // namespace hubero_local_planner
