#include <humap_local_planner/geometry/pose.h>

namespace humap_local_planner {
namespace geometry {

Pose::Pose(double x, double y, double yaw): Pose::Pose(x, y, 0.0, 0.0, 0.0, yaw) {}

Pose::Pose(double x, double y, double z, double roll, double pitch, double yaw):
    pose_(x, y, z, roll, pitch, yaw) {}

Pose::Pose(
    double translation_x,
    double translation_y,
    double translation_z,
    double orientation_x,
    double orientation_y,
    double orientation_z,
    double orientation_w):
    // NOTE: quaternion components order
    pose_(translation_x, translation_y, translation_z, orientation_w, orientation_x, orientation_y, orientation_z) {}

Pose::Pose(const ignition::math::Vector3d& pos, const ignition::math::Quaterniond& quat):
    pose_(ignition::math::Pose3d(pos, quat)) {}

Pose::Pose(const Vector& position, const Quaternion& orientation):
    Pose::Pose(position.getRawVector(), orientation.getRawQuaternion()) {}

Pose::Pose(const geometry_msgs::Twist& twist):
    Pose::Pose(twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z) {}

Pose::Pose(const tf::Stamped<tf::Pose>& pose):
    Pose::Pose(
        pose.getOrigin().x(),
        pose.getOrigin().y(),
        pose.getOrigin().z(),
        pose.getRotation().x(),
        pose.getRotation().y(),
        pose.getRotation().z(),
        pose.getRotation().w()
    ) {}

Pose::Pose(const geometry_msgs::Pose& pose):
    Pose::Pose(
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ) {}

Pose::Pose(const geometry_msgs::PoseStamped& pose): Pose::Pose(pose.pose) {}

Pose::Pose(const geometry_msgs::Transform& pose):
    Pose::Pose(
        pose.translation.x,
        pose.translation.y,
        pose.translation.z,
        pose.rotation.x,
        pose.rotation.y,
        pose.rotation.z,
        pose.rotation.w
    ) {}

Pose::Pose(const teb_local_planner::PoseSE2& pose): Pose::Pose(pose.x(), pose.y(), pose.theta()) {}

void Pose::setPosition(const double& x, const double& y, const double& z) {
    setPositionX(x);
    setPositionY(y);
    setPositionZ(z);
}

void Pose::setPosition(const Vector& pos) {
    setPositionX(pos.getX());
    setPositionY(pos.getY());
    setPositionZ(pos.getZ());
}

void Pose::setPositionX(const double& x) {
    pose_.Pos().X(x);
}

void Pose::setPositionY(const double& y) {
    pose_.Pos().Y(y);
}

void Pose::setPositionZ(const double& z) {
    pose_.Pos().Z(z);
}


void Pose::setOrientation(const double& roll, const double& pitch, const double& yaw) {
    pose_.Rot().Euler(roll, pitch, yaw);
}

void Pose::setOrientation(const double& x, const double& y, const double& z, const double& w) {
    setOrientationX(x);
    setOrientationY(y);
    setOrientationZ(z);
    setOrientationW(w);
}

void Pose::setOrientationX(const double& x) {
    pose_.Rot().X(x);
}

void Pose::setOrientationY(const double& y) {
    pose_.Rot().Y(y);
}

void Pose::setOrientationZ(const double& z) {
    pose_.Rot().Z(z);
}

void Pose::setOrientationW(const double& w) {
    pose_.Rot().W(w);
}


tf::Stamped<tf::Pose> Pose::getAsTfPose() const {
    tf::Stamped<tf::Pose> pose;
    tf::Vector3 translation(
        getX(),
        getY(),
        getZ()
    );
    pose.setOrigin(translation);
    tf::Quaternion quat(
        getQuaternionX(),
        getQuaternionY(),
        getQuaternionZ(),
        getQuaternionW()
    );
    pose.setRotation(quat);
    return pose;
}

geometry_msgs::Pose Pose::getAsMsgPose() const {
    geometry_msgs::Pose pose;
    pose.position.x = getX();
    pose.position.y = getY();
    pose.position.z = getZ();
    pose.orientation.x = getQuaternionX();
    pose.orientation.y = getQuaternionY();
    pose.orientation.z = getQuaternionZ();
    pose.orientation.w = getQuaternionW();
    return pose;
}

geometry_msgs::Transform Pose::getAsTfTransform() const {
    geometry_msgs::Transform tf_stamp;
    tf_stamp.translation.x = getX();
    tf_stamp.translation.y = getY();
    tf_stamp.translation.z = getZ();

    tf_stamp.rotation.x = getQuaternionX();
    tf_stamp.rotation.y = getQuaternionY();
    tf_stamp.rotation.z = getQuaternionZ();
    tf_stamp.rotation.w = getQuaternionW();
    return tf_stamp;
}

teb_local_planner::PoseSE2 Pose::getAsTebPose() const {
    return teb_local_planner::PoseSE2(
        getX(),
        getY(),
        getYaw()
    );
}

Eigen::Vector3f Pose::getAsEigen2D() const {
    return Eigen::Vector3f(
        getX(),
        getY(),
        getYaw()
    );
}

}; // namespace geometry
}; // namespace humap_local_planner
