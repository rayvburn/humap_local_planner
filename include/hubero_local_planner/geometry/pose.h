#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <hubero_local_planner/geometry/vector.h>
#include <hubero_local_planner/geometry/quaternion.h>
#include <ignition/math/Pose3.hh>
#include <teb_local_planner/pose_se2.h>
#include <tf/transform_datatypes.h>

namespace hubero_local_planner {
namespace geometry {

// Abstraction layer between different types that represent Pose
// Potential stamps (mostly ROS-related) must be added separately
class Pose {
public:
    Pose(double x = 0.0, double y = 0.0, double yaw = 0.0);
    Pose(double x, double y, double z, double roll, double pitch, double yaw);
    Pose(
        double translation_x,
        double translation_y,
        double translation_z,
        double orientation_x,
        double orientation_y,
        double orientation_z,
        double orientation_w
    );
    Pose(
        const ignition::math::Vector3d& pos,
        const ignition::math::Quaterniond& quat = ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0)
    );
    Pose(const Vector& position, const Quaternion& orientation);
    Pose(const geometry_msgs::Twist& twist);
    Pose(const tf::Stamped<tf::Pose>& pose);
    Pose(const geometry_msgs::Pose& pose);
    Pose(const geometry_msgs::PoseStamped& pose);
    Pose(const geometry_msgs::Transform& pose);
    Pose(const teb_local_planner::PoseSE2& pose);

    void setPosition(const double& x, const double& y, const double& z);
    void setPosition(const Vector& pos);
    void setPositionX(const double& x);
    void setPositionY(const double& y);
    void setPositionZ(const double& z);

    void setOrientation(const double& roll, const double& pitch, const double& yaw);
    void setOrientation(const double& x, const double& y, const double& z, const double& w);
    void setOrientationX(const double& x);
    void setOrientationY(const double& y);
    void setOrientationZ(const double& z);
    void setOrientationW(const double& w);

    inline double getX() const {
        return pose_.Pos().X();
    }
    inline double getY() const {
        return pose_.Pos().Y();
    }
    inline double getZ() const {
        return pose_.Pos().Z();
    }
    inline double getRoll() const {
        return pose_.Rot().Roll();
    }
    inline double getPitch() const {
        return pose_.Rot().Pitch();
    }
    inline double getYaw() const {
        return pose_.Rot().Yaw();
    }

    inline double getQuaternionX() const {
        return pose_.Rot().X();
    }
    inline double getQuaternionY() const {
        return pose_.Rot().Y();
    }
    inline double getQuaternionZ() const {
        return pose_.Rot().Z();
    }
    inline double getQuaternionW() const {
        return pose_.Rot().W();
    }

    inline ignition::math::Pose3d getRawPose() const {
        return pose_;
    }

    inline ignition::math::Vector3d getRawPosition() const {
        return pose_.Pos();
    }

    inline Vector getPosition() const {
        return Vector(getRawPosition());
    }

    inline ignition::math::Quaterniond getRawOrientation() const {
        return pose_.Rot();
    }

    inline Quaternion getOrientation() const {
        return Quaternion(pose_.Rot());
    }

    // TODO: operators?

    tf::Stamped<tf::Pose> getAsTfPose() const;

    geometry_msgs::Pose getAsMsgPose() const;

    geometry_msgs::Transform getAsTfTransform() const;

    teb_local_planner::PoseSE2 getAsTebPose() const;

protected:
    ignition::math::Pose3d pose_;
}; // class Pose

}; // namespace geometry
}; // namespace hubero_local_planner
