#pragma once

#include <ignition/math/Quaternion.hh>

namespace hubero {
namespace geometry {

class Quaternion {
public:
    explicit Quaternion(const double& yaw);
    explicit Quaternion(const ignition::math::Quaterniond& quaternion);
    explicit Quaternion(const double& roll, const double& pitch, const double& yaw);
    explicit Quaternion(const double& x, const double& y, const double& z, const double& w);

    double getX() const {
        return quat_.X();
    }

    double getY() const {
        return quat_.Y();
    }

    double getZ() const {
        return quat_.Z();
    }

    double getW() const {
        return quat_.W();
    }

    double getRoll() const {
        return quat_.Roll();
    }

    double getPitch() const {
        return quat_.Pitch();
    }

    double getYaw() const {
        return quat_.Yaw();
    }

    ignition::math::Quaterniond getRawQuaternion() const {
        return quat_;
    }

protected:
    ignition::math::Quaterniond quat_;
};

}; // namespace geometry
}; // namespace hubero
