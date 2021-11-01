#include <hubero_local_planner/geometry/angle.h>

namespace hubero {
namespace geometry {

Angle::Angle(const double& angle_rad, bool normalize): angle_(angle_rad) {
    if (normalize) {
        angle_.Normalize();
    }
}

Angle::Angle(const ignition::math::Angle& angle, bool normalize): Angle::Angle(angle.Radian(), normalize) {}

// Angle::Angle(const Angle& obj): angle_(obj.getRawAngle()) {}

Angle::Angle(const ignition::math::Vector3d& vector): Angle(std::atan2(vector.Y(), vector.X()), false) {}

Angle::Angle(const Vector& vector): Angle::Angle(vector.getRawVector()) {}

void Angle::setRadian(const double& angle, bool normalize) {
    angle_.Radian(angle);
    if (normalize) {
        angle_.Normalize();
    }
}

Angle Angle::normalized() const {
    Angle angle_temp(angle_);
    angle_temp.normalize();
    return angle_temp;
}

Angle Angle::normalize() {
    angle_.Normalize();
    return *this;
}

}; // namespace geometry
}; // namespace hubero