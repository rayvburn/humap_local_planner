#pragma once

#include <humap_local_planner/geometry/vector.h>

#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>

namespace humap_local_planner {
namespace geometry {

class Vector;
class Angle {
public:
    explicit Angle(const double& angle_rad = 0.0, bool normalize = true);
    explicit Angle(const ignition::math::Angle& angle, bool normalize = true);

    /**
     * // FIXME: Problematic copy constructor
     * \details "no matching function for call to ‘geometry::Angle::Angle(geometry::Angle)’"
     */
    // explicit Angle(const Angle& obj);

    /**
     * @defgroup vectorctors Constructor that calculate direction of the vector (XY plane)
     * @{
     * @details Delegated constructors with additional bool parameter were problematic.
     * However, vector direction calculation returns normalized value.
     */
    explicit Angle(const ignition::math::Vector3d& vector);
    explicit Angle(const Vector& vector);
    /// @}

    void setRadian(const double& angle, bool normalize = true);

    Angle normalized() const;
    Angle normalize();

    double getRadian() const {
        return angle_.Radian();
    }

    double getDegree() const {
        return angle_.Degree();
    }

    /**
     * Returns the wrapped class' object
     */
    ignition::math::Angle getRawAngle() const {
        return angle_;
    }

    /**
     * @defgroup operators Operators
     * @{
     */
    /// \brief Substraction, result = this - angle
    /// \param[in] angle Angle for substraction
    /// \return the new angle
    /// \details Does not normalize Angle
    Angle operator-(const Angle& angle) const {
        return Angle(angle_ - angle.getRawAngle(), false);
    }

    /// \brief Addition operator, result = this + angle
    /// \param[in] angle Angle for addition
    /// \return the new angle
    /// \details Does not normalize Angle
    Angle operator+(const Angle& angle) const {
        return Angle(angle_ + angle.getRawAngle(), false);
    }

    /// \brief Multiplication operator, result = this * angle
    /// \param[in] angle Angle for multiplication
    /// \return the new angle
    /// \details Does not normalize Angle
    Angle operator*(const Angle& angle) const {
        return Angle(angle_ * angle.getRawAngle(), false);
    }

    /// \brief Division, result = this / angle
    /// \param[in] angle Angle for division
    /// \return the new angle
    /// \details Does not normalize Angle
    Angle operator/(const Angle& angle) const {
        return Angle(angle_ / angle.getRawAngle(), false);
    }

    /// \brief Subtraction set, this = this - angle
    /// \param[in] angle Angle for subtraction
    /// \return angle
    /// \details Does not normalize Angle
    Angle operator-=(const Angle& angle) {
        angle_ -= angle.getRawAngle();
        return Angle(*this);
    }

    /// \brief Addition set, this = this + angle
    /// \param[in] angle Angle for addition
    /// \return angle
    /// \details Does not normalize Angle
    Angle operator+=(const Angle& angle) {
        angle_ += angle.getRawAngle();
        return Angle(*this);
    }

    /// \brief Multiplication set, this = this * angle
    /// \param[in] angle Angle for multiplication
    /// \return angle
    /// \details Does not normalize Angle
    Angle operator*=(const Angle& angle) {
        angle_ *= angle.getRawAngle();
        return Angle(*this);
    }

    /// \brief Division set, this = this / angle
    /// \param[in] angle Angle for division
    /// \return angle
    /// \details Does not normalize Angle
    Angle operator/=(const Angle& angle) {
        angle_ /= angle.getRawAngle();
        return Angle(*this);
    }

    /// \brief Equality operator, result = this == angle
    /// \param[in] angle Angle to check for equality
    /// \return true if this == angle
    bool operator==(const Angle& angle) const {
        return angle_ == angle.getRawAngle();
    }

    /// \brief Inequality
    /// \param[in] angle Angle to check for inequality
    /// \return true if this != angle
    bool operator!=(const Angle& angle) const {
        return angle_ != angle.getRawAngle();
    }

    /// \brief Less than operator
    /// \param[in] angle Angle to check
    /// \return true if this < angle
    bool operator<(const Angle& angle) const {
        return angle_ < angle.getRawAngle();
    }

    /// \brief Less or equal operator
    /// \param[in] angle Angle to check
    /// \return true if this <= angle
    bool operator<=(const Angle& angle) const {
        return angle_ <= angle.getRawAngle();
    }

    /// \brief Greater than operator
    /// \param[in] angle Angle to check
    /// \return true if this > angle
    bool operator>(const Angle& angle) const {
        return angle_ > angle.getRawAngle();
    }

    /// \brief Greater or equal operator
    /// \param[in] angle Angle to check
    /// \return true if this >= angle
    bool operator>=(const Angle& angle) const {
        return angle_ >= angle.getRawAngle();
    }
    /// @}
protected:
    ignition::math::Angle angle_;
};

}; // namespace geometry
}; // namespace humap_local_planner
