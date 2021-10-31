#pragma once

#include <hubero_local_planner/geometry/angle.h>

#include <Eigen/Core>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <ignition/math/Vector3.hh>
#include <tf/transform_datatypes.h>

#include <math.h>

namespace hubero {
namespace geometry {

class Angle;
// Abstraction layer between different types that represent 3-element Vector
class Vector {
public:
    Vector(const double& x = 0, const double& y = 0, const double& z = 0);
    Vector(const ignition::math::Vector3d& vector);
    Vector(const Eigen::Vector2d& v);
    Vector(const Eigen::Vector3d& v);
    Vector(const geometry_msgs::Twist& twist);

    /**
     * Creates a unit vector directed according to @ref angle
     */
    Vector(const Angle& angle);

    /**
     * @defgroup pose2d Constructors that convert 3D pose into 2D pose
     * @{
     */
    //Vector(const Pose& pose); // circular dependency
    Vector(const tf::Stamped<tf::Pose>& pose);
    Vector(const geometry_msgs::Pose& pose);
    Vector(const geometry_msgs::PoseStamped& pose);
    // @}

    void setX(const double& value) {
        v_.X(value);
    }
    void setY(const double& value) {
        v_.Y(value);
    }
    void setZ(const double& value) {
        v_.Z(value);
    }

    inline double calculateLength() const {
        return v_.Length();
    }

    Angle calculateDirection() const;

    /// Does not make sense if Pose2D ctor was used
    Vector normalized() const;
    Vector normalize();

    /// Does not make sense if Pose2D ctor was used
    inline double dot(const Vector& vector) const {
        return v_.Dot(vector.getVector());
    }

    /// Does not make sense if Pose2D ctor was used
    inline Vector cross(const Vector& vector) const {
        return v_.Cross(vector.getVector());
    }

    /**
     * Performs rotation around Z axis
     */
    Vector rotate(const double& angle_rad);
    Vector rotate(const Angle& angle);
    Vector rotated(const double& angle_rad) const;
    Vector rotated(const Angle& angle) const;

    // /**
    //  * Performs inverse rotation around Z axis
    //  */
    // Vector rotateInverse(const double& angle_rad);
    // Vector rotateInverse(const Angle& angle);
    // Vector rotatedInverse(const double& angle_rad) const;
    // Vector rotatedInverse(const Angle& angle) const;

    inline double getX() const {
        return v_.X();
    }
    inline double getY() const {
        return v_.Y();
    }
    inline double getZ() const {
        return v_.Z();
    }
    // getRawVector
    inline ignition::math::Vector3d getVector() const {
        return v_;
    }

    /// NOTE: makes sense only if one of the Pose-connected constructors was used (getZ returns Yaw then)
    geometry_msgs::Pose getAsMsgPose() const;

	Eigen::Vector3d getAsEigen() const;

    geometry_msgs::Twist getAsTwist() const;

    /**
     * \addtogroup operators Operators with doc taken from ignition::math::Vector
     * @{
     */
    /// \brief Assignment operator
    /// \param[in] v a new value
    /// \return this
    Vector& operator=(const Vector& v) {
        v_ = v.getVector();
        return *this;
    }

    /// \brief Assignment operator
    /// \param[in] value assigned to all elements
    /// \return this
    Vector& operator=(double value) {
        v_ = value;
        return *this;
    }

    /// \brief Addition operator
    /// \param[in] v vector to add
    /// \return the sum vector
    Vector operator+(const Vector &v) const {
        return Vector(v_ + v.getVector());
    }

    /// \brief Addition assignment operator
    /// \param[in] v vector to add
    /// \return the sum vector
    const Vector& operator+=(const Vector& v) {
        v_ += v.getVector();
        return *this;
    }

    /// \brief Addition operators
    /// \param[in] s the scalar addend
    /// \return sum vector
    Vector operator+(const double s) const {
        return Vector(v_ + s);
    }

    /// \brief Addition operators
    /// \param[in] s the scalar addend
    /// \param[in] v input vector
    /// \return sum vector
    friend Vector operator+(const double s, const Vector& v) {
        return Vector(v.getX() + s, v.getY() + s, v.getZ() + s);
    }

    /// \brief Addition assignment operator
    /// \param[in] s scalar addend
    /// \return this
    const Vector& operator+=(const double s) {
        v_ += s;
        return *this;
    }

    /// \brief Negation operator
    /// \return negative of this vector
    inline Vector operator-() const {
        return Vector(-v_);
    }

    /// \brief Subtraction operators
    /// \param[in] pt a vector to substract
    /// \return a vector after the substraction
    inline Vector operator-(const Vector &pt) const {
        return Vector(v_ - pt.getVector());
    }

    /// \brief Subtraction assignment operators
    /// \param[in] pt subtrahend
    /// \return a vector after the substraction
    const Vector& operator-=(const Vector& pt) {
        v_ -= pt.getVector();
        return *this;
    }

    /// \brief Subtraction operators
    /// \param[in] s the scalar subtrahend
    /// \return difference vector
    inline Vector operator-(const double s) const {
        return Vector(v_ - s);
    }

    /// \brief Subtraction operators
    /// \param[in] s the scalar minuend
    /// \param[in] v vector subtrahend
    /// \return difference vector
    friend inline Vector operator-(const double s, const Vector& v) {
        return Vector(s - v.getVector());
    }

    /// \brief Subtraction assignment operator
    /// \param[in] s scalar subtrahend
    /// \return this
    const Vector& operator-=(const double s) {
        v_ -= s;
        return *this;
    }

    /// \brief Division operator
    /// \remarks this is an element wise division
    /// \param[in] pt the vector divisor
    /// \return a vector
    const Vector operator/(const Vector& pt) const {
        return Vector(v_ / pt.getVector());
    }

    /// \brief Division assignment operator
    /// \remarks this is an element wise division
    /// \param[in] pt the vector divisor
    /// \return a vector
    const Vector& operator/=(const Vector& pt) {
        v_ /= pt.getVector();
        return *this;
    }

    /// \brief Division operator
    /// \remarks this is an element wise division
    /// \param[in] v the divisor
    /// \return a vector
    const Vector operator/(double v) const {
        return Vector(v_ / v);
    }

    /// \brief Division assignment operator
    /// \remarks this is an element wise division
    /// \param[in] v the divisor
    /// \return this
    const Vector& operator/=(double v) {
        v_ /= v;
        return *this;
    }

    /// \brief Multiplication operator
    /// \remarks this is an element wise multiplication, not a cross product
    /// \param[in] p multiplier operator
    /// \return a vector
    Vector operator*(const Vector &p) const {
        return Vector(v_ * p.getVector());
    }

    /// \brief Multiplication assignment operators
    /// \remarks this is an element wise multiplication, not a cross product
    /// \param[in] v a vector
    /// \return this
    const Vector& operator*=(const Vector& v) {
        v_ *= v.getVector();
        return *this;
    }

    /// \brief Multiplication operators
    /// \param[in] s the scaling factor
    /// \return a scaled vector
    inline Vector operator*(double s) const {
        return Vector(v_ * s);
    }

    /// \brief Multiplication operators
    /// \param[in] s the scaling factor
    /// \param[in] v input vector
    /// \return a scaled vector
    friend inline Vector operator*(double s, const Vector& v) {
        return Vector(v.getVector() * s);
    }

    /// \brief Multiplication operator
    /// \param[in] v scaling factor
    /// \return this
    const Vector& operator*=(double v) {
        v_ *= v;
        return *this;
    }
    /** @}*/ // operators group

protected:
    ignition::math::Vector3d v_;
}; // class Vector

}; // namespace geometry
}; // namespace hubero