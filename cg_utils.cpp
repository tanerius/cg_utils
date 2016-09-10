#include <cmath>
#include <float.h>
#include "cg_utils.hpp"

using namespace CGutils;

// Vector addition.
Vector  Vector::operator+(const Vector& v) const
{
    Vector  result;
    result.x[0] = x[0] + v.x[0];
    result.x[1] = x[1] + v.x[1];
    result.x[2] = x[2] + v.x[2];
    return result;
}


// Vector subtraction.
Vector  Vector::operator-(const Vector& v) const
{
    Vector  result;
    result.x[0] = x[0] - v.x[0];
    result.x[1] = x[1] - v.x[1];
    result.x[2] = x[2] - v.x[2];
    return result;
}


// Returns negative of given vector.
Vector  Vector::operator-() const
{
    Vector  result;
    result.x[0] = -x[0];
    result.x[1] = -x[1];
    result.x[2] = -x[2];
    return result;
}


// Scalar multiple of vector (kV)
Vector  Vector::operator*(float f) const
{
    Vector  result;
    result.x[0] = x[0] * f;
    result.x[1] = x[1] * f;
    result.x[2] = x[2] * f;
    return result;
}


// Cross product.
Vector  Vector::Cross(const Vector& v) const
{
    Vector  result;
    result.x[0] = x[1] * v.x[2] - x[2] * v.x[1];
    result.x[1] = x[2] * v.x[0] - x[0] * v.x[2];
    result.x[2] = x[0] * v.x[1] - x[1] * v.x[0];
    return result;
}


// Scales the Vector to unit length.  Preserves its direction.
Vector& Vector::Normalize()
{
    float   f = Magnitude();
    if (f < 0.0000001) {
        x[0] = 1;
        x[1] = 0;
        x[2] = 0;
    } else {
        this->operator/=(f);
    }
    return *this;
}


// Scalar multiple of vector (kV) (this vector)
Vector& Vector::operator*=(float f)
{
    x[0] *= f;
    x[1] *= f;
    x[2] *= f;
    return *this;
}


// Return magnitude of the vector
float   Vector::Magnitude() const
{
    return sqrt(Sqrmag());
}


// Returns the square of the length 
float   Vector::Sqrmag() const
{
    return x[0]*x[0] + x[1]*x[1] + x[2]*x[2];
}


// Check if a vector component is not a number
bool    Vector::CheckNaN() const
{
    if (fabs(x[0]) > 10000000 || fabs(x[1]) > 10000000 || fabs(x[2]) > 10000000) {
        return true;
    }
    if (isnan(x[0]) || isnan(x[1]) || isnan(x[2])) {
        return true;
    }
    else return false;
}

// Define the default basic vectors
Vector  ZeroVector(0, 0, 0);
Vector  XAxis(1, 0, 0);
Vector  YAxis(0, 1, 0);
Vector  ZAxis(0, 0, 1);





/*
* Quaternions implementation
*/
//
// class Quaternion
//


Quaternion::Quaternion(const Vector& _axis, float _angle /* in radians */)
{
    scalar = cos(_angle / 2);
    vector = _axis;
    vector *= sin(_angle / 2);
}


Quaternion  Quaternion::operator*(const Quaternion& _q) const
{
    return Quaternion(
        scalar * _q.scalar - vector * _q.vector, 
        _q.vector * scalar + vector * _q.scalar + vector.Cross(_q.vector)
        );
}

// Ensures that the Quaternion has magnitude 1.
Quaternion& Quaternion::Normalize()
{
    float   mag = sqrt(scalar * scalar + vector * vector);
    if (mag > 0.0000001) {
        float   inv = 1.0 / mag;
        scalar *= inv;
        vector *= inv;
    } else {
        // No rotation! Bad quaternion
        scalar = 1;
        vector = ZeroVector;
    }

    return *this;
}


Quaternion& Quaternion::operator*=(const Quaternion& _q)
{
    *this = *this * _q; 

    return *this;
}


// Rotates the given vector v by the rotation represented by this Quaternion.
// Stores the result in the given _result vector.
void    Quaternion::ApplyRotation(Vector* _result, const Vector& _v)
{
    Quaternion  q(*this * Quaternion(0, _v) * Quaternion(scalar, -vector));    

    *_result = q.vector;
}


// Does a spherical linear interpolation between *this and q, using f as
// the blend factor.  f == 0 --> result == *this, f == 1 --> result == q.
Quaternion  Quaternion::Lerp(const Quaternion& _q, float _f) const
{
    Quaternion result;

    float f0, f1;

    float cos_omega = vector * _q.vector + scalar * _q.scalar;
    Quaternion  qtemp(_q);

    // Adjust signs if necessary.
    if (cos_omega < 0) {
        cos_omega = -cos_omega;
        qtemp.vector = -qtemp.vector;
        qtemp.scalar = -qtemp.scalar;
    }

    if (cos_omega < 0.99) {
        // Do the spherical interp.
        float   omega = acos(cos_omega);
        float   sin_omega = sin(omega);
        f0 = sin((1 - _f) * omega) / sin_omega;
        f1 = sin(_f * omega) / sin_omega;
    } else {
        // Quaternions are close; just do straight lerp and avoid division by near-zero.
        f0 = 1 - _f;
        f1 = _f;
    }
    
    result.scalar = scalar * f0 + qtemp.scalar * f1;
    result.vector = vector * f0 + qtemp.vector * f1;
    result.Normalize();

    return result;
}






// Rotates the given point through the given angle (in radians) about the given
// axis.
Vector Rotate(float _angle, const Vector& _axis, const Vector& _point)
{
    Quaternion q(cos(_angle/2), _axis * sin(_angle/2));
    Vector result;
    q.ApplyRotation(&result, _point);

    return result;
}

