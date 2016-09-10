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

