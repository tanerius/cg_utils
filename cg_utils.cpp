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



/*
* Matrix implementation
*/

// Sets *this to be an identity Matrix.
void Matrix::Identity()
{
    SetColumn(0, XAxis);
    SetColumn(1, YAxis);
    SetColumn(2, ZAxis);
    SetColumn(3, ZeroVector);
}


// Turns *this into a view Matrix, given the direction the camera is
// looking (ViewNormal) and the camera's up vector (ViewUp), and its
// location (ViewLocation) (all vectors in world-coordinates).  The
// resulting Matrix will transform points from world coordinates to view
// coordinates, which is a right-handed system with the x axis pointing
// left, the y axis pointing up, and the z axis pointing into the scene.
void Matrix::View(const Vector& _vViewNormal, const Vector& _vViewUp, const Vector& _vViewLocation)
{
    Vector _vViewX = _vViewUp.Cross(_vViewNormal);

    // Construct the view-to-world Matrix.
    Orient(_vViewX, _vViewUp, _vViewLocation);

    // Turn it around, to make it world-to-view.
    Invert();
}


// Turns *this into a transformation Matrix, that transforms vectors
// from object coordinates to world coordinates, given the object's Direction, Up,
// and Location in world coordinates.
void Matrix::Orient(const Vector& _vDirection, const Vector& _vUp, const Vector& _vLocation)
{
    Vector vZAxis = _vDirection.Cross(_vUp);

    SetColumn(0, _vDirection);
    SetColumn(1, _vUp);
    SetColumn(2, vZAxis);
    SetColumn(3, _vLocation);
}


// Applies *this to the given vector, and returns the transformed vector.
Vector Matrix::operator*(const Vector& _v) const
{
    Vector result;
    Apply(&result, _v);
    return result;
}


// Composes the two matrices, returns the product.  Creates a temporary
// for the return value.
Matrix Matrix::operator*(const Matrix& _a) const
{
    Matrix result;
    Compose(&result, *this, _a);

    return result;
}


// Multiplies left * right, and puts the result in *dest.
void Matrix::Compose(Matrix* _dest, const Matrix& _left, const Matrix& _right)
{
    _left.ApplyRotation(&const_cast<Vector&>(_dest->GetColumn(0)), _right.GetColumn(0));
    _left.ApplyRotation(&const_cast<Vector&>(_dest->GetColumn(1)), _right.GetColumn(1));
    _left.ApplyRotation(&const_cast<Vector&>(_dest->GetColumn(2)), _right.GetColumn(2));
    _left.Apply(&const_cast<Vector&>(_dest->GetColumn(3)), _right.GetColumn(3));
}


// Scalar multiply of a Matrix.
Matrix& Matrix::operator*=(float _f)
{
    int i;
    for (i = 0; i < 4; i++) m[i] *= _f;
    return *this;
}


// Memberwise Matrix addition.
Matrix& Matrix::operator+=(const Matrix& _mat)
{
    int i;
    for (i = 0; i < 4; i++) m[i] += _mat.m[i];
    return *this;
}


// Inverts *this.  Uses transpose property of rotation matrices.
void Matrix::Invert()
{
    InvertRotation();
    // Compute the translation part of the inverted Matrix, by applying
    // the inverse rotation to the original translation.
    Vector  vTransPrime;
    ApplyRotation(&vTransPrime, GetColumn(3));
    SetColumn(3, -vTransPrime);
}


// Inverts the rotation part of *this.  Ignores the translation.
// Uses the transpose property of rotation matrices.
void Matrix::InvertRotation()
{
    float   f;

    // Swap elements across the diagonal.
    f = m[1].Get(0);
    m[1].Set(0, m[0].Get(1));
    m[0].Set(1, f);

    f = m[2].Get(0);
    m[2].Set(0, m[0].Get(2));
    m[0].Set(2, f);

    f = m[2].Get(1);
    m[2].Set(1, m[1].Get(2));
    m[1].Set(2, f);
}


// Normalizes the rotation part of the Matrix.
void Matrix::NormalizeRotation()
{
    m[0].Normalize();
    m[1] = m[2].Cross(m[0]);
    m[1].Normalize();
    m[2] = m[0].Cross(m[1]);
}


// Applies v to *this, and puts the transformed result in *result.
void Matrix::Apply(Vector* _result, const Vector& _v) const
{
    // Do the rotation.
    ApplyRotation(_result, _v);
    // Do the translation.
    *_result += m[3];
}


// Applies the rotation portion of *this, and puts the transformed result in *result.
void Matrix::ApplyRotation(Vector* _result, const Vector& _v) const
{
    _result->Set(0, m[0].Get(0) * _v.Get(0) + m[1].Get(0) * _v.Get(1) + m[2].Get(0) * _v.Get(2));
    _result->Set(1, m[0].Get(1) * _v.Get(0) + m[1].Get(1) * _v.Get(1) + m[2].Get(1) * _v.Get(2));
    _result->Set(2, m[0].Get(2) * _v.Get(0) + m[1].Get(2) * _v.Get(1) + m[2].Get(2) * _v.Get(2));
}


// Applies v to the inverse of *this, and puts the transformed result in *result.
void Matrix::ApplyInverse(Vector* _result, const Vector& _v) const
{
    ApplyInverseRotation(_result, _v - m[3]);
}


// Applies v to the inverse rotation part of *this, and puts the result in *result.
void Matrix::ApplyInverseRotation(Vector* _result, const Vector& _v) const
{
    _result->Set(0, m[0] * _v);
    _result->Set(1, m[1] * _v);
    _result->Set(2, m[2] * _v);
}


// Composes a translation on the right of *this.
void Matrix::Translate(const Vector& _v)
{
    Vector newtrans;
    Apply(&newtrans, _v);
    SetColumn(3, newtrans);
}


// Sets the rotation part of the Matrix to the values which correspond to the given
// quaternion orientation.
void Matrix::SetOrientation(const Quaternion& _q)
{
    float S = _q.GetS();
    const Vector& v = _q.GetV();
    
    m[0].Set(0, 1 - 2 * v.GetY() * v.GetY() - 2 * v.GetZ() * v.GetZ());
    m[0].Set(1, 2 * v.GetX() * v.GetY() + 2 * S * v.GetZ());
    m[0].Set(2, 2 * v.GetX() * v.GetZ() - 2 * S * v.GetY());

    m[1].Set(0, 2 * v.GetX() * v.GetY() - 2 * S * v.GetZ());
    m[1].Set(1, 1 - 2 * v.GetX() * v.GetX() - 2 * v.GetZ() * v.GetZ());
    m[1].Set(2, 2 * v.GetY() * v.GetZ() + 2 * S * v.GetX());

    m[2].Set(0, 2 * v.GetX() * v.GetZ() + 2 * S * v.GetY());
    m[2].Set(1, 2 * v.GetY() * v.GetZ() - 2 * S * v.GetX());
    m[2].Set(2, 1 - 2 * v.GetX() * v.GetX() - 2 * v.GetY() * v.GetY());
}


// Converts the rotation part of *this into a quaternion, and returns it.
Quaternion Matrix::GetOrientation() const
{
    // Code adapted from Baraff, "Rigid Body Simulation", from SIGGRAPH 95 course notes for Physically Based Modeling.
    Quaternion  q;
    float   tr, s;

    tr = m[0].Get(0) + m[1].Get(1) + m[2].Get(2);   // trace

    if (tr >= 0) {
        s = sqrt(tr + 1);
        q.SetS(0.5 * s);
        s = 0.5 / s;
        q.SetV(Vector(m[1].Get(2) - m[2].Get(1), m[2].Get(0) - m[0].Get(2), m[0].Get(1) - m[1].Get(0)) * s);
    } else {
        int i = 0;

        if (m[1].Get(1) > m[0].Get(0)) {
            i = 1;
        }
        if (m[2].Get(2) > m[i].Get(i)) {
            i = 2;
        }

        float   qr, qi, qj, qk;
        switch (i) {
        case 0:
            s = sqrt((m[0].Get(0) - (m[1].Get(1) + m[2].Get(2))) + 1);
            qi = 0.5 * s;
            s = 0.5 / s;
            qj = (m[1].Get(0) + m[0].Get(1)) * s;
            qk = (m[0].Get(2) + m[2].Get(0)) * s;
            qr = (m[1].Get(2) - m[2].Get(1)) * s;
            break;

        case 1:
            s = sqrt((m[1].Get(1) - (m[2].Get(2) + m[0].Get(0))) + 1);
            qj = 0.5 * s;
            s = 0.5 / s;
            qk = (m[2].Get(1) + m[1].Get(2)) * s;
            qi = (m[1].Get(0) + m[0].Get(1)) * s;
            qr = (m[2].Get(0) - m[0].Get(2)) * s;
            break;

        case 2:
            s = sqrt((m[2].Get(2) - (m[0].Get(0) + m[1].Get(1))) + 1);
            qk = 0.5 * s;
            s = 0.5 / s;
            qi = (m[0].Get(2) + m[2].Get(0)) * s;
            qj = (m[2].Get(1) + m[1].Get(2)) * s;
            qr = (m[0].Get(1) - m[1].Get(0)) * s;
            break;
        }
        q.SetS(qr);
        q.SetV(Vector(qi, qj, qk));
    }

    return q;
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

