#include <cmath>
#include <float.h>
#include <cassert>
#include "cg_utils.hpp"

using namespace CGutils;


// Check if a CGVector3D component is not a number
bool    CGVector3D::CheckNaN() const
{
    if (fabs(x[0]) > 10000000 || fabs(x[1]) > 10000000 || fabs(x[2]) > 10000000) {
        return true;
    }
    if (isnan(x[0]) || isnan(x[1]) || isnan(x[2])) {
        return true;
    }
    else return false;
}



// CGVector3D addition.
CGVector3D  CGVector3D::operator+(const CGVector3D& v) const
{
    CGVector3D  result;
    result.x[0] = x[0] + v.x[0];
    result.x[1] = x[1] + v.x[1];
    result.x[2] = x[2] + v.x[2];
    return result;
}



// CGVector3D subtraction.
CGVector3D  CGVector3D::operator-(const CGVector3D& v) const
{
    CGVector3D  result;
    result.x[0] = x[0] - v.x[0];
    result.x[1] = x[1] - v.x[1];
    result.x[2] = x[2] - v.x[2];
    return result;
}



// Returns negative of given CGVector3D.
CGVector3D  CGVector3D::operator-() const
{
    CGVector3D  result;
    result.x[0] = -x[0];
    result.x[1] = -x[1];
    result.x[2] = -x[2];
    return result;
}



// Scalar multiple of CGVector3D (kV)
CGVector3D  CGVector3D::operator*(float f) const
{
    CGVector3D  result;
    result.x[0] = x[0] * f;
    result.x[1] = x[1] * f;
    result.x[2] = x[2] * f;
    return result;
}



// Scalar multiple of CGVector3D (kV) (this CGVector3D)
CGVector3D& CGVector3D::operator*=(float f)
{
    x[0] *= f;
    x[1] *= f;
    x[2] *= f;
    return *this;
}




// Calculate an angle between this CGVector3D and a given one
float   CGVector3D::AngleBetween(const CGVector3D& _v)
{
    CGVector3D a(GetX(),GetY(),GetZ());
    a.NormalizeThis();
    return std::acos(a * CGVector3D::Normalize(_v)); // acos of the dot product compute dot product
}



// Cross product.
// Returns a new copy of a CGVector3D on stack
CGVector3D  CGVector3D::Cross(const CGVector3D& v) const
{
    CGVector3D  result;
    result.x[0] = x[1] * v.x[2] - x[2] * v.x[1];
    result.x[1] = x[2] * v.x[0] - x[0] * v.x[2];
    result.x[2] = x[0] * v.x[1] - x[1] * v.x[0];
    return result;
}



// Scales the CGVector3D to unit length.  Preserves its direction.
// This is an in place normalization which will modify this CGVector3D
void CGVector3D::NormalizeThis()
{
    float   f = Magnitude();
    if (f < 0.0000001) {
        x[0] = 1;
        x[1] = 0;
        x[2] = 0;
    } else {
        this->operator/=(f);
    }
}



// Return magnitude of the CGVector3D
float   CGVector3D::Magnitude() const
{
    return sqrt(Sqrmag());
}



// Multiply a CGVector3Ds components by another given CGVector3D
CGVector3D CGVector3D::MultiplyComponents(const CGVector3D& _v)
{
    CGVector3D ret = CGVector3D(x[0] * _v.GetX(),
        x[1] * _v.GetY(),
        x[2] * _v.GetZ());
    return ret;
}



CGVector3D CGVector3D::RotateAroundAxis(const CGVector3D& _axis, const float _theta)
{
    float u = _axis.GetX();
    float v = _axis.GetY();
    float w = _axis.GetZ();

    float cosTheta = std::cos(_theta);
    float sinTheta = std::sin(_theta);
    float ms = _axis.Sqrmag();
    float m = std::sqrt(ms);
    return CGVector3D(
        ((u * (u * x[0] + v * x[1] + w * x[2])) + 
        (((x[0] * (v * v + w * w)) - (u * (v * x[1] + w * x[2]))) * cosTheta) + 
        (m * ((-w * x[1]) + (v * x[2])) * sinTheta)) / ms,

        ((v * (u * x[0] + v * x[1] + w * x[2])) + 
        (((x[1] * (u * u + w * w)) - (v * (u * x[0] + w * x[2]))) * cosTheta) + 
        (m * ((w * x[0]) - (u * x[2])) * sinTheta)) / ms,

        ((w * (u * x[0] + v * x[1] + w * x[2])) + 
        (((x[2] * (u * u + v * v)) - (w * (u * x[0] + v * x[1]))) * cosTheta) + 
        (m * (-(v * x[0]) + (u * x[1])) * sinTheta)) / ms
    );
}



// Returns the square of the length 
float   CGVector3D::Sqrmag() const
{
    return x[0]*x[0] + x[1]*x[1] + x[2]*x[2];
}



// Define the default basic CGVector3Ds
CGVector3D  ZeroVector3D(0.0f, 0.0f, 0.0f);
CGVector3D  UnitVector3D(1.0f, 1.0f, 1.0f);
CGVector3D  XAxis(1.0f, 0.0f, 0.0f);
CGVector3D  YAxis(0.0f, 1.0f, 0.0f);
CGVector3D  ZAxis(0.0f, 0.0f, 1.0f);





/*
* Quaternions implementation
*/



Quaternion::Quaternion(const CGVector3D& _axis, float _angle /* in radians */)
{
    scalar = cos(_angle / 2);
    v = _axis;
    v *= sin(_angle / 2);
}



Quaternion  Quaternion::operator*(const Quaternion& _q) const
{
    return Quaternion(
        scalar * _q.scalar - v * _q.v, 
        _q.v * scalar + v * _q.scalar + v.Cross(_q.v)
        );
}



// Ensures that the Quaternion has magnitude 1.
Quaternion& Quaternion::Normalize()
{
    float   mag = sqrt(scalar * scalar + v * v);
    if (mag > 0.0000001) {
        float   inv = 1.0 / mag;
        scalar *= inv;
        v *= inv;
    } else {
        // No rotation! Bad quaternion
        scalar = 1;
        v = ZeroVector3D;
    }

    return *this;
}



Quaternion& Quaternion::operator*=(const Quaternion& _q)
{
    *this = *this * _q; 

    return *this;
}



// Rotates the given CGVector3D v by the rotation represented by this Quaternion.
// Stores the result in the given _result CGVector3D.
void    Quaternion::ApplyRotation(CGVector3D* _result, const CGVector3D& _v)
{
    Quaternion  q(*this * Quaternion(0, _v) * Quaternion(scalar, -v));    

    *_result = q.v;
}



// Does a spherical linear interpolation between *this and q, using f as
// the blend factor.  f == 0 --> result == *this, f == 1 --> result == q.
Quaternion  Quaternion::Lerp(const Quaternion& _q, float _f) const
{
    Quaternion result;

    float f0, f1;

    float cos_omega = v * _q.v + scalar * _q.scalar;
    Quaternion  qtemp(_q);

    // Adjust signs if necessary.
    if (cos_omega < 0) {
        cos_omega = -cos_omega;
        qtemp.v = -qtemp.v;
        qtemp.scalar = -qtemp.scalar;
    }

    if (cos_omega < 0.99) {
        // Do the spherical interp.
        float omega = acos(cos_omega);
        float sin_omega = sin(omega);
        f0 = sin((1 - _f) * omega) / sin_omega;
        f1 = sin(_f * omega) / sin_omega;
    } else {
        // Quaternions are close; just do straight lerp and avoid division by near-zero.
        f0 = 1 - _f;
        f1 = _f;
    }
    
    result.scalar = scalar * f0 + qtemp.scalar * f1;
    result.v = v * f0 + qtemp.v * f1;
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
    SetColumn(3, ZeroVector3D);
}



// Turns *this into a view Matrix, given the direction the camera is
// looking (ViewNormal) and the camera's up CGVector3D (ViewUp), and its
// location (ViewLocation) (all CGVector3Ds in world-coordinates).  The
// resulting Matrix will transform points from world coordinates to view
// coordinates, which is a right-handed system with the x axis pointing
// left, the y axis pointing up, and the z axis pointing into the scene.
void Matrix::View(const CGVector3D& _vViewNormal, const CGVector3D& _vViewUp, const CGVector3D& _vViewLocation)
{
    CGVector3D _vViewX = _vViewUp.Cross(_vViewNormal);

    // Construct the view-to-world Matrix.
    Orient(_vViewX, _vViewUp, _vViewLocation);

    // Turn it around, to make it world-to-view.
    Invert();
}



// Turns *this into a transformation Matrix, that transforms CGVector3Ds
// from object coordinates to world coordinates, given the object's Direction, Up,
// and Location in world coordinates.
void Matrix::Orient(const CGVector3D& _vDirection, const CGVector3D& _vUp, const CGVector3D& _vLocation)
{
    CGVector3D vZAxis = _vDirection.Cross(_vUp);

    SetColumn(0, _vDirection);
    SetColumn(1, _vUp);
    SetColumn(2, vZAxis);
    SetColumn(3, _vLocation);
}



// Applies *this to the given CGVector3D, and returns the transformed CGVector3D.
CGVector3D Matrix::operator*(const CGVector3D& _v) const
{
    CGVector3D result;
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
    _left.ApplyRotation(&const_cast<CGVector3D&>(_dest->GetColumn(0)), _right.GetColumn(0));
    _left.ApplyRotation(&const_cast<CGVector3D&>(_dest->GetColumn(1)), _right.GetColumn(1));
    _left.ApplyRotation(&const_cast<CGVector3D&>(_dest->GetColumn(2)), _right.GetColumn(2));
    _left.Apply(&const_cast<CGVector3D&>(_dest->GetColumn(3)), _right.GetColumn(3));
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
    CGVector3D  vTransPrime;
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
    m[0].NormalizeThis();
    m[1] = m[2].Cross(m[0]);
    m[1].NormalizeThis();
    m[2] = m[0].Cross(m[1]);
}



// Applies v to *this, and puts the transformed result in *result.
void Matrix::Apply(CGVector3D* _result, const CGVector3D& _v) const
{
    // Do the rotation.
    ApplyRotation(_result, _v);
    // Do the translation.
    *_result += m[3];
}



// Applies the rotation portion of *this, and puts the transformed result in *result.
void Matrix::ApplyRotation(CGVector3D* _result, const CGVector3D& _v) const
{
    _result->Set(0, m[0].Get(0) * _v.Get(0) + m[1].Get(0) * _v.Get(1) + m[2].Get(0) * _v.Get(2));
    _result->Set(1, m[0].Get(1) * _v.Get(0) + m[1].Get(1) * _v.Get(1) + m[2].Get(1) * _v.Get(2));
    _result->Set(2, m[0].Get(2) * _v.Get(0) + m[1].Get(2) * _v.Get(1) + m[2].Get(2) * _v.Get(2));
}



// Applies v to the inverse of *this, and puts the transformed result in *result.
void Matrix::ApplyInverse(CGVector3D* _result, const CGVector3D& _v) const
{
    ApplyInverseRotation(_result, _v - m[3]);
}



// Applies v to the inverse rotation part of *this, and puts the result in *result.
void Matrix::ApplyInverseRotation(CGVector3D* _result, const CGVector3D& _v) const
{
    _result->Set(0, m[0] * _v);
    _result->Set(1, m[1] * _v);
    _result->Set(2, m[2] * _v);
}



// Composes a translation on the right of *this.
void Matrix::Translate(const CGVector3D& _v)
{
    CGVector3D newtrans;
    Apply(&newtrans, _v);
    SetColumn(3, newtrans);
}



// Sets the rotation part of the Matrix to the values which correspond to the given
// quaternion orientation.
void Matrix::SetOrientation(const Quaternion& _q)
{
    float S = _q.GetS();
    const CGVector3D& v = _q.GetV();
    
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
        q.SetV(CGVector3D(m[1].Get(2) - m[2].Get(1), m[2].Get(0) - m[0].Get(2), m[0].Get(1) - m[1].Get(0)) * s);
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
        q.SetV(CGVector3D(qi, qj, qk));
    }

    return q;
}



// Rotates the given point through the given angle (in radians) about the given
// axis.
CGVector3D Rotate(float _angle, const CGVector3D& _axis, const CGVector3D& _point)
{
    Quaternion q(cos(_angle/2), _axis * sin(_angle/2));
    CGVector3D result;
    q.ApplyRotation(&result, _point);

    return result;
}



bool Geodetic2D::EqualsEpsilon(Geodetic2D& _other, double _epsilon)
{
    return ( (std::abs(longitude - _other.longitude) <= _epsilon) && 
        (std::abs(latitude - _other.latitude) <= _epsilon) );
}




////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
// Eclipse implementation
Ellipse::Ellipse(const float _x, const float _y, const float _z)
{
    // Ctor
    radii[RadiusType::R] = new CGVector3D(_x, _y, _z);
    radii[RadiusType::R_SQUARED] = new CGVector3D(
        _x * _x, 
        _y * _y, 
        _z * _z);
    radii[RadiusType::R_FOURTH] = new CGVector3D(
        radii[RadiusType::R_SQUARED]->GetX() * radii[RadiusType::R_SQUARED]->GetX(), 
        radii[RadiusType::R_SQUARED]->GetY() * radii[RadiusType::R_SQUARED]->GetY(), 
        radii[RadiusType::R_SQUARED]->GetZ() * radii[RadiusType::R_SQUARED]->GetZ()
        );
    radii[RadiusType::ONE_OVER_R_SQUARED] = new CGVector3D(
        1.0f / radii[RadiusType::R_SQUARED]->GetX(), 
        1.0f / radii[RadiusType::R_SQUARED]->GetX(), 
        1.0f / radii[RadiusType::R_SQUARED]->GetX()
        );

}



// Compute a curve (path) on an ellipse given start and stop points and a granularity
// which is an angle between two adjecent points (appoximations)
CGVector3D**    Ellipse::ComputeCurve(CGVector3D& _start, CGVector3D& _stop, float _granularity, unsigned int& _n_out)
{
    // there MUST ecist a granularity
    assert(_granularity > 0.0f);

    // method to compute a list of durves
    CGVector3D normal; 
    normal = CGVector3D::Normalize(_start.Cross(_stop));
    // calsulate angle between _start and _stop
    float theta = _start.AngleBetween(_stop);

    int n = std::fmaxf((int)(theta / _granularity) - 1.0f, 0.0f);
    int listSize = 2 + n;
    CGVector3D** positions = new CGVector3D*[listSize];
    positions[0] = new CGVector3D(_start.GetX(), _start.GetY(), _start.GetZ());

    for (int i = 1; i < (listSize - 1); i++)
    {
        float phi = (i * _granularity);
        CGVector3D rotation = _start.RotateAroundAxis(normal, phi);
        CGVector3D scaledCGVector3D = ScaleToGeocentricSurface( rotation );
        positions[i] = new CGVector3D(scaledCGVector3D);
    }

    positions[listSize-1] = new CGVector3D(_stop.GetX(), _stop.GetY(), _stop.GetZ());
    return positions;
}



CGVector3D      Ellipse::GeodeticSurfNormal(CGVector3D& _v)
{
    // Get a geodesic surface normal
    return CGVector3D::Normalize(_v.MultiplyComponents(*(radii[RadiusType::ONE_OVER_R_SQUARED])));
}



CGVector3D      Ellipse::GeodeticSurfNormal(Geodetic3D& _geodetic)
{
    // Like the previous method but takes in as parameter a geodetic3D
    float cosLatitude = std::cos(_geodetic.Latitude());

    return CGVector3D(
        cosLatitude * std::cos(_geodetic.Longitude()),
        cosLatitude * std::sin(_geodetic.Longitude()),
        std::sin(_geodetic.Latitude())
    );
}



// Solve an ellipse intersection equation ax^2 + bx + c = 0
// Returns CGVector3D. Third component indicates the number of solutions
CGVector3D      Ellipse::Intersections(CGVector3D& _origin, CGVector3D& _direction)
{
    _direction.NormalizeThis();

    float a = _direction.GetX() * _direction.GetX() * radii[RadiusType::ONE_OVER_R_SQUARED]->GetX() +
        _direction.GetY() * _direction.GetY() * radii[RadiusType::ONE_OVER_R_SQUARED]->GetY() +
        _direction.GetZ() * _direction.GetZ() * radii[RadiusType::ONE_OVER_R_SQUARED]->GetZ();

    float b = 2.0f * 
        (
            _origin.GetX() * _direction.GetX() * radii[RadiusType::ONE_OVER_R_SQUARED]->GetX() +
            _origin.GetY() * _direction.GetY() * radii[RadiusType::ONE_OVER_R_SQUARED]->GetY() +
            _origin.GetZ() * _direction.GetZ() * radii[RadiusType::ONE_OVER_R_SQUARED]->GetZ()
        );

    float c = _origin.GetX() * _origin.GetX() * radii[RadiusType::ONE_OVER_R_SQUARED]->GetX() +
        _origin.GetY() * _origin.GetY() * radii[RadiusType::ONE_OVER_R_SQUARED]->GetY() +
        _origin.GetZ() * _origin.GetZ() * radii[RadiusType::ONE_OVER_R_SQUARED]->GetZ() - 1.0f;

    // Now solve ax^2 + bx + c = 0
    float discriminant = b * b - 4 * a * c;

    CGVector3D(0.0f, 0.0f, 0.0f);

    if (discriminant < 0.0f)
    {
        // no intersections
        return CGVector3D(0.0f, 0.0f, 0.0f);
    }
    else if (discriminant == 0.0f)
    {
        // one intersection at a tangent point
        CGVector3D(-0.5f * b / a, 0.0f, 1.0f);
    }
    float t = -0.5 * (b + (b > 0.0f ? 1.0f : -1.0f) * std::sqrt(discriminant));
    float root1 = t / a;
    float root2 = c / t;

    // Two intersections - return the smallest intersection as the first component
    if(root1 < root2)
    {
        return CGVector3D(root1, root2, 2.0f);
    }
    
    return CGVector3D(root2, root1, 2.0f);
}



// return the smallest radius of this ellipse
float       Ellipse::MinimumRadius()
{
    return std::fminf(
        radii[RadiusType::R]->GetX(), 
        std::fminf(
            radii[RadiusType::R]->GetY(), 
            radii[RadiusType::R]->GetZ()));
}



// Return the greatest radius of this ellipse
float       Ellipse::MaximumRadius()
{
    return std::fmaxf(
        radii[RadiusType::R]->GetX(), 
        std::fmaxf(
            radii[RadiusType::R]->GetY(), 
            radii[RadiusType::R]->GetZ()));
}



CGVector3D      Ellipse::ToCGVector3D(Geodetic2D& _geodetic)
{
    Geodetic3D gd((float)_geodetic.Latitude(), (float)_geodetic.Longitude(), 0.0f );
    return ToCGVector3D(gd);
}



// Convertor from Geodetic coordinate system to cartesian coordinates
CGVector3D      Ellipse::ToCGVector3D(Geodetic3D& _geodetic)
{
    CGVector3D n = GeodeticSurfNormal(_geodetic);
    CGVector3D k = radii[RadiusType::R_SQUARED]->MultiplyComponents(n);
    double gamma = std::sqrt(
        (k.GetX() * n.GetX()) +
        (k.GetY() * n.GetY()) +
        (k.GetZ() * n.GetZ())
        );
 
    CGVector3D rSurface = CGVector3D(k / gamma);

    return CGVector3D (rSurface + (n * _geodetic.Height())) ;
}



Geodetic2D  Ellipse::ToGeodetic2D(CGVector3D& _positions)
{
    CGVector3D n = GeodeticSurfNormal(_positions);
    Geodetic2D gd(
                      (float) std::atan2 (n.GetY(), n.GetX()),
                      (float) std::asin(n.GetZ() / n.Magnitude())
                      );
    return gd;
}



Geodetic3D  Ellipse::ToGeodetic3D(CGVector3D& _position)
{
    CGVector3D p = ScaleToGeodeticSurface(_position);
    CGVector3D h = _position - p;
    float height = sgn<float>(h.operator*(_position)) * h.Magnitude(); 
    Geodetic2D gd2d;
    gd2d = ToGeodetic2D(p);
    Geodetic3D gd(gd2d.Latitude(), gd2d.Longitude(), height);
    return gd;
}



CGVector3D      Ellipse::ScaleToGeodeticSurface(CGVector3D _position)
{
    float beta = 1.0f / std::sqrt(
        (_position.GetX() * _position.GetX()) * radii[RadiusType::ONE_OVER_R_SQUARED]->GetX() +
        (_position.GetY() * _position.GetY()) * radii[RadiusType::ONE_OVER_R_SQUARED]->GetY() +
        (_position.GetZ() * _position.GetZ()) * radii[RadiusType::ONE_OVER_R_SQUARED]->GetZ()
    );
    
    float n = CGVector3D(
        beta*_position.GetX()*radii[RadiusType::ONE_OVER_R_SQUARED]->GetX(),
        beta*_position.GetY()*radii[RadiusType::ONE_OVER_R_SQUARED]->GetY(),
        beta*_position.GetZ()*radii[RadiusType::ONE_OVER_R_SQUARED]->GetZ()
    ).Magnitude();
    
    float alpha = (1.0f - beta) * (_position.Magnitude() / n);
    float x2 = _position.GetX();
    float y2 = _position.GetY();
    float z2 = _position.GetZ();
    
    float da = 0.0f;
    float db = 0.0f;
    float dc = 0.0f;
    
    float s = 0.0f;
    float dSdA = 1.0f;
    
    do
    {
        alpha -= (s / dSdA);
        da = 1.0f + (alpha * radii[RadiusType::ONE_OVER_R_SQUARED]->GetX());
        db = 1.0f + (alpha * radii[RadiusType::ONE_OVER_R_SQUARED]->GetY());
        dc = 1.0f + (alpha * radii[RadiusType::ONE_OVER_R_SQUARED]->GetZ());
        
        float da2 = da * da;
        float db2 = db * db;
        float dc2 = dc * dc;
        
        float da3 = da * da2;
        float db3 = db * db2;
        float dc3 = dc * dc2;

        


        
        s = x2 / (radii[RadiusType::R_SQUARED]->GetX() * da2) +
            y2 / (radii[RadiusType::R_SQUARED]->GetY() * db2) +
            z2 / (radii[RadiusType::R_SQUARED]->GetZ() * dc2) - 1.0f;
        
        dSdA = -2.0 * (
            x2 / (radii[RadiusType::R_FOURTH]->GetX() * da3) +
            y2 / (radii[RadiusType::R_FOURTH]->GetY() * db3) +
            z2 / (radii[RadiusType::R_FOURTH]->GetZ() * dc3)
        );
        
    }
    while (std::abs(s) > 1e-10);

    return CGVector3D( _position.GetX() / da, _position.GetY() / db, _position.GetZ() / dc );
    
}



CGVector3D      Ellipse::ScaleToGeocentricSurface(CGVector3D& _position)
{
    float beta = 1.0f / std::sqrt(
        (_position.GetX() * _position.GetX()) * radii[RadiusType::ONE_OVER_R_SQUARED]->GetX() +
        (_position.GetY() * _position.GetY()) * radii[RadiusType::ONE_OVER_R_SQUARED]->GetY() +
        (_position.GetZ() * _position.GetZ()) * radii[RadiusType::ONE_OVER_R_SQUARED]->GetZ() );
    
    return _position * beta;
}