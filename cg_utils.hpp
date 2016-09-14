#ifndef CG_UTILS_HPP
#define CG_UTILS_HPP
// #include <cmath>

/** 
 *   Conventions:
 *** Class names always begin with CG. So class name example: CGVector
 *** Member functions start with capital letters
 *** Parameters of fucntions are prefixed wiht an underscore. Example int _a is a parameter.
 *** Long variable names with camel notation
*/


namespace CGutils {

    class Vector
    // Vector class for 3D stuff.
    {
    public:
        Vector() {}
        Vector(float _x, float _y, float _z) { x[0] = _x; x[1] = _y; x[2] = _z; }
        Vector(const Vector& _v) { x[0] = _v.x[0]; x[1] = _v.x[1]; x[2] = _v.x[2]; }

        operator    const float*() const { return &x[0]; }
                
        float   Get(int _element) const { return x[_element]; }
        void    Set(int _element, float _newValue) { x[_element] = _newValue; }
        float   GetX() const { return x[0]; }
        float   GetY() const { return x[1]; }
        float   GetZ() const { return x[2]; }
        void    SetX(float _newx) { x[0] = _newx; }
        void    SetY(float _newy) { x[1] = _newy; }
        void    SetZ(float _newz) { x[2] = _newz; }
        void    SetXYZ(float _newx, float _newy, float _newz) { x[0] = _newx; x[1] = _newy; x[2] = _newz; }
        
        Vector  operator+(const Vector& _v) const;
        Vector  operator-(const Vector& _v) const;
        Vector  operator-() const;
        float   operator*(const Vector& _v) const;
        Vector  operator*(float _f) const;
        Vector  operator/(float _f) const { return this->operator*(1.0 / _f); }
        Vector  Cross(const Vector& _v) const;

        Vector MultiplyComponents(Vector& _v);
        Vector& Normalize();

        Vector& operator=(const Vector& _v) { x[0] = _v.x[0]; x[1] = _v.x[1]; x[2] = _v.x[2]; return *this; }
        Vector& operator+=(const Vector& _v);
        Vector& operator-=(const Vector& _v);
        Vector& operator*=(float _f);
        Vector& operator/=(float _f) { return this->operator*=(1.0 / _f); }

        float   Magnitude() const;
        float   Sqrmag() const;
    //  float   min() const;
    //  float   max() const;
    //  float   minabs() const;
    //  float   maxabs() const;

        bool    CheckNaN() const;   
    private:
        float   x[3];
    };

// Make sure these are defined only once
#define INLINE_VECTOR
#ifdef INLINE_VECTOR

    // Vector dot product.
    inline float    Vector::operator*(const Vector& _v) const
    {
        float   result;
        result = x[0] * _v.x[0];
        result += x[1] * _v.x[1];
        result += x[2] * _v.x[2];
        return result;
    }


    // Vector addition operator.
    inline Vector&  Vector::operator+=(const Vector& _v)
    {
        x[0] += _v.x[0];
        x[1] += _v.x[1];
        x[2] += _v.x[2];
        return *this;
    }


    // Vector sunbtraction operator.
    inline Vector&  Vector::operator-=(const Vector& _v)
    {
        x[0] -= _v.x[0];
        x[1] -= _v.x[1];
        x[2] -= _v.x[2];
        return *this;
    }

#endif // INLINE_VECTOR
}

    extern CGutils::Vector ZeroVector;   // (0, 0, 0)
    extern CGutils::Vector UnitVector;   // (1, 1, 1)
    extern CGutils::Vector XAxis;        // (1, 0, 0)
    extern CGutils::Vector YAxis;        // (0, 1, 0)
    extern CGutils::Vector ZAxis;        // (0, 0, 1)

namespace CGutils {

    // Quaternion to represent rotations
    class Quaternion {
    public:
        // Default constriction 
        Quaternion() : scalar(1), vector(ZeroVector) {}
        // Copy constructor
        Quaternion(const Quaternion& _q) : scalar(_q.scalar), vector(_q.vector) {}
        // Implicit definition of members
        Quaternion(float _s, const Vector& _v) : scalar(_s), vector(_v) {}
        // Construct a rotation
        Quaternion(const Vector& _axis, float _angle); 

        float   GetS() const { return scalar; } // get the scalar component
        const Vector&   GetV() const { return vector; } // get the vector component - improve
        void    SetS(float _s) { scalar = _s; } //set scalar component
        void    SetV(const Vector& _v) { vector = _v; } // set vector component

        float   Get(int _i) const { if (_i==0) return GetS(); else return vector.Get(_i-1); }
        void    Set(int _i, float _f) { if (_i==0) scalar = _f; else vector.Set(_i-1, _f); }

        Quaternion  operator*(const Quaternion& _q) const;
        Quaternion& operator*=(float _f) { scalar *= _f; vector *= _f; return *this; }
        Quaternion& operator+=(const Quaternion& _q) { scalar += _q.scalar; vector += _q.vector; return *this; }
        Quaternion& operator=(const Quaternion& _q) { scalar = _q.scalar; vector = _q.vector; return *this; }
        Quaternion& Normalize();
        Quaternion& operator*=(const Quaternion& q);
        void    ApplyRotation(Vector* _result, const Vector& _v);
        
        Quaternion  Lerp(const Quaternion& _q, float _f) const;
    private:
        float   scalar;
        Vector  vector;
    };




    // Matrix class for 3D tranformations
    class Matrix
    {
    public:
        Matrix() { Identity(); }

        void        Identity();
        void        View(const Vector& _vViewNormal, const Vector& _vViewUp, const Vector& _vViewLocation);
        void        Orient(const Vector& _vObjectDirection, const Vector& _vObjectUp, const Vector& _vObjectLocation);
        static void Compose(Matrix* _dest, const Matrix& _left, const Matrix& _right);
        
        Vector      operator*(const Vector& _v) const;
        Matrix      operator*(const Matrix& _m) const;
        Matrix&     operator*=(float _f);
        Matrix&     operator+=(const Matrix& _m);
        
        void        Invert();
        void        InvertRotation();
        void        NormalizeRotation();
        void        Apply(Vector* _result, const Vector& _v) const;
        void        ApplyRotation(Vector* _result, const Vector& _v) const;
        void        ApplyInverse(Vector* _result, const Vector& _v) const;
        void        ApplyInverseRotation(Vector* _result, const Vector& _v) const;
        void        Translate(const Vector& _v);
        void        SetOrientation(const Quaternion& _q);
        Quaternion  GetOrientation() const;
        
        void        SetColumn(int _column, const Vector& _v) { m[_column] = _v; }
        const Vector&   GetColumn(int _column) const { return m[_column]; }
    private:
        Vector  m[4];
    };




    Vector Rotate(float _angle, const Vector& _axis, const Vector& _point);
    
    
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }




    // Geodetic2D class for representing geo coorinates on a sphere
    class Geodetic2D
    {
    public:
        Geodetic2D() : latitude(0.0f), longitude(0.0f) {}
        Geodetic2D(float _lat, float _lng) : latitude(_lat), longitude(_lng) {}
        Geodetic2D(Geodetic2D& _c) : latitude(_c.Latitude()), longitude(_c.Longitude()) {}

        Geodetic2D& operator=(const Geodetic2D& _g) { latitude = _g.Latitude(); longitude = _g.Longitude(); return *this; }

        float   Latitude() const{ return latitude; } 
        float   Longitude() const { return longitude; } 
        bool EqualsEpsilon(Geodetic2D& _other, double _epsilon);

        bool Equals(const Geodetic2D& _other) { return operator==(_other); }
        inline bool operator==(const Geodetic2D& _c) const {
            return ( (_c.latitude==latitude) && (_c.longitude==longitude) );
        }
        inline bool operator!=(const Geodetic2D& _other) const { return !operator==(_other); }

    private:
        float latitude;
        float longitude;
    };




    // Geodetic3D class for representing geo coorinates on a sphere
    class Geodetic3D
    {
    public:
        Geodetic3D() : latitude(0.0f), longitude(0.0f), height(0.0f) {}
        Geodetic3D(float _lat, float _lng, float _h) : latitude(_lat), longitude(_lng), height(_h) {}
        Geodetic3D(Geodetic3D& _c) : latitude(_c.latitude), longitude(_c.longitude), height(_c.height) {}
        Geodetic3D(Geodetic2D& _c) : latitude(_c.Latitude()), longitude(_c.Longitude()), height(0.0f) {}

        Geodetic3D& operator=(const Geodetic3D& _g) { 
            latitude = _g.Latitude(); 
            longitude = _g.Longitude(); 
            height = _g.Height();
            return *this; }


        float   Latitude() const { return latitude; } 
        float   Longitude() const { return longitude; } 
        float   Height() const { return height; } 

        bool Equals(const Geodetic3D& _other) { return operator==(_other); }
        inline bool operator==(const Geodetic3D& _c) const {
            return ( (_c.latitude==latitude) && (_c.longitude==longitude) && (_c.height==height) );
        }
        inline bool operator!=(const Geodetic3D& _other) const { return !operator==(_other); }

    private:
        float latitude;
        float longitude;
        float height;
    };



    // An anumerator for types of radii for ellipses
    enum RadiusType { 
        R = 0, 
        R_SQUARED, 
        R_FOURTH, 
        ONE_OVER_R_SQUARED };



    // Ellipse class for representing a mathematical ellipse
    class Ellipse
    {
    public:
        Ellipse() { Ellipse( 1.0f, 1.0f, 1.0f ); } 
        Ellipse(const Vector& _v) { Ellipse(_v.GetX(), _v.GetY(), _v.GetZ()); }
        Ellipse(const float _x, const float _y, const float _z);


        // Functions to get the radii 
        const float     Get(int _r) { return radii[0]->Get(_r); } 
        const float     GetX() {return radii[0]->GetX(); }
        const float     GetY() {return radii[0]->GetY(); }
        const float     GetZ() {return radii[0]->GetZ(); }

        static Vector   GeocentricSurfNormal(Vector& _vertexPosition) { return Vector( _vertexPosition.Normalize() ); }
        Vector          GeodeticSurfNormal(Vector& _vertexPosition);
        Vector          GeodeticSurfNormal(Geodetic3D& _geodetic);
        const Vector*   GetR(const RadiusType _rt) { return radii[_rt]; } // get a radius component

        // Solve an ellipse intersection equation ax^2 + bx + c = 0
        // Returns vector. Third component indicates the number of solutions
        Vector          Intersections(Vector& _origin, Vector& _direction);
        
        float           MinimumRadius();
        float           MaximumRadius();

        // Members to convert Geodetic coordinates to vectors (cartesian coordinates) and vice versa
        Vector          ToVector(Geodetic2D& _geodetic);
        Vector          ToVector(Geodetic3D& _geodetic);
        Geodetic2D      ToGeodetic2D(Vector& _position);
        Geodetic3D      ToGeodetic3D(Vector& _position);
        Vector          ScaleToGeodeticSurface(Vector _position);
        Vector          ScaleToGeocentricSurface(Vector _position);

/*
        float[]         Intersections(Vector origin, Vector direction); 
*/
    private:
        Vector *radii[4];
    };


}

#endif