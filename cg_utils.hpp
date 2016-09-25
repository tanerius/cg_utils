#ifndef CG_UTILS_HPP
#define CG_UTILS_HPP
// #include <cmath>

/** 
 *   Conventions:
 *** Class names always begin with CG. So class name example: CGCGVector3D
 *** Member functions start with capital letters
 *** Parameters of fucntions are prefixed wiht an underscore. Example int _a is a parameter.
 *** Long variable names with camel notation
*/


namespace CGutils {

    class CGVector3D
    // CGVector3D class for 3D stuff.
    {
    public:
        CGVector3D() {}
        CGVector3D(const float _x, const float _y, const float _z) { x[0] = _x; x[1] = _y; x[2] = _z; }
        CGVector3D(const CGVector3D& _v) { x[0] = _v.x[0]; x[1] = _v.x[1]; x[2] = _v.x[2]; }

        bool            CheckNaN() const;   

        float           AngleBetween(const CGVector3D& _v); //get an angle between two CGVector3Ds
        float           Get(int _element) const { return x[_element]; }
        float           GetX() const { return x[0]; }
        float           GetY() const { return x[1]; }
        float           GetZ() const { return x[2]; }
        float           Magnitude() const;
        float           operator*(const CGVector3D& _v) const; // inline 
        float           Sqrmag() const;

        // When a CGVector3D is assigned to a float what happens (implicit conversion) ?
        operator        const float*() const { return &x[0]; }

        static CGVector3D   Normalize(const CGVector3D _v); // inline
                
        void            NormalizeThis(); // Normalize this CGVector3D (modify it)
        void            Set(int _element, float _newValue) { x[_element] = _newValue; }   
        void            SetX(float _newx) { x[0] = _newx; }
        void            SetY(float _newy) { x[1] = _newy; }
        void            SetZ(float _newz) { x[2] = _newz; }
        void            SetXYZ(const float _newx, const float _newy, const float _newz); // inline 
        
        CGVector3D          Cross(const CGVector3D& _v) const;
        CGVector3D          MultiplyComponents(const CGVector3D& _v);
        CGVector3D          operator+(const CGVector3D& _v) const;
        CGVector3D          operator-(const CGVector3D& _v) const;
        CGVector3D          operator-() const;
        CGVector3D          operator*(float _f) const;
        CGVector3D          operator/(float _f) const { return this->operator*(1.0 / _f); }
        CGVector3D          RotateAroundAxis(const CGVector3D& _v, const float _theta);
        
        CGVector3D&         operator=(const CGVector3D& _v) { x[0] = _v.x[0]; x[1] = _v.x[1]; x[2] = _v.x[2]; return *this; }
        CGVector3D&         operator+=(const CGVector3D& _v); // inline
        CGVector3D&         operator-=(const CGVector3D& _v); // inline 
        CGVector3D&         operator*=(float _f);
        CGVector3D&         operator/=(float _f) { return this->operator*=(1.0 / _f); }
        
    private:
        float           x[3];
    };

// Make sure these are defined only once
#define INLINE_CGVector3D
#ifdef INLINE_CGVector3D

    // CGVector3D dot product.
    inline float    CGVector3D::operator*(const CGVector3D& _v) const
    {
        float   result;
        result = x[0] * _v.x[0];
        result += x[1] * _v.x[1];
        result += x[2] * _v.x[2];
        return result;
    }


    // CGVector3D addition operator.
    inline CGVector3D&  CGVector3D::operator+=(const CGVector3D& _v)
    {
        x[0] += _v.x[0];
        x[1] += _v.x[1];
        x[2] += _v.x[2];
        return *this;
    }


    // CGVector3D sunbtraction operator.
    inline CGVector3D&  CGVector3D::operator-=(const CGVector3D& _v)
    {
        x[0] -= _v.x[0];
        x[1] -= _v.x[1];
        x[2] -= _v.x[2];
        return *this;
    }


    // Static function to return a normalized CGVector3D
    inline CGVector3D CGVector3D::Normalize(const CGVector3D _v)
    {
        CGVector3D normalized = _v;
        normalized.NormalizeThis();
        return normalized;
    }


    inline void CGVector3D::SetXYZ(const float _newx, const float _newy, const float _newz)
    { 
        x[0] = _newx; 
        x[1] = _newy; 
        x[2] = _newz; 
    }

#endif // INLINE_CGVector3D
}

    extern CGutils::CGVector3D ZeroVector3D;   // (0, 0, 0)
    extern CGutils::CGVector3D UnitVector3D;   // (1, 1, 1)
    extern CGutils::CGVector3D XAxis;        // (1, 0, 0)
    extern CGutils::CGVector3D YAxis;        // (0, 1, 0)
    extern CGutils::CGVector3D ZAxis;        // (0, 0, 1)

namespace CGutils {

    // Quaternion to represent rotations
    class Quaternion {
    public:
        // Default constriction 
        Quaternion() : scalar(1), v(ZeroVector3D) {}
        // Copy constructor
        Quaternion(const Quaternion& _q) : scalar(_q.scalar), v(_q.v) {}
        // Implicit definition of members
        Quaternion(float _s, const CGVector3D& _v) : scalar(_s), v(_v) {}
        // Construct a rotation
        Quaternion(const CGVector3D& _axis, float _angle); 

        float   GetS() const { return scalar; } // get the scalar component
        const CGVector3D&   GetV() const { return v; } // get the CGVector3D component - improve
        void    SetS(float _s) { scalar = _s; } //set scalar component
        void    SetV(const CGVector3D& _v) { v = _v; } // set CGVector3D component

        float   Get(int _i) const { if (_i==0) return GetS(); else return v.Get(_i-1); }
        void    Set(int _i, float _f) { if (_i==0) scalar = _f; else v.Set(_i-1, _f); }

        Quaternion  operator*(const Quaternion& _q) const;
        Quaternion& operator*=(float _f) { scalar *= _f; v *= _f; return *this; }
        Quaternion& operator+=(const Quaternion& _q) { scalar += _q.scalar; v += _q.v; return *this; }
        Quaternion& operator=(const Quaternion& _q) { scalar = _q.scalar; v = _q.v; return *this; }
        Quaternion& Normalize();
        Quaternion& operator*=(const Quaternion& q);
        void    ApplyRotation(CGVector3D* _result, const CGVector3D& _v);
        
        Quaternion  Lerp(const Quaternion& _q, float _f) const;
    private:
        float   scalar;
        CGVector3D  v;
    };




    // Matrix class for 3D tranformations
    class Matrix
    {
    public:
        Matrix() { Identity(); }

        void        Identity();
        void        View(const CGVector3D& _vViewNormal, const CGVector3D& _vViewUp, const CGVector3D& _vViewLocation);
        void        Orient(const CGVector3D& _vObjectDirection, const CGVector3D& _vObjectUp, const CGVector3D& _vObjectLocation);
        static void Compose(Matrix* _dest, const Matrix& _left, const Matrix& _right);
        
        CGVector3D      operator*(const CGVector3D& _v) const;
        Matrix      operator*(const Matrix& _m) const;
        Matrix&     operator*=(float _f);
        Matrix&     operator+=(const Matrix& _m);
        
        void        Invert();
        void        InvertRotation();
        void        NormalizeRotation();
        void        Apply(CGVector3D* _result, const CGVector3D& _v) const;
        void        ApplyRotation(CGVector3D* _result, const CGVector3D& _v) const;
        void        ApplyInverse(CGVector3D* _result, const CGVector3D& _v) const;
        void        ApplyInverseRotation(CGVector3D* _result, const CGVector3D& _v) const;
        void        Translate(const CGVector3D& _v);
        void        SetOrientation(const Quaternion& _q);
        Quaternion  GetOrientation() const;
        
        void        SetColumn(int _column, const CGVector3D& _v) { m[_column] = _v; }
        const CGVector3D&   GetColumn(int _column) const { return m[_column]; }
    private:
        CGVector3D  m[4];
    };


    CGVector3D Rotate(float _angle, const CGVector3D& _axis, const CGVector3D& _point);
    
    
    template <typename T> int sgn(T val) 
    {
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

        float           Latitude() const{ return latitude; } 
        float           Longitude() const { return longitude; } 
        bool            EqualsEpsilon(Geodetic2D& _other, double _epsilon);

        bool            Equals(const Geodetic2D& _other) { return operator==(_other); }
        inline bool     operator==(const Geodetic2D& _c) const 
        {
            return ( (_c.latitude==latitude) && (_c.longitude==longitude) );
        }
        inline bool     operator!=(const Geodetic2D& _other) const { return !operator==(_other); }

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
        inline bool operator==(const Geodetic3D& _c) const 
        {
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
        Ellipse(const CGVector3D& _v) { Ellipse(_v.GetX(), _v.GetY(), _v.GetZ()); }
        Ellipse(const float _x, const float _y, const float _z);

        CGVector3D**        ComputeCurve(CGVector3D& _start, CGVector3D& _stop, float _granularity, unsigned int& _n_out);

        // Functions to get the radii 
        const float     Get(int _r) { return radii[0]->Get(_r); } 
        const float     GetX() {return radii[0]->GetX(); }
        const float     GetY() {return radii[0]->GetY(); }
        const float     GetZ() {return radii[0]->GetZ(); }

        static CGVector3D   GeocentricSurfNormal(CGVector3D& _vertexPosition) { return CGVector3D::Normalize(_vertexPosition); }
        CGVector3D          GeodeticSurfNormal(CGVector3D& _vertexPosition);
        CGVector3D          GeodeticSurfNormal(Geodetic3D& _geodetic);
        const CGVector3D*   GetR(const RadiusType _rt) { return radii[_rt]; } // get a radius component

        // Solve an ellipse intersection equation ax^2 + bx + c = 0
        // Returns CGVector3D. Third component indicates the number of solutions
        CGVector3D          Intersections(CGVector3D& _origin, CGVector3D& _direction);

        float           MinimumRadius();
        float           MaximumRadius();

        // Members to convert Geodetic coordinates to CGVector3Ds (cartesian coordinates) and vice versa
        CGVector3D          ToCGVector3D(Geodetic2D& _geodetic);
        CGVector3D          ToCGVector3D(Geodetic3D& _geodetic);
        Geodetic2D          ToGeodetic2D(CGVector3D& _position);
        Geodetic3D          ToGeodetic3D(CGVector3D& _position);
        CGVector3D          ScaleToGeodeticSurface(CGVector3D _position);
        CGVector3D          ScaleToGeocentricSurface(CGVector3D& _position);


    private:
        CGVector3D*         radii[4];
    };


}

    

#endif