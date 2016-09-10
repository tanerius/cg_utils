#ifndef CG_UTILS_HPP
#define CG_UTILS_HPP

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


}

#endif