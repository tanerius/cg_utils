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
        Vector  cross(const Vector& _v) const;

        Vector& normalize();
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

    extern Vector ZeroVector;   // (0, 0, 0)
    extern Vector XAxis;        // (1, 0, 0)
    extern Vector YAxis;        // (0, 1, 0)
    extern Vector ZAxis;        // (0, 0, 1)
}

#endif