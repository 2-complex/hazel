
/*
  g2c Copyright (C) 2012 2-Complex

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/


#ifndef _QUATERNION_
#define _QUATERNION_

#include "lin.h"

class Quaternion;
class Quaternion {
public:
    // xi + yj + zk + w
    double x, y, z, w;
    Quaternion() : x(0.0), y(0.0), z(0.0), w(1.0) {}
    Quaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
    Quaternion(const Vec3& xyz, double w) : x(xyz.x), y(xyz.y), z(xyz.z), w(w) {}
    Quaternion(const Vec3& xyz) : x(xyz.x), y(xyz.y), z(xyz.z), w(0.0) {}
    Quaternion(const Vec4& xyzw) : x(xyzw.x), y(xyzw.y), z(xyzw.z), w(xyzw.w) {}
    Quaternion(const Quaternion& xyzw) : x(xyzw.x), y(xyzw.y), z(xyzw.z), w(xyzw.w) {}
    Quaternion(const Mat3& M)
    {
        w = 0.5*sqrt(1+M.m00+M.m11+M.m22);
        double k = 0.25/w;
        x = k*(M.m21 - M.m12);
        y = k*(M.m02 - M.m20);
        z = k*(M.m10 - M.m01);
    }
    
    inline Quaternion& operator= (const Quaternion& v) {x=v.x; y=v.y; z=v.z; w=v.w; return *this;}
    inline bool operator== (const Quaternion& v) const {return x==v.x && y==v.y && z==v.z && w==v.w;}
    inline bool operator!= (const Quaternion& v) const {return x!=v.x || y!=v.y || z!=v.z || w!=v.w;}
    inline void set(double in_x, double in_y, double in_z, double in_w) {x=in_x; y=in_y; z=in_z; w=in_w;}
    inline void set(const double* v) {x=v[0]; y=v[1]; z=v[2]; w=v[3];}
    inline Quaternion operator- () const {return Quaternion(-x, -y, -z, -w);}
    inline Quaternion operator+ (const Quaternion& v) const {return Quaternion(x+v.x, y+v.y, z+v.z, w+v.w);}
    inline Quaternion operator- (const Quaternion& v) const {return Quaternion(x-v.x, y-v.y, z-v.z, w-v.w);}
    inline Quaternion operator* (double k) const {return Quaternion(x*k, y*k, z*k, w*k);}
    inline Quaternion operator/ (double k) const {return Quaternion(x/k, y/k, z/k, w/k);}
    inline friend Quaternion operator* (double k, const Quaternion& v) {return Quaternion(k*v.x, k*v.y, k*v.z, k*v.w);}
    inline friend Quaternion operator/ (double k, const Quaternion& v) {return Quaternion(k/v.x, k/v.y, k/v.z, k/v.w);}
    inline Quaternion& operator+= (const Quaternion& v) {x+=v.x; y+=v.y; z+=v.z; w+=v.w; return *this;}
    inline Quaternion& operator-= (const Quaternion& v) {x-=v.x; y-=v.y; z-=v.z; w-=v.w; return *this;}
    inline Quaternion& operator*= (double k) {x*=k; y*=k; z*=k; w*=k; return *this;}
    inline Quaternion& operator/= (double k) {x/=k; y/=k; z/=k; w/=k; return *this;}
    
    double dot(const Quaternion& v) const {return x*v.x + y*v.y + z*v.z + w*v.w;}
    double magSquared() const {return x*x + y*y + z*z + w*w;}
    double mag() const {return sqrt(x*x + y*y + z*z + w*w);}
    
    const double* ptr() const {return &(x);}
    void print() const {printf("(%f, %f, %f, %f)", x, y, z, w);}
    void display() const {print(); printf("\n");}
    
    inline Quaternion conjugate() const {return Quaternion(-x, -y, -z, w);}
    inline Quaternion inverse() const {return conjugate()/magSquared();}
    inline Quaternion operator* (const Quaternion& q) const
    {
        return Quaternion(w*q.x + x*q.w + y*q.z - z*q.y,
                          w*q.y + y*q.w + z*q.x - x*q.z,
                          w*q.z + z*q.w + x*q.y - y*q.x,
                          w*q.w - x*q.x - y*q.y - z*q.z);
    }
    
    inline Quaternion operator/ (const Quaternion& q) const {return (*this)*q.inverse();}
    inline Quaternion& operator*= (const Quaternion& q) {(*this)=(*this)*q; return *this;}
    inline Quaternion& operator/= (const Quaternion& q) {(*this)=(*this)/q; return *this;}
    
    Vec3 vector() const {return Vec3(x, y, z);}
    Mat3 matrix() const {return Mat3(1.0-2.0*(y*y+z*z), 2.0*(x*y+w*z), 2.0*(x*z-w*y),
                                     2.0*(x*y-w*z), 1.0-2.0*(x*x+z*z), 2.0*(w*x+y*z),
                                     2.0*(w*y+x*z), 2.0*(y*z-w*x), 1.0-2.0*(x*x+y*y));}
    
    Vec3 rotate(const Vec3& V) const
    {
        return (inverse()*Quaternion(V)*(*this)).vector();
    }
};

#endif

