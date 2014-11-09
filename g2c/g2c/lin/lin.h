
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



#ifndef _lin_
#define _lin_

#include <stdio.h>
#include <math.h>

class Vec2;
class Vec2 {
public:
    double x, y;
    Vec2() : x(0.0), y(0.0) {}
    Vec2(double x, double y) : x(x), y(y) {}
    Vec2(const Vec2& xy) : x(xy.x), y(xy.y) {}
    
    inline Vec2& operator= (const Vec2& v) {x=v.x; y=v.y; return *this;}
    inline bool operator== (const Vec2& v) const {return x==v.x && y==v.y;}
    inline bool operator!= (const Vec2& v) const {return x!=v.x || y!=v.y;}
    inline void set(double in_x, double in_y) {x=in_x; y=in_y;}
    inline void set(const double* v) {x=v[0]; y=v[1];}
    inline Vec2 operator- () const {return Vec2(-x, -y);}
    inline Vec2 operator+ (const Vec2& v) const {return Vec2(x+v.x, y+v.y);}
    inline Vec2 operator- (const Vec2& v) const {return Vec2(x-v.x, y-v.y);}
    inline Vec2 operator* (double k) const {return Vec2(x*k, y*k);}
    inline Vec2 operator/ (double k) const {return Vec2(x/k, y/k);}
    inline friend Vec2 operator* (double k, const Vec2& v) {return Vec2(k*v.x, k*v.y);}
    inline friend Vec2 operator/ (double k, const Vec2& v) {return Vec2(k/v.x, k/v.y);}
    inline Vec2& operator+= (const Vec2& v) {x+=v.x; y+=v.y; return *this;}
    inline Vec2& operator-= (const Vec2& v) {x-=v.x; y-=v.y; return *this;}
    inline Vec2& operator*= (double k) {x*=k; y*=k; return *this;}
    inline Vec2& operator/= (double k) {x/=k; y/=k; return *this;}
    
    inline Vec2 operator* (const Vec2& v) const {return Vec2(x*v.x, y*v.y);}
    inline Vec2 operator/ (const Vec2& v) const {return Vec2(x/v.x, y/v.y);}
    inline Vec2& operator*= (const Vec2& v) {x*=v.x; y*=v.y; return *this;}
    inline Vec2& operator/= (const Vec2& v) {x/=v.x; y/=v.y; return *this;}
    
    double dot(const Vec2& v) const {return x*v.x + y*v.y;}
    double magSquared() const {return x*x + y*y;}
    double mag() const {return sqrt(x*x + y*y);}
    
    const double* ptr() const {return &(x);}
    void print() const {printf("(%f, %f)", x, y);}
    void display() const {print(); printf("\n");}
};

class Vec3;
class Vec3 {
public:
    double x, y, z;
    Vec3() : x(0.0), y(0.0), z(0.0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    Vec3(double x, const Vec2& yz) : x(x), y(yz.x), z(yz.y) {}
    Vec3(const Vec2& xy, double z) : x(xy.x), y(xy.y), z(z) {}
    Vec3(const Vec3& xyz) : x(xyz.x), y(xyz.y), z(xyz.z) {}
    
    inline Vec3& operator= (const Vec3& v) {x=v.x; y=v.y; z=v.z; return *this;}
    inline bool operator== (const Vec3& v) const {return x==v.x && y==v.y && z==v.z;}
    inline bool operator!= (const Vec3& v) const {return x!=v.x || y!=v.y || z!=v.z;}
    inline void set(double in_x, double in_y, double in_z) {x=in_x; y=in_y; z=in_z;}
    inline void set(const double* v) {x=v[0]; y=v[1]; z=v[2];}
    inline Vec3 operator- () const {return Vec3(-x, -y, -z);}
    inline Vec3 operator+ (const Vec3& v) const {return Vec3(x+v.x, y+v.y, z+v.z);}
    inline Vec3 operator- (const Vec3& v) const {return Vec3(x-v.x, y-v.y, z-v.z);}
    inline Vec3 operator* (double k) const {return Vec3(x*k, y*k, z*k);}
    inline Vec3 operator/ (double k) const {return Vec3(x/k, y/k, z/k);}
    inline friend Vec3 operator* (double k, const Vec3& v) {return Vec3(k*v.x, k*v.y, k*v.z);}
    inline friend Vec3 operator/ (double k, const Vec3& v) {return Vec3(k/v.x, k/v.y, k/v.z);}
    inline Vec3& operator+= (const Vec3& v) {x+=v.x; y+=v.y; z+=v.z; return *this;}
    inline Vec3& operator-= (const Vec3& v) {x-=v.x; y-=v.y; z-=v.z; return *this;}
    inline Vec3& operator*= (double k) {x*=k; y*=k; z*=k; return *this;}
    inline Vec3& operator/= (double k) {x/=k; y/=k; z/=k; return *this;}
    
    inline Vec3 operator* (const Vec3& v) const {return Vec3(x*v.x, y*v.y, z*v.z);}
    inline Vec3 operator/ (const Vec3& v) const {return Vec3(x/v.x, y/v.y, z/v.z);}
    inline Vec3& operator*= (const Vec3& v) {x*=v.x; y*=v.y; z*=v.z; return *this;}
    inline Vec3& operator/= (const Vec3& v) {x/=v.x; y/=v.y; z/=v.z; return *this;}
    
    double dot(const Vec3& v) const {return x*v.x + y*v.y + z*v.z;}
    double magSquared() const {return x*x + y*y + z*z;}
    double mag() const {return sqrt(x*x + y*y + z*z);}
    Vec3 cross(const Vec3& v) const {return Vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);}
    
    const double* ptr() const {return &(x);}
    void print() const {printf("(%f, %f, %f)", x, y, z);}
    void display() const {print(); printf("\n");}
};

class Vec4;
class Vec4 {
public:
    double x, y, z, w;
    Vec4() : x(0.0), y(0.0), z(0.0), w(0.0) {}
    Vec4(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
    Vec4(double x, double y, const Vec2& zw) : x(x), y(y), z(zw.x), w(zw.y) {}
    Vec4(double x, const Vec2& yz, double w) : x(x), y(yz.x), z(yz.y), w(w) {}
    Vec4(double x, const Vec3& yzw) : x(x), y(yzw.x), z(yzw.y), w(yzw.z) {}
    Vec4(const Vec2& xy, double z, double w) : x(xy.x), y(xy.y), z(z), w(w) {}
    Vec4(const Vec2& xy, const Vec2& zw) : x(xy.x), y(xy.y), z(zw.x), w(zw.y) {}
    Vec4(const Vec3& xyz, double w) : x(xyz.x), y(xyz.y), z(xyz.z), w(w) {}
    Vec4(const Vec4& xyzw) : x(xyzw.x), y(xyzw.y), z(xyzw.z), w(xyzw.w) {}
    
    inline Vec4& operator= (const Vec4& v) {x=v.x; y=v.y; z=v.z; w=v.w; return *this;}
    inline bool operator== (const Vec4& v) const {return x==v.x && y==v.y && z==v.z && w==v.w;}
    inline bool operator!= (const Vec4& v) const {return x!=v.x || y!=v.y || z!=v.z || w!=v.w;}
    inline void set(double in_x, double in_y, double in_z, double in_w) {x=in_x; y=in_y; z=in_z; w=in_w;}
    inline void set(const double* v) {x=v[0]; y=v[1]; z=v[2]; w=v[3];}
    inline Vec4 operator- () const {return Vec4(-x, -y, -z, -w);}
    inline Vec4 operator+ (const Vec4& v) const {return Vec4(x+v.x, y+v.y, z+v.z, w+v.w);}
    inline Vec4 operator- (const Vec4& v) const {return Vec4(x-v.x, y-v.y, z-v.z, w-v.w);}
    inline Vec4 operator* (double k) const {return Vec4(x*k, y*k, z*k, w*k);}
    inline Vec4 operator/ (double k) const {return Vec4(x/k, y/k, z/k, w/k);}
    inline friend Vec4 operator* (double k, const Vec4& v) {return Vec4(k*v.x, k*v.y, k*v.z, k*v.w);}
    inline friend Vec4 operator/ (double k, const Vec4& v) {return Vec4(k/v.x, k/v.y, k/v.z, k/v.w);}
    inline Vec4& operator+= (const Vec4& v) {x+=v.x; y+=v.y; z+=v.z; w+=v.w; return *this;}
    inline Vec4& operator-= (const Vec4& v) {x-=v.x; y-=v.y; z-=v.z; w-=v.w; return *this;}
    inline Vec4& operator*= (double k) {x*=k; y*=k; z*=k; w*=k; return *this;}
    inline Vec4& operator/= (double k) {x/=k; y/=k; z/=k; w/=k; return *this;}
    
    inline Vec4 operator* (const Vec4& v) const {return Vec4(x*v.x, y*v.y, z*v.z, w*v.w);}
    inline Vec4 operator/ (const Vec4& v) const {return Vec4(x/v.x, y/v.y, z/v.z, w/v.w);}
    inline Vec4& operator*= (const Vec4& v) {x*=v.x; y*=v.y; z*=v.z; w*=v.w; return *this;}
    inline Vec4& operator/= (const Vec4& v) {x/=v.x; y/=v.y; z/=v.z; w/=v.w; return *this;}
    
    double dot(const Vec4& v) const {return x*v.x + y*v.y + z*v.z + w*v.w;}
    double magSquared() const {return x*x + y*y + z*z + w*w;}
    double mag() const {return sqrt(x*x + y*y + z*z + w*w);}
    
    const double* ptr() const {return &(x);}
    void print() const {printf("(%f, %f, %f, %f)", x, y, z, w);}
    void display() const {print(); printf("\n");}
};

class Vec5;
class Vec5 {
public:
    double x, y, z, w, t;
    Vec5() : x(0.0), y(0.0), z(0.0), w(0.0), t(0.0) {}
    Vec5(double x, double y, double z, double w, double t) : x(x), y(y), z(z), w(w), t(t) {}
    Vec5(double x, double y, double z, const Vec2& wt) : x(x), y(y), z(z), w(wt.x), t(wt.y) {}
    Vec5(double x, double y, const Vec2& zw, double t) : x(x), y(y), z(zw.x), w(zw.y), t(t) {}
    Vec5(double x, double y, const Vec3& zwt) : x(x), y(y), z(zwt.x), w(zwt.y), t(zwt.z) {}
    Vec5(double x, const Vec2& yz, double w, double t) : x(x), y(yz.x), z(yz.y), w(w), t(t) {}
    Vec5(double x, const Vec2& yz, const Vec2& wt) : x(x), y(yz.x), z(yz.y), w(wt.x), t(wt.y) {}
    Vec5(double x, const Vec3& yzw, double t) : x(x), y(yzw.x), z(yzw.y), w(yzw.z), t(t) {}
    Vec5(double x, const Vec4& yzwt) : x(x), y(yzwt.x), z(yzwt.y), w(yzwt.z), t(yzwt.w) {}
    Vec5(const Vec2& xy, double z, double w, double t) : x(xy.x), y(xy.y), z(z), w(w), t(t) {}
    Vec5(const Vec2& xy, double z, const Vec2& wt) : x(xy.x), y(xy.y), z(z), w(wt.x), t(wt.y) {}
    Vec5(const Vec2& xy, const Vec2& zw, double t) : x(xy.x), y(xy.y), z(zw.x), w(zw.y), t(t) {}
    Vec5(const Vec2& xy, const Vec3& zwt) : x(xy.x), y(xy.y), z(zwt.x), w(zwt.y), t(zwt.z) {}
    Vec5(const Vec3& xyz, double w, double t) : x(xyz.x), y(xyz.y), z(xyz.z), w(w), t(t) {}
    Vec5(const Vec3& xyz, const Vec2& wt) : x(xyz.x), y(xyz.y), z(xyz.z), w(wt.x), t(wt.y) {}
    Vec5(const Vec4& xyzw, double t) : x(xyzw.x), y(xyzw.y), z(xyzw.z), w(xyzw.w), t(t) {}
    Vec5(const Vec5& xyzwt) : x(xyzwt.x), y(xyzwt.y), z(xyzwt.z), w(xyzwt.w), t(xyzwt.t) {}
    
    inline Vec5& operator= (const Vec5& v) {x=v.x; y=v.y; z=v.z; w=v.w; t=v.t; return *this;}
    inline bool operator== (const Vec5& v) const {return x==v.x && y==v.y && z==v.z && w==v.w && t==v.t;}
    inline bool operator!= (const Vec5& v) const {return x!=v.x || y!=v.y || z!=v.z || w!=v.w || t!=v.t;}
    inline void set(double in_x, double in_y, double in_z, double in_w, double in_t) {x=in_x; y=in_y; z=in_z; w=in_w; t=in_t;}
    inline void set(const double* v) {x=v[0]; y=v[1]; z=v[2]; w=v[3]; t=v[4];}
    inline Vec5 operator- () const {return Vec5(-x, -y, -z, -w, -t);}
    inline Vec5 operator+ (const Vec5& v) const {return Vec5(x+v.x, y+v.y, z+v.z, w+v.w, t+v.t);}
    inline Vec5 operator- (const Vec5& v) const {return Vec5(x-v.x, y-v.y, z-v.z, w-v.w, t-v.t);}
    inline Vec5 operator* (double k) const {return Vec5(x*k, y*k, z*k, w*k, t*k);}
    inline Vec5 operator/ (double k) const {return Vec5(x/k, y/k, z/k, w/k, t/k);}
    inline friend Vec5 operator* (double k, const Vec5& v) {return Vec5(k*v.x, k*v.y, k*v.z, k*v.w, k*v.t);}
    inline friend Vec5 operator/ (double k, const Vec5& v) {return Vec5(k/v.x, k/v.y, k/v.z, k/v.w, k/v.t);}
    inline Vec5& operator+= (const Vec5& v) {x+=v.x; y+=v.y; z+=v.z; w+=v.w; t+=v.t; return *this;}
    inline Vec5& operator-= (const Vec5& v) {x-=v.x; y-=v.y; z-=v.z; w-=v.w; t-=v.t; return *this;}
    inline Vec5& operator*= (double k) {x*=k; y*=k; z*=k; w*=k; t*=k; return *this;}
    inline Vec5& operator/= (double k) {x/=k; y/=k; z/=k; w/=k; t/=k; return *this;}
    
    inline Vec5 operator* (const Vec5& v) const {return Vec5(x*v.x, y*v.y, z*v.z, w*v.w, t*v.t);}
    inline Vec5 operator/ (const Vec5& v) const {return Vec5(x/v.x, y/v.y, z/v.z, w/v.w, t/v.t);}
    inline Vec5& operator*= (const Vec5& v) {x*=v.x; y*=v.y; z*=v.z; w*=v.w; t*=v.t; return *this;}
    inline Vec5& operator/= (const Vec5& v) {x/=v.x; y/=v.y; z/=v.z; w/=v.w; t/=v.t; return *this;}
    
    double dot(const Vec5& v) const {return x*v.x + y*v.y + z*v.z + w*v.w + t*v.t;}
    double magSquared() const {return x*x + y*y + z*z + w*w + t*t;}
    double mag() const {return sqrt(x*x + y*y + z*z + w*w + t*t);}
    
    const double* ptr() const {return &(x);}
    void print() const {printf("(%f, %f, %f, %f, %f)", x, y, z, w, t);}
    void display() const {print(); printf("\n");}
};

class Mat2 {
public:
    double m00, m01, m10, m11;
    Mat2() : m00(1.0), m01(0.0), m10(0.0), m11(1.0) {}
    Mat2(double m00, double m01, double m10, double m11) : m00(m00), m01(m01), m10(m10), m11(m11) {}
    Mat2(const Vec2& m0, const Vec2& m1) : m00(m0.x), m01(m0.y), m10(m1.x), m11(m1.y) {}
    
    inline Mat2& operator= (const Mat2& v) {m00=v.m00; m01=v.m01; m10=v.m10; m11=v.m11; return *this;}
    inline bool operator== (const Mat2& v) const {return m00==v.m00 && m01==v.m01 && m10==v.m10 && m11==v.m11;}
    inline bool operator!= (const Mat2& v) const {return m00!=v.m00 || m01!=v.m01 || m10!=v.m10 || m11!=v.m11;}
    inline void set(double in_m00, double in_m01, double in_m10, double in_m11) {m00=in_m00; m01=in_m01; m10=in_m10; m11=in_m11;}
    inline void set(const double* v) {m00=v[0]; m01=v[1]; m10=v[2]; m11=v[3];}
    inline Mat2 operator- () const {return Mat2(-m00, -m01, -m10, -m11);}
    inline Mat2 operator+ (const Mat2& v) const {return Mat2(m00+v.m00, m01+v.m01, m10+v.m10, m11+v.m11);}
    inline Mat2 operator- (const Mat2& v) const {return Mat2(m00-v.m00, m01-v.m01, m10-v.m10, m11-v.m11);}
    inline Mat2 operator* (double k) const {return Mat2(m00*k, m01*k, m10*k, m11*k);}
    inline Mat2 operator/ (double k) const {return Mat2(m00/k, m01/k, m10/k, m11/k);}
    inline friend Mat2 operator* (double k, const Mat2& v) {return Mat2(k*v.m00, k*v.m01, k*v.m10, k*v.m11);}
    inline friend Mat2 operator/ (double k, const Mat2& v) {return Mat2(k/v.m00, k/v.m01, k/v.m10, k/v.m11);}
    inline Mat2& operator+= (const Mat2& v) {m00+=v.m00; m01+=v.m01; m10+=v.m10; m11+=v.m11; return *this;}
    inline Mat2& operator-= (const Mat2& v) {m00-=v.m00; m01-=v.m01; m10-=v.m10; m11-=v.m11; return *this;}
    inline Mat2& operator*= (double k) {m00*=k; m01*=k; m10*=k; m11*=k; return *this;}
    inline Mat2& operator/= (double k) {m00/=k; m01/=k; m10/=k; m11/=k; return *this;}
    inline Mat2 operator* (const Mat2& m) const {
        return Mat2(
            m00*m.m00 + m10*m.m01,
            m01*m.m00 + m11*m.m01,
            m00*m.m10 + m10*m.m11,
            m01*m.m10 + m11*m.m11
        );
    }
    Vec2 operator* (const Vec2& v) const {
        return Vec2(
            m00*v.x + m10*v.y,
            m01*v.x + m11*v.y
        );
    }
    friend Vec2 operator* (const Vec2& v, const Mat2& m) {
        return Vec2(
            v.x*m.m00 + v.y*m.m01,
            v.x*m.m10 + v.y*m.m11
        );
    }
    double det() const {
        return m00*m11 - m01*m10;
    }
    Mat2 inverse() const {
        double d = m00*m11 - m01*m10;
        return Mat2(
            (m11),
            (-m01),
            (-m10),
            (m00)
        ) / d;
    }
    Mat2 transpose() const {
        return Mat2(
            m00, m10,
            m01, m11
        );
    }

    const double* ptr() const {return &(m00);}
    void print() const {printf("(%f, %f, %f, %f)", m00, m01, m10, m11);}
    void display() const {print(); printf("\n");}
};

class Mat3 {
public:
    double m00, m01, m02, m10, m11, m12, m20, m21, m22;
    Mat3() : m00(1.0), m01(0.0), m02(0.0), m10(0.0), m11(1.0), m12(0.0), m20(0.0), m21(0.0), m22(1.0) {}
    Mat3(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22) : m00(m00), m01(m01), m02(m02), m10(m10), m11(m11), m12(m12), m20(m20), m21(m21), m22(m22) {}
    Mat3(const Vec3& m0, const Vec3& m1, const Vec3& m2) : m00(m0.x), m01(m0.y), m02(m0.z), m10(m1.x), m11(m1.y), m12(m1.z), m20(m2.x), m21(m2.y), m22(m2.z) {}
    
    inline Mat3& operator= (const Mat3& v) {m00=v.m00; m01=v.m01; m02=v.m02; m10=v.m10; m11=v.m11; m12=v.m12; m20=v.m20; m21=v.m21; m22=v.m22; return *this;}
    inline bool operator== (const Mat3& v) const {return m00==v.m00 && m01==v.m01 && m02==v.m02 && m10==v.m10 && m11==v.m11 && m12==v.m12 && m20==v.m20 && m21==v.m21 && m22==v.m22;}
    inline bool operator!= (const Mat3& v) const {return m00!=v.m00 || m01!=v.m01 || m02!=v.m02 || m10!=v.m10 || m11!=v.m11 || m12!=v.m12 || m20!=v.m20 || m21!=v.m21 || m22!=v.m22;}
    inline void set(double in_m00, double in_m01, double in_m02, double in_m10, double in_m11, double in_m12, double in_m20, double in_m21, double in_m22) {m00=in_m00; m01=in_m01; m02=in_m02; m10=in_m10; m11=in_m11; m12=in_m12; m20=in_m20; m21=in_m21; m22=in_m22;}
    inline void set(const double* v) {m00=v[0]; m01=v[1]; m02=v[2]; m10=v[3]; m11=v[4]; m12=v[5]; m20=v[6]; m21=v[7]; m22=v[8];}
    inline Mat3 operator- () const {return Mat3(-m00, -m01, -m02, -m10, -m11, -m12, -m20, -m21, -m22);}
    inline Mat3 operator+ (const Mat3& v) const {return Mat3(m00+v.m00, m01+v.m01, m02+v.m02, m10+v.m10, m11+v.m11, m12+v.m12, m20+v.m20, m21+v.m21, m22+v.m22);}
    inline Mat3 operator- (const Mat3& v) const {return Mat3(m00-v.m00, m01-v.m01, m02-v.m02, m10-v.m10, m11-v.m11, m12-v.m12, m20-v.m20, m21-v.m21, m22-v.m22);}
    inline Mat3 operator* (double k) const {return Mat3(m00*k, m01*k, m02*k, m10*k, m11*k, m12*k, m20*k, m21*k, m22*k);}
    inline Mat3 operator/ (double k) const {return Mat3(m00/k, m01/k, m02/k, m10/k, m11/k, m12/k, m20/k, m21/k, m22/k);}
    inline friend Mat3 operator* (double k, const Mat3& v) {return Mat3(k*v.m00, k*v.m01, k*v.m02, k*v.m10, k*v.m11, k*v.m12, k*v.m20, k*v.m21, k*v.m22);}
    inline friend Mat3 operator/ (double k, const Mat3& v) {return Mat3(k/v.m00, k/v.m01, k/v.m02, k/v.m10, k/v.m11, k/v.m12, k/v.m20, k/v.m21, k/v.m22);}
    inline Mat3& operator+= (const Mat3& v) {m00+=v.m00; m01+=v.m01; m02+=v.m02; m10+=v.m10; m11+=v.m11; m12+=v.m12; m20+=v.m20; m21+=v.m21; m22+=v.m22; return *this;}
    inline Mat3& operator-= (const Mat3& v) {m00-=v.m00; m01-=v.m01; m02-=v.m02; m10-=v.m10; m11-=v.m11; m12-=v.m12; m20-=v.m20; m21-=v.m21; m22-=v.m22; return *this;}
    inline Mat3& operator*= (double k) {m00*=k; m01*=k; m02*=k; m10*=k; m11*=k; m12*=k; m20*=k; m21*=k; m22*=k; return *this;}
    inline Mat3& operator/= (double k) {m00/=k; m01/=k; m02/=k; m10/=k; m11/=k; m12/=k; m20/=k; m21/=k; m22/=k; return *this;}
    inline Mat3 operator* (const Mat3& m) const {
        return Mat3(
            m00*m.m00 + m10*m.m01 + m20*m.m02,
            m01*m.m00 + m11*m.m01 + m21*m.m02,
            m02*m.m00 + m12*m.m01 + m22*m.m02,
            m00*m.m10 + m10*m.m11 + m20*m.m12,
            m01*m.m10 + m11*m.m11 + m21*m.m12,
            m02*m.m10 + m12*m.m11 + m22*m.m12,
            m00*m.m20 + m10*m.m21 + m20*m.m22,
            m01*m.m20 + m11*m.m21 + m21*m.m22,
            m02*m.m20 + m12*m.m21 + m22*m.m22
        );
    }
    Vec3 operator* (const Vec3& v) const {
        return Vec3(
            m00*v.x + m10*v.y + m20*v.z,
            m01*v.x + m11*v.y + m21*v.z,
            m02*v.x + m12*v.y + m22*v.z
        );
    }
    friend Vec3 operator* (const Vec3& v, const Mat3& m) {
        return Vec3(
            v.x*m.m00 + v.y*m.m01 + v.z*m.m02,
            v.x*m.m10 + v.y*m.m11 + v.z*m.m12,
            v.x*m.m20 + v.y*m.m21 + v.z*m.m22
        );
    }
    double det() const {
        return m00*(m11*m22 - m12*m21) - m01*(m10*m22 - m12*m20) + m02*(m10*m21 - m11*m20);
    }
    Mat3 inverse() const {
        double temp_0 = m10*m21 - m11*m20;
        double temp_1 = m11*m22 - m12*m21;
        double temp_2 = m10*m22 - m12*m20;
        double d = m00*temp_1 - m01*temp_2 + m02*temp_0;
        return Mat3(
            temp_1,
            (-(m01*m22 - m02*m21)),
            (m01*m12 - m02*m11),
            (-temp_2),
            (m00*m22 - m02*m20),
            (-(m00*m12 - m02*m10)),
            temp_0,
            (-(m00*m21 - m01*m20)),
            (m00*m11 - m01*m10)
        ) / d;
    }
    Mat3 transpose() const {
        return Mat3(
            m00, m10, m20,
            m01, m11, m21,
            m02, m12, m22
        );
    }

    const double* ptr() const {return &(m00);}
    void print() const {printf("(%f, %f, %f, %f, %f, %f, %f, %f, %f)", m00, m01, m02, m10, m11, m12, m20, m21, m22);}
    void display() const {print(); printf("\n");}
};

class Mat4 {
public:
    double m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33;
    Mat4() : m00(1.0), m01(0.0), m02(0.0), m03(0.0), m10(0.0), m11(1.0), m12(0.0), m13(0.0), m20(0.0), m21(0.0), m22(1.0), m23(0.0), m30(0.0), m31(0.0), m32(0.0), m33(1.0) {}
    Mat4(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23, double m30, double m31, double m32, double m33) : m00(m00), m01(m01), m02(m02), m03(m03), m10(m10), m11(m11), m12(m12), m13(m13), m20(m20), m21(m21), m22(m22), m23(m23), m30(m30), m31(m31), m32(m32), m33(m33) {}
    Mat4(const Vec4& m0, const Vec4& m1, const Vec4& m2, const Vec4& m3) : m00(m0.x), m01(m0.y), m02(m0.z), m03(m0.w), m10(m1.x), m11(m1.y), m12(m1.z), m13(m1.w), m20(m2.x), m21(m2.y), m22(m2.z), m23(m2.w), m30(m3.x), m31(m3.y), m32(m3.z), m33(m3.w) {}
    
    inline Mat4& operator= (const Mat4& v) {m00=v.m00; m01=v.m01; m02=v.m02; m03=v.m03; m10=v.m10; m11=v.m11; m12=v.m12; m13=v.m13; m20=v.m20; m21=v.m21; m22=v.m22; m23=v.m23; m30=v.m30; m31=v.m31; m32=v.m32; m33=v.m33; return *this;}
    inline bool operator== (const Mat4& v) const {return m00==v.m00 && m01==v.m01 && m02==v.m02 && m03==v.m03 && m10==v.m10 && m11==v.m11 && m12==v.m12 && m13==v.m13 && m20==v.m20 && m21==v.m21 && m22==v.m22 && m23==v.m23 && m30==v.m30 && m31==v.m31 && m32==v.m32 && m33==v.m33;}
    inline bool operator!= (const Mat4& v) const {return m00!=v.m00 || m01!=v.m01 || m02!=v.m02 || m03!=v.m03 || m10!=v.m10 || m11!=v.m11 || m12!=v.m12 || m13!=v.m13 || m20!=v.m20 || m21!=v.m21 || m22!=v.m22 || m23!=v.m23 || m30!=v.m30 || m31!=v.m31 || m32!=v.m32 || m33!=v.m33;}
    inline void set(double in_m00, double in_m01, double in_m02, double in_m03, double in_m10, double in_m11, double in_m12, double in_m13, double in_m20, double in_m21, double in_m22, double in_m23, double in_m30, double in_m31, double in_m32, double in_m33) {m00=in_m00; m01=in_m01; m02=in_m02; m03=in_m03; m10=in_m10; m11=in_m11; m12=in_m12; m13=in_m13; m20=in_m20; m21=in_m21; m22=in_m22; m23=in_m23; m30=in_m30; m31=in_m31; m32=in_m32; m33=in_m33;}
    inline void set(const double* v) {m00=v[0]; m01=v[1]; m02=v[2]; m03=v[3]; m10=v[4]; m11=v[5]; m12=v[6]; m13=v[7]; m20=v[8]; m21=v[9]; m22=v[10]; m23=v[11]; m30=v[12]; m31=v[13]; m32=v[14]; m33=v[15];}
    inline Mat4 operator- () const {return Mat4(-m00, -m01, -m02, -m03, -m10, -m11, -m12, -m13, -m20, -m21, -m22, -m23, -m30, -m31, -m32, -m33);}
    inline Mat4 operator+ (const Mat4& v) const {return Mat4(m00+v.m00, m01+v.m01, m02+v.m02, m03+v.m03, m10+v.m10, m11+v.m11, m12+v.m12, m13+v.m13, m20+v.m20, m21+v.m21, m22+v.m22, m23+v.m23, m30+v.m30, m31+v.m31, m32+v.m32, m33+v.m33);}
    inline Mat4 operator- (const Mat4& v) const {return Mat4(m00-v.m00, m01-v.m01, m02-v.m02, m03-v.m03, m10-v.m10, m11-v.m11, m12-v.m12, m13-v.m13, m20-v.m20, m21-v.m21, m22-v.m22, m23-v.m23, m30-v.m30, m31-v.m31, m32-v.m32, m33-v.m33);}
    inline Mat4 operator* (double k) const {return Mat4(m00*k, m01*k, m02*k, m03*k, m10*k, m11*k, m12*k, m13*k, m20*k, m21*k, m22*k, m23*k, m30*k, m31*k, m32*k, m33*k);}
    inline Mat4 operator/ (double k) const {return Mat4(m00/k, m01/k, m02/k, m03/k, m10/k, m11/k, m12/k, m13/k, m20/k, m21/k, m22/k, m23/k, m30/k, m31/k, m32/k, m33/k);}
    inline friend Mat4 operator* (double k, const Mat4& v) {return Mat4(k*v.m00, k*v.m01, k*v.m02, k*v.m03, k*v.m10, k*v.m11, k*v.m12, k*v.m13, k*v.m20, k*v.m21, k*v.m22, k*v.m23, k*v.m30, k*v.m31, k*v.m32, k*v.m33);}
    inline friend Mat4 operator/ (double k, const Mat4& v) {return Mat4(k/v.m00, k/v.m01, k/v.m02, k/v.m03, k/v.m10, k/v.m11, k/v.m12, k/v.m13, k/v.m20, k/v.m21, k/v.m22, k/v.m23, k/v.m30, k/v.m31, k/v.m32, k/v.m33);}
    inline Mat4& operator+= (const Mat4& v) {m00+=v.m00; m01+=v.m01; m02+=v.m02; m03+=v.m03; m10+=v.m10; m11+=v.m11; m12+=v.m12; m13+=v.m13; m20+=v.m20; m21+=v.m21; m22+=v.m22; m23+=v.m23; m30+=v.m30; m31+=v.m31; m32+=v.m32; m33+=v.m33; return *this;}
    inline Mat4& operator-= (const Mat4& v) {m00-=v.m00; m01-=v.m01; m02-=v.m02; m03-=v.m03; m10-=v.m10; m11-=v.m11; m12-=v.m12; m13-=v.m13; m20-=v.m20; m21-=v.m21; m22-=v.m22; m23-=v.m23; m30-=v.m30; m31-=v.m31; m32-=v.m32; m33-=v.m33; return *this;}
    inline Mat4& operator*= (double k) {m00*=k; m01*=k; m02*=k; m03*=k; m10*=k; m11*=k; m12*=k; m13*=k; m20*=k; m21*=k; m22*=k; m23*=k; m30*=k; m31*=k; m32*=k; m33*=k; return *this;}
    inline Mat4& operator/= (double k) {m00/=k; m01/=k; m02/=k; m03/=k; m10/=k; m11/=k; m12/=k; m13/=k; m20/=k; m21/=k; m22/=k; m23/=k; m30/=k; m31/=k; m32/=k; m33/=k; return *this;}
    inline Mat4 operator* (const Mat4& m) const {
        return Mat4(
            m00*m.m00 + m10*m.m01 + m20*m.m02 + m30*m.m03,
            m01*m.m00 + m11*m.m01 + m21*m.m02 + m31*m.m03,
            m02*m.m00 + m12*m.m01 + m22*m.m02 + m32*m.m03,
            m03*m.m00 + m13*m.m01 + m23*m.m02 + m33*m.m03,
            m00*m.m10 + m10*m.m11 + m20*m.m12 + m30*m.m13,
            m01*m.m10 + m11*m.m11 + m21*m.m12 + m31*m.m13,
            m02*m.m10 + m12*m.m11 + m22*m.m12 + m32*m.m13,
            m03*m.m10 + m13*m.m11 + m23*m.m12 + m33*m.m13,
            m00*m.m20 + m10*m.m21 + m20*m.m22 + m30*m.m23,
            m01*m.m20 + m11*m.m21 + m21*m.m22 + m31*m.m23,
            m02*m.m20 + m12*m.m21 + m22*m.m22 + m32*m.m23,
            m03*m.m20 + m13*m.m21 + m23*m.m22 + m33*m.m23,
            m00*m.m30 + m10*m.m31 + m20*m.m32 + m30*m.m33,
            m01*m.m30 + m11*m.m31 + m21*m.m32 + m31*m.m33,
            m02*m.m30 + m12*m.m31 + m22*m.m32 + m32*m.m33,
            m03*m.m30 + m13*m.m31 + m23*m.m32 + m33*m.m33
        );
    }
    Vec4 operator* (const Vec4& v) const {
        return Vec4(
            m00*v.x + m10*v.y + m20*v.z + m30*v.w,
            m01*v.x + m11*v.y + m21*v.z + m31*v.w,
            m02*v.x + m12*v.y + m22*v.z + m32*v.w,
            m03*v.x + m13*v.y + m23*v.z + m33*v.w
        );
    }
    friend Vec4 operator* (const Vec4& v, const Mat4& m) {
        return Vec4(
            v.x*m.m00 + v.y*m.m01 + v.z*m.m02 + v.w*m.m03,
            v.x*m.m10 + v.y*m.m11 + v.z*m.m12 + v.w*m.m13,
            v.x*m.m20 + v.y*m.m21 + v.z*m.m22 + v.w*m.m23,
            v.x*m.m30 + v.y*m.m31 + v.z*m.m32 + v.w*m.m33
        );
    }
    double det() const {
        double temp_0 = m20*m31 - m21*m30;
        double temp_1 = m20*m32 - m22*m30;
        double temp_2 = m21*m33 - m23*m31;
        double temp_3 = m22*m33 - m23*m32;
        double temp_4 = m20*m33 - m23*m30;
        double temp_5 = m21*m32 - m22*m31;
        return m00*(m11*temp_3 - m12*temp_2 + m13*temp_5) - m01*(m10*temp_3 - m12*temp_4 + m13*temp_1) + m02*(m10*temp_2 - m11*temp_4 + m13*temp_0) - m03*(m10*temp_5 - m11*temp_1 + m12*temp_0);
    }
    Mat4 inverse() const {
        double temp_5 = m20*m33 - m23*m30;
        double temp_7 = m20*m31 - m21*m30;
        double temp_9 = m21*m33 - m23*m31;
        double temp_0 = m10*temp_9 - m11*temp_5 + m13*temp_7;
        double temp_6 = m21*m32 - m22*m31;
        double temp_8 = m22*m33 - m23*m32;
        double temp_1 = m11*temp_8 - m12*temp_9 + m13*temp_6;
        double temp_4 = m20*m32 - m22*m30;
        double temp_2 = m10*temp_6 - m11*temp_4 + m12*temp_7;
        double temp_3 = m10*temp_8 - m12*temp_5 + m13*temp_4;
        double temp_10 = m12*m23 - m13*m22;
        double temp_11 = m11*m23 - m13*m21;
        double temp_12 = m11*m33 - m13*m31;
        double temp_13 = m10*m31 - m11*m30;
        double temp_14 = m11*m22 - m12*m21;
        double temp_15 = m10*m22 - m12*m20;
        double temp_16 = m10*m33 - m13*m30;
        double temp_17 = m10*m21 - m11*m20;
        double temp_18 = m11*m32 - m12*m31;
        double temp_19 = m10*m32 - m12*m30;
        double temp_20 = m10*m23 - m13*m20;
        double temp_21 = m12*m33 - m13*m32;
        double d = m00*temp_1 - m01*temp_3 + m02*temp_0 - m03*temp_2;
        return Mat4(
            temp_1,
            (-(m01*temp_8 - m02*temp_9 + m03*temp_6)),
            (m01*temp_21 - m02*temp_12 + m03*temp_18),
            (-(m01*temp_10 - m02*temp_11 + m03*temp_14)),
            (-temp_3),
            (m00*temp_8 - m02*temp_5 + m03*temp_4),
            (-(m00*temp_21 - m02*temp_16 + m03*temp_19)),
            (m00*temp_10 - m02*temp_20 + m03*temp_15),
            temp_0,
            (-(m00*temp_9 - m01*temp_5 + m03*temp_7)),
            (m00*temp_12 - m01*temp_16 + m03*temp_13),
            (-(m00*temp_11 - m01*temp_20 + m03*temp_17)),
            (-temp_2),
            (m00*temp_6 - m01*temp_4 + m02*temp_7),
            (-(m00*temp_18 - m01*temp_19 + m02*temp_13)),
            (m00*temp_14 - m01*temp_15 + m02*temp_17)
        ) / d;
    }
    Mat4 transpose() const {
        return Mat4(
            m00, m10, m20, m30,
            m01, m11, m21, m31,
            m02, m12, m22, m32,
            m03, m13, m23, m33
        );
    }

    const double* ptr() const {return &(m00);}
    void print() const {printf("(%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)", m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33);}
    void display() const {print(); printf("\n");}
};

class Mat5 {
public:
    double m00, m01, m02, m03, m04, m10, m11, m12, m13, m14, m20, m21, m22, m23, m24, m30, m31, m32, m33, m34, m40, m41, m42, m43, m44;
    Mat5() : m00(1.0), m01(0.0), m02(0.0), m03(0.0), m04(0.0), m10(0.0), m11(1.0), m12(0.0), m13(0.0), m14(0.0), m20(0.0), m21(0.0), m22(1.0), m23(0.0), m24(0.0), m30(0.0), m31(0.0), m32(0.0), m33(1.0), m34(0.0), m40(0.0), m41(0.0), m42(0.0), m43(0.0), m44(1.0) {}
    Mat5(double m00, double m01, double m02, double m03, double m04, double m10, double m11, double m12, double m13, double m14, double m20, double m21, double m22, double m23, double m24, double m30, double m31, double m32, double m33, double m34, double m40, double m41, double m42, double m43, double m44) : m00(m00), m01(m01), m02(m02), m03(m03), m04(m04), m10(m10), m11(m11), m12(m12), m13(m13), m14(m14), m20(m20), m21(m21), m22(m22), m23(m23), m24(m24), m30(m30), m31(m31), m32(m32), m33(m33), m34(m34), m40(m40), m41(m41), m42(m42), m43(m43), m44(m44) {}
    Mat5(const Vec5& m0, const Vec5& m1, const Vec5& m2, const Vec5& m3, const Vec5& m4) : m00(m0.x), m01(m0.y), m02(m0.z), m03(m0.w), m04(m0.t), m10(m1.x), m11(m1.y), m12(m1.z), m13(m1.w), m14(m1.t), m20(m2.x), m21(m2.y), m22(m2.z), m23(m2.w), m24(m2.t), m30(m3.x), m31(m3.y), m32(m3.z), m33(m3.w), m34(m3.t), m40(m4.x), m41(m4.y), m42(m4.z), m43(m4.w), m44(m4.t) {}
    
    inline Mat5& operator= (const Mat5& v) {m00=v.m00; m01=v.m01; m02=v.m02; m03=v.m03; m04=v.m04; m10=v.m10; m11=v.m11; m12=v.m12; m13=v.m13; m14=v.m14; m20=v.m20; m21=v.m21; m22=v.m22; m23=v.m23; m24=v.m24; m30=v.m30; m31=v.m31; m32=v.m32; m33=v.m33; m34=v.m34; m40=v.m40; m41=v.m41; m42=v.m42; m43=v.m43; m44=v.m44; return *this;}
    inline bool operator== (const Mat5& v) const {return m00==v.m00 && m01==v.m01 && m02==v.m02 && m03==v.m03 && m04==v.m04 && m10==v.m10 && m11==v.m11 && m12==v.m12 && m13==v.m13 && m14==v.m14 && m20==v.m20 && m21==v.m21 && m22==v.m22 && m23==v.m23 && m24==v.m24 && m30==v.m30 && m31==v.m31 && m32==v.m32 && m33==v.m33 && m34==v.m34 && m40==v.m40 && m41==v.m41 && m42==v.m42 && m43==v.m43 && m44==v.m44;}
    inline bool operator!= (const Mat5& v) const {return m00!=v.m00 || m01!=v.m01 || m02!=v.m02 || m03!=v.m03 || m04!=v.m04 || m10!=v.m10 || m11!=v.m11 || m12!=v.m12 || m13!=v.m13 || m14!=v.m14 || m20!=v.m20 || m21!=v.m21 || m22!=v.m22 || m23!=v.m23 || m24!=v.m24 || m30!=v.m30 || m31!=v.m31 || m32!=v.m32 || m33!=v.m33 || m34!=v.m34 || m40!=v.m40 || m41!=v.m41 || m42!=v.m42 || m43!=v.m43 || m44!=v.m44;}
    inline void set(double in_m00, double in_m01, double in_m02, double in_m03, double in_m04, double in_m10, double in_m11, double in_m12, double in_m13, double in_m14, double in_m20, double in_m21, double in_m22, double in_m23, double in_m24, double in_m30, double in_m31, double in_m32, double in_m33, double in_m34, double in_m40, double in_m41, double in_m42, double in_m43, double in_m44) {m00=in_m00; m01=in_m01; m02=in_m02; m03=in_m03; m04=in_m04; m10=in_m10; m11=in_m11; m12=in_m12; m13=in_m13; m14=in_m14; m20=in_m20; m21=in_m21; m22=in_m22; m23=in_m23; m24=in_m24; m30=in_m30; m31=in_m31; m32=in_m32; m33=in_m33; m34=in_m34; m40=in_m40; m41=in_m41; m42=in_m42; m43=in_m43; m44=in_m44;}
    inline void set(const double* v) {m00=v[0]; m01=v[1]; m02=v[2]; m03=v[3]; m04=v[4]; m10=v[5]; m11=v[6]; m12=v[7]; m13=v[8]; m14=v[9]; m20=v[10]; m21=v[11]; m22=v[12]; m23=v[13]; m24=v[14]; m30=v[15]; m31=v[16]; m32=v[17]; m33=v[18]; m34=v[19]; m40=v[20]; m41=v[21]; m42=v[22]; m43=v[23]; m44=v[24];}
    inline Mat5 operator- () const {return Mat5(-m00, -m01, -m02, -m03, -m04, -m10, -m11, -m12, -m13, -m14, -m20, -m21, -m22, -m23, -m24, -m30, -m31, -m32, -m33, -m34, -m40, -m41, -m42, -m43, -m44);}
    inline Mat5 operator+ (const Mat5& v) const {return Mat5(m00+v.m00, m01+v.m01, m02+v.m02, m03+v.m03, m04+v.m04, m10+v.m10, m11+v.m11, m12+v.m12, m13+v.m13, m14+v.m14, m20+v.m20, m21+v.m21, m22+v.m22, m23+v.m23, m24+v.m24, m30+v.m30, m31+v.m31, m32+v.m32, m33+v.m33, m34+v.m34, m40+v.m40, m41+v.m41, m42+v.m42, m43+v.m43, m44+v.m44);}
    inline Mat5 operator- (const Mat5& v) const {return Mat5(m00-v.m00, m01-v.m01, m02-v.m02, m03-v.m03, m04-v.m04, m10-v.m10, m11-v.m11, m12-v.m12, m13-v.m13, m14-v.m14, m20-v.m20, m21-v.m21, m22-v.m22, m23-v.m23, m24-v.m24, m30-v.m30, m31-v.m31, m32-v.m32, m33-v.m33, m34-v.m34, m40-v.m40, m41-v.m41, m42-v.m42, m43-v.m43, m44-v.m44);}
    inline Mat5 operator* (double k) const {return Mat5(m00*k, m01*k, m02*k, m03*k, m04*k, m10*k, m11*k, m12*k, m13*k, m14*k, m20*k, m21*k, m22*k, m23*k, m24*k, m30*k, m31*k, m32*k, m33*k, m34*k, m40*k, m41*k, m42*k, m43*k, m44*k);}
    inline Mat5 operator/ (double k) const {return Mat5(m00/k, m01/k, m02/k, m03/k, m04/k, m10/k, m11/k, m12/k, m13/k, m14/k, m20/k, m21/k, m22/k, m23/k, m24/k, m30/k, m31/k, m32/k, m33/k, m34/k, m40/k, m41/k, m42/k, m43/k, m44/k);}
    inline friend Mat5 operator* (double k, const Mat5& v) {return Mat5(k*v.m00, k*v.m01, k*v.m02, k*v.m03, k*v.m04, k*v.m10, k*v.m11, k*v.m12, k*v.m13, k*v.m14, k*v.m20, k*v.m21, k*v.m22, k*v.m23, k*v.m24, k*v.m30, k*v.m31, k*v.m32, k*v.m33, k*v.m34, k*v.m40, k*v.m41, k*v.m42, k*v.m43, k*v.m44);}
    inline friend Mat5 operator/ (double k, const Mat5& v) {return Mat5(k/v.m00, k/v.m01, k/v.m02, k/v.m03, k/v.m04, k/v.m10, k/v.m11, k/v.m12, k/v.m13, k/v.m14, k/v.m20, k/v.m21, k/v.m22, k/v.m23, k/v.m24, k/v.m30, k/v.m31, k/v.m32, k/v.m33, k/v.m34, k/v.m40, k/v.m41, k/v.m42, k/v.m43, k/v.m44);}
    inline Mat5& operator+= (const Mat5& v) {m00+=v.m00; m01+=v.m01; m02+=v.m02; m03+=v.m03; m04+=v.m04; m10+=v.m10; m11+=v.m11; m12+=v.m12; m13+=v.m13; m14+=v.m14; m20+=v.m20; m21+=v.m21; m22+=v.m22; m23+=v.m23; m24+=v.m24; m30+=v.m30; m31+=v.m31; m32+=v.m32; m33+=v.m33; m34+=v.m34; m40+=v.m40; m41+=v.m41; m42+=v.m42; m43+=v.m43; m44+=v.m44; return *this;}
    inline Mat5& operator-= (const Mat5& v) {m00-=v.m00; m01-=v.m01; m02-=v.m02; m03-=v.m03; m04-=v.m04; m10-=v.m10; m11-=v.m11; m12-=v.m12; m13-=v.m13; m14-=v.m14; m20-=v.m20; m21-=v.m21; m22-=v.m22; m23-=v.m23; m24-=v.m24; m30-=v.m30; m31-=v.m31; m32-=v.m32; m33-=v.m33; m34-=v.m34; m40-=v.m40; m41-=v.m41; m42-=v.m42; m43-=v.m43; m44-=v.m44; return *this;}
    inline Mat5& operator*= (double k) {m00*=k; m01*=k; m02*=k; m03*=k; m04*=k; m10*=k; m11*=k; m12*=k; m13*=k; m14*=k; m20*=k; m21*=k; m22*=k; m23*=k; m24*=k; m30*=k; m31*=k; m32*=k; m33*=k; m34*=k; m40*=k; m41*=k; m42*=k; m43*=k; m44*=k; return *this;}
    inline Mat5& operator/= (double k) {m00/=k; m01/=k; m02/=k; m03/=k; m04/=k; m10/=k; m11/=k; m12/=k; m13/=k; m14/=k; m20/=k; m21/=k; m22/=k; m23/=k; m24/=k; m30/=k; m31/=k; m32/=k; m33/=k; m34/=k; m40/=k; m41/=k; m42/=k; m43/=k; m44/=k; return *this;}
    inline Mat5 operator* (const Mat5& m) const {
        return Mat5(
            m00*m.m00 + m10*m.m01 + m20*m.m02 + m30*m.m03 + m40*m.m04,
            m01*m.m00 + m11*m.m01 + m21*m.m02 + m31*m.m03 + m41*m.m04,
            m02*m.m00 + m12*m.m01 + m22*m.m02 + m32*m.m03 + m42*m.m04,
            m03*m.m00 + m13*m.m01 + m23*m.m02 + m33*m.m03 + m43*m.m04,
            m04*m.m00 + m14*m.m01 + m24*m.m02 + m34*m.m03 + m44*m.m04,
            m00*m.m10 + m10*m.m11 + m20*m.m12 + m30*m.m13 + m40*m.m14,
            m01*m.m10 + m11*m.m11 + m21*m.m12 + m31*m.m13 + m41*m.m14,
            m02*m.m10 + m12*m.m11 + m22*m.m12 + m32*m.m13 + m42*m.m14,
            m03*m.m10 + m13*m.m11 + m23*m.m12 + m33*m.m13 + m43*m.m14,
            m04*m.m10 + m14*m.m11 + m24*m.m12 + m34*m.m13 + m44*m.m14,
            m00*m.m20 + m10*m.m21 + m20*m.m22 + m30*m.m23 + m40*m.m24,
            m01*m.m20 + m11*m.m21 + m21*m.m22 + m31*m.m23 + m41*m.m24,
            m02*m.m20 + m12*m.m21 + m22*m.m22 + m32*m.m23 + m42*m.m24,
            m03*m.m20 + m13*m.m21 + m23*m.m22 + m33*m.m23 + m43*m.m24,
            m04*m.m20 + m14*m.m21 + m24*m.m22 + m34*m.m23 + m44*m.m24,
            m00*m.m30 + m10*m.m31 + m20*m.m32 + m30*m.m33 + m40*m.m34,
            m01*m.m30 + m11*m.m31 + m21*m.m32 + m31*m.m33 + m41*m.m34,
            m02*m.m30 + m12*m.m31 + m22*m.m32 + m32*m.m33 + m42*m.m34,
            m03*m.m30 + m13*m.m31 + m23*m.m32 + m33*m.m33 + m43*m.m34,
            m04*m.m30 + m14*m.m31 + m24*m.m32 + m34*m.m33 + m44*m.m34,
            m00*m.m40 + m10*m.m41 + m20*m.m42 + m30*m.m43 + m40*m.m44,
            m01*m.m40 + m11*m.m41 + m21*m.m42 + m31*m.m43 + m41*m.m44,
            m02*m.m40 + m12*m.m41 + m22*m.m42 + m32*m.m43 + m42*m.m44,
            m03*m.m40 + m13*m.m41 + m23*m.m42 + m33*m.m43 + m43*m.m44,
            m04*m.m40 + m14*m.m41 + m24*m.m42 + m34*m.m43 + m44*m.m44
        );
    }
    Vec5 operator* (const Vec5& v) const {
        return Vec5(
            m00*v.x + m10*v.y + m20*v.z + m30*v.w + m40*v.t,
            m01*v.x + m11*v.y + m21*v.z + m31*v.w + m41*v.t,
            m02*v.x + m12*v.y + m22*v.z + m32*v.w + m42*v.t,
            m03*v.x + m13*v.y + m23*v.z + m33*v.w + m43*v.t,
            m04*v.x + m14*v.y + m24*v.z + m34*v.w + m44*v.t
        );
    }
    friend Vec5 operator* (const Vec5& v, const Mat5& m) {
        return Vec5(
            v.x*m.m00 + v.y*m.m01 + v.z*m.m02 + v.w*m.m03 + v.t*m.m04,
            v.x*m.m10 + v.y*m.m11 + v.z*m.m12 + v.w*m.m13 + v.t*m.m14,
            v.x*m.m20 + v.y*m.m21 + v.z*m.m22 + v.w*m.m23 + v.t*m.m24,
            v.x*m.m30 + v.y*m.m31 + v.z*m.m32 + v.w*m.m33 + v.t*m.m34,
            v.x*m.m40 + v.y*m.m41 + v.z*m.m42 + v.w*m.m43 + v.t*m.m44
        );
    }
    double det() const {
        double temp_10 = m31*m44 - m34*m41;
        double temp_12 = m31*m43 - m33*m41;
        double temp_19 = m33*m44 - m34*m43;
        double temp_0 = m21*temp_19 - m23*temp_10 + m24*temp_12;
        double temp_15 = m31*m42 - m32*m41;
        double temp_17 = m32*m43 - m33*m42;
        double temp_1 = m21*temp_17 - m22*temp_12 + m23*temp_15;
        double temp_11 = m30*m43 - m33*m40;
        double temp_13 = m30*m41 - m31*m40;
        double temp_2 = m20*temp_12 - m21*temp_11 + m23*temp_13;
        double temp_14 = m30*m42 - m32*m40;
        double temp_3 = m20*temp_15 - m21*temp_14 + m22*temp_13;
        double temp_18 = m30*m44 - m34*m40;
        double temp_4 = m20*temp_10 - m21*temp_18 + m24*temp_13;
        double temp_16 = m32*m44 - m34*m42;
        double temp_5 = m20*temp_16 - m22*temp_18 + m24*temp_14;
        double temp_6 = m22*temp_19 - m23*temp_16 + m24*temp_17;
        double temp_7 = m21*temp_16 - m22*temp_10 + m24*temp_15;
        double temp_8 = m20*temp_19 - m23*temp_18 + m24*temp_11;
        double temp_9 = m20*temp_17 - m22*temp_11 + m23*temp_14;
        return m00*(m11*temp_6 - m12*temp_0 + m13*temp_7 - m14*temp_1) - m01*(m10*temp_6 - m12*temp_8 + m13*temp_5 - m14*temp_9) + m02*(m10*temp_0 - m11*temp_8 + m13*temp_4 - m14*temp_2) - m03*(m10*temp_7 - m11*temp_5 + m12*temp_4 - m14*temp_3) + m04*(m10*temp_1 - m11*temp_9 + m12*temp_2 - m13*temp_3);
    }
    Mat5 inverse() const {
        double temp_45 = m30*m44 - m34*m40;
        double temp_50 = m30*m42 - m32*m40;
        double temp_54 = m32*m44 - m34*m42;
        double temp_11 = m20*temp_54 - m22*temp_45 + m24*temp_50;
        double temp_46 = m32*m43 - m33*m42;
        double temp_49 = m30*m43 - m33*m40;
        double temp_12 = m20*temp_46 - m22*temp_49 + m23*temp_50;
        double temp_47 = m33*m44 - m34*m43;
        double temp_13 = m22*temp_47 - m23*temp_54 + m24*temp_46;
        double temp_14 = m20*temp_47 - m23*temp_45 + m24*temp_49;
        double temp_0 = m10*temp_13 - m12*temp_14 + m13*temp_11 - m14*temp_12;
        double temp_52 = m31*m44 - m34*m41;
        double temp_53 = m31*m43 - m33*m41;
        double temp_6 = m21*temp_47 - m23*temp_52 + m24*temp_53;
        double temp_51 = m31*m42 - m32*m41;
        double temp_9 = m21*temp_46 - m22*temp_53 + m23*temp_51;
        double temp_10 = m21*temp_54 - m22*temp_52 + m24*temp_51;
        double temp_1 = m11*temp_13 - m12*temp_6 + m13*temp_10 - m14*temp_9;
        double temp_48 = m30*m41 - m31*m40;
        double temp_7 = m20*temp_52 - m21*temp_45 + m24*temp_48;
        double temp_8 = m20*temp_51 - m21*temp_50 + m22*temp_48;
        double temp_2 = m10*temp_10 - m11*temp_11 + m12*temp_7 - m14*temp_8;
        double temp_5 = m20*temp_53 - m21*temp_49 + m23*temp_48;
        double temp_3 = m10*temp_6 - m11*temp_14 + m13*temp_7 - m14*temp_5;
        double temp_4 = m10*temp_9 - m11*temp_12 + m12*temp_5 - m13*temp_8;
        double temp_63 = m22*m44 - m24*m42;
        double temp_66 = m21*m44 - m24*m41;
        double temp_67 = m21*m42 - m22*m41;
        double temp_15 = m11*temp_63 - m12*temp_66 + m14*temp_67;
        double temp_58 = m22*m43 - m23*m42;
        double temp_60 = m20*m43 - m23*m40;
        double temp_74 = m20*m42 - m22*m40;
        double temp_16 = m10*temp_58 - m12*temp_60 + m13*temp_74;
        double temp_61 = m20*m41 - m21*m40;
        double temp_17 = m10*temp_67 - m11*temp_74 + m12*temp_61;
        double temp_55 = m20*m34 - m24*m30;
        double temp_68 = m21*m34 - m24*m31;
        double temp_70 = m20*m31 - m21*m30;
        double temp_18 = m10*temp_68 - m11*temp_55 + m14*temp_70;
        double temp_19 = m10*temp_51 - m11*temp_50 + m12*temp_48;
        double temp_20 = m10*temp_54 - m12*temp_45 + m14*temp_50;
        double temp_57 = m23*m44 - m24*m43;
        double temp_59 = m21*m43 - m23*m41;
        double temp_21 = m11*temp_57 - m13*temp_66 + m14*temp_59;
        double temp_22 = m10*temp_46 - m12*temp_49 + m13*temp_50;
        double temp_23 = m11*temp_58 - m12*temp_59 + m13*temp_67;
        double temp_65 = m22*m34 - m24*m32;
        double temp_69 = m22*m33 - m23*m32;
        double temp_72 = m23*m34 - m24*m33;
        double temp_24 = m12*temp_72 - m13*temp_65 + m14*temp_69;
        double temp_25 = m11*temp_54 - m12*temp_52 + m14*temp_51;
        double temp_64 = m20*m33 - m23*m30;
        double temp_73 = m21*m33 - m23*m31;
        double temp_26 = m10*temp_73 - m11*temp_64 + m13*temp_70;
        double temp_27 = m12*temp_57 - m13*temp_63 + m14*temp_58;
        double temp_28 = m10*temp_53 - m11*temp_49 + m13*temp_48;
        double temp_29 = m10*temp_47 - m13*temp_45 + m14*temp_49;
        double temp_62 = m20*m32 - m22*m30;
        double temp_71 = m21*m32 - m22*m31;
        double temp_30 = m10*temp_71 - m11*temp_62 + m12*temp_70;
        double temp_31 = m11*temp_69 - m12*temp_73 + m13*temp_71;
        double temp_32 = m11*temp_72 - m13*temp_68 + m14*temp_73;
        double temp_33 = m10*temp_69 - m12*temp_64 + m13*temp_62;
        double temp_34 = m12*temp_47 - m13*temp_54 + m14*temp_46;
        double temp_35 = m10*temp_65 - m12*temp_55 + m14*temp_62;
        double temp_36 = m10*temp_59 - m11*temp_60 + m13*temp_61;
        double temp_56 = m20*m44 - m24*m40;
        double temp_37 = m10*temp_63 - m12*temp_56 + m14*temp_74;
        double temp_38 = m10*temp_66 - m11*temp_56 + m14*temp_61;
        double temp_39 = m10*temp_72 - m13*temp_55 + m14*temp_64;
        double temp_40 = m11*temp_47 - m13*temp_52 + m14*temp_53;
        double temp_41 = m10*temp_52 - m11*temp_45 + m14*temp_48;
        double temp_42 = m11*temp_46 - m12*temp_53 + m13*temp_51;
        double temp_43 = m10*temp_57 - m13*temp_56 + m14*temp_60;
        double temp_44 = m11*temp_65 - m12*temp_68 + m14*temp_71;
        double d = m00*temp_1 - m01*temp_0 + m02*temp_3 - m03*temp_2 + m04*temp_4;
        return Mat5(
            temp_1,
            (-(m01*temp_13 - m02*temp_6 + m03*temp_10 - m04*temp_9)),
            (m01*temp_34 - m02*temp_40 + m03*temp_25 - m04*temp_42),
            (-(m01*temp_27 - m02*temp_21 + m03*temp_15 - m04*temp_23)),
            (m01*temp_24 - m02*temp_32 + m03*temp_44 - m04*temp_31),
            (-temp_0),
            (m00*temp_13 - m02*temp_14 + m03*temp_11 - m04*temp_12),
            (-(m00*temp_34 - m02*temp_29 + m03*temp_20 - m04*temp_22)),
            (m00*temp_27 - m02*temp_43 + m03*temp_37 - m04*temp_16),
            (-(m00*temp_24 - m02*temp_39 + m03*temp_35 - m04*temp_33)),
            temp_3,
            (-(m00*temp_6 - m01*temp_14 + m03*temp_7 - m04*temp_5)),
            (m00*temp_40 - m01*temp_29 + m03*temp_41 - m04*temp_28),
            (-(m00*temp_21 - m01*temp_43 + m03*temp_38 - m04*temp_36)),
            (m00*temp_32 - m01*temp_39 + m03*temp_18 - m04*temp_26),
            (-temp_2),
            (m00*temp_10 - m01*temp_11 + m02*temp_7 - m04*temp_8),
            (-(m00*temp_25 - m01*temp_20 + m02*temp_41 - m04*temp_19)),
            (m00*temp_15 - m01*temp_37 + m02*temp_38 - m04*temp_17),
            (-(m00*temp_44 - m01*temp_35 + m02*temp_18 - m04*temp_30)),
            temp_4,
            (-(m00*temp_9 - m01*temp_12 + m02*temp_5 - m03*temp_8)),
            (m00*temp_42 - m01*temp_22 + m02*temp_28 - m03*temp_19),
            (-(m00*temp_23 - m01*temp_16 + m02*temp_36 - m03*temp_17)),
            (m00*temp_31 - m01*temp_33 + m02*temp_26 - m03*temp_30)
        ) / d;
    }
    Mat5 transpose() const {
        return Mat5(
            m00, m10, m20, m30, m40,
            m01, m11, m21, m31, m41,
            m02, m12, m22, m32, m42,
            m03, m13, m23, m33, m43,
            m04, m14, m24, m34, m44
        );
    }

    const double* ptr() const {return &(m00);}
    void print() const {printf("(%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)", m00, m01, m02, m03, m04, m10, m11, m12, m13, m14, m20, m21, m22, m23, m24, m30, m31, m32, m33, m34, m40, m41, m42, m43, m44);}
    void display() const {print(); printf("\n");}
};


#endif
    
