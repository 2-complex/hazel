
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


#include "transforms.h"

namespace g2c {


/*    Computes a perspective transformation matrix given the angular height
    of the frustum, the aspect ratio, and the near and far clipping planes.*/
Mat4 perspective(double angle, double aspect, double near, double far)
{
    double f = tan(0.5 * (M_PI - angle));
    double range = near - far;
    return Mat4(f / aspect,  0.0,  0.0,                          0.0,
                0.0,         f,    0.0,                          0.0,
                0.0,         0.0,  (far + near) / range,        -1.0,
                0.0,         0.0,  2.0 * near * far / range,     0.0);
};

/*    Computes an orthographic projection matrix given the coordinates of the
    planes defining the viewing volume.*/
Mat4 orthographic(double left, double right, double bottom, double top, double near, double far) {
    return Mat4(
        2.0 / (right - left), 0.0, 0.0, 0.0,
        0.0, 2.0 / (top - bottom), 0.0, 0.0,
        0.0, 0.0, 2.0 / (far - near), 0.0,
        (left + right) / (left - right),
        (bottom + top) / (bottom - top),
        (near + far)  / (far - near), 1.0);
};

/*    Computes a perspective transformation matrix given the left, right,
    top, bottom, near and far clipping planes.*/
Mat4 frustum(double left, double right, double bottom, double top, double near, double far) {
    double dx = (right - left);
    double dy = (top - bottom);
    double dz = (near - far);
    return Mat4(
        2.0 * near / dx, 0.0, 0.0, 0.0,
        0.0, 2.0 * near / dy, 0.0, 0.0,
        (left + right) / dx, (top + bottom) / dy, far / dz, -1.0,
        0.0, 0.0, near * far / dz, 0.0);
};

/*    Computes a look-at transformation.*/
Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
    Vec3 vz = eye - target;
    Vec3 vx = up.cross(vz);
    Vec3 vy = vz.cross(vx);
    
    vx /= vx.mag();
    vy /= vy.mag();
    vz /= vz.mag();
    
    return Mat4(
        vx.x, vx.y, vx.z, 0.0,
        vy.x, vy.y, vy.z, 0.0,
        vz.x, vz.y, vz.z, 0.0,
        eye.x, eye.y, eye.z, 1.0).inverse();
};

/*    Creates a 4-by-4 matrix which translates by the given vector v.*/
Mat4 translate(const Vec3& v) {
    return Mat4(
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        v.x, v.y, v.z, 1.0);
};

/*    Creates a matrix which scales in each dimension by an amount given by
    the corresponding entry in the given vector.*/
Mat4 scale(const Vec3& v) {
    return Mat4(
        v.x, 0.0, 0.0, 0.0,
        0.0, v.y, 0.0, 0.0,
        0.0, 0.0, v.z, 0.0,
        0.0, 0.0, 0.0, 1.0);
};

/*    Creates a 4-by-4 matrix which rotates around the x-axis by the given angle.*/
Mat4 rotateX(double theta) {
    double c = cos(theta);
    double s = sin(theta);
    return Mat4(
        1.0, 0.0, 0.0, 0.0,
        0.0, c, s, 0.0,
        0.0, -s, c, 0.0,
        0.0, 0.0, 0.0, 1.0);
};

/*    Creates a 4-by-4 matrix which rotates around the y-axis by the given angle.*/
Mat4 rotateY(double theta) {
    double c = cos(theta);
    double s = sin(theta);
    return Mat4(
        c, 0.0, -s, 0.0,
        0.0, 1, 0.0, 0.0,
        s, 0.0, c, 0.0,
        0.0, 0.0, 0.0, 1.0);
};

/*    Creates a 4-by-4 matrix which rotates around the z-axis by the given angle.*/
Mat4 rotateZ(double theta) {
    double c = cos(theta);
    double s = sin(theta);
    return Mat4(
        c, s, 0.0, 0.0,
        -s, c, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0);
};

/*    Creates a 4-by-4 matrix which rotates around the given axis by the given
    angle in a right-handed manner.*/
Mat4 axisRotate(const Vec3& axis, double theta)
{
    double x = axis.x, y = axis.y, z = axis.z;
    double n = axis.mag();
    x /= n;
    y /= n;
    z /= n;
    double xx = x * x, yy = y * y, zz = z * z;
    double c = cos(theta);
    double s = sin(theta);
    double omc = 1 - c; // one minus cosine
    
    return Mat4(xx + (1 - xx) * c,
        x * y * omc + z * s,
        x * z * omc - y * s,
        0.0,
        x * y * omc - z * s,
        yy + (1.0 - yy) * c,
        y * z * omc + x * s,
        0.0,
        x * z * omc + y * s,
        y * z * omc - x * s,
        zz + (1.0 - zz) * c,
        0.0,
        0.0, 0.0, 0.0, 1.0);
};

/*    Retrieves the upper 3x3 matrix of a 4x4 transformation matrix.*/
Mat3 upper3x3(const Mat4& m)
{
    return Mat3(m.m00, m.m01, m.m02,
                m.m10, m.m11, m.m12,
                m.m20, m.m21, m.m22);
}

} // end namespace
