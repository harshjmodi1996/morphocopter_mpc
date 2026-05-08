#pragma once

/**
 * conversion_functions.hpp
 *
 * C++ conversion of conversion_functions.py
 * Provides conversions between rotation matrices, quaternions, and Euler angles
 * using the ZYX convention.
 *
 * Quaternion convention throughout: q = [w, x, y, z]
 * Rotation matrix type: using a simple 3x3 struct (Mat3) to avoid external deps.
 */

#include <array>
#include <cmath>
#include <stdexcept>

// ─────────────────────────────────────────────────────────────────────────────
// Types
// ─────────────────────────────────────────────────────────────────────────────

/** Quaternion: {w, x, y, z} */
using Quaternion = std::array<double, 4>;

/** Euler angles: {roll, pitch, yaw} in radians */
using RPY = std::array<double, 3>;

/** 3-D vector */
using Vec3 = std::array<double, 3>;

/**
 * Row-major 3×3 rotation matrix.
 * Access element (row, col) as data[row*3 + col].
 */
struct Mat3 {
    double data[9]{};

    double  operator()(int r, int c) const { return data[r * 3 + c]; }
    double& operator()(int r, int c)       { return data[r * 3 + c]; }
};

// ─────────────────────────────────────────────────────────────────────────────
// Matrix helpers
// ─────────────────────────────────────────────────────────────────────────────

/** Matrix–matrix multiplication (3×3). */
inline Mat3 matmul(const Mat3& A, const Mat3& B)
{
    Mat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                C(i, j) += A(i, k) * B(k, j);
    return C;
}

/** Matrix–vector multiplication (3×3 · 3). */
inline Vec3 matmul(const Mat3& A, const Vec3& v)
{
    Vec3 result{};
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k)
            result[i] += A(i, k) * v[k];
    return result;
}

/** Trace of a 3×3 matrix. */
inline double trace(const Mat3& R)
{
    return R(0,0) + R(1,1) + R(2,2);
}

// ─────────────────────────────────────────────────────────────────────────────
// q2rpy  —  quaternion → roll, pitch, yaw  (ZYX convention)
// ─────────────────────────────────────────────────────────────────────────────
/**
 * Convert quaternion [w, x, y, z] to roll-pitch-yaw (ZYX convention).
 * Reference: https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
 */
inline RPY q2rpy(const Quaternion& q)
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];

    double roll  = std::atan2(2.0 * (z * z + w * x),
                              1.0 - 2.0 * (x * x + y * y));
    double pitch = std::asin (2.0 * (y * w - z * x));
    double yaw   = std::atan2(2.0 * (z * w + x * y),
                             -1.0 + 2.0 * (w * w + x * x));
    return {roll, pitch, yaw};
}

// ─────────────────────────────────────────────────────────────────────────────
// rpy2rotmat  —  roll, pitch, yaw → rotation matrix  (ZYX convention)
// ─────────────────────────────────────────────────────────────────────────────
inline Mat3 rpy2rotmat(double r, double p, double y)
{
    const double cy = std::cos(y), sy = std::sin(y);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cr = std::cos(r), sr = std::sin(r);

    // Rz(y)
    Mat3 Rz{};
    Rz(0,0)= cy; Rz(0,1)=-sy; Rz(0,2)=0.0;
    Rz(1,0)= sy; Rz(1,1)= cy; Rz(1,2)=0.0;
    Rz(2,0)=0.0; Rz(2,1)=0.0; Rz(2,2)=1.0;

    // Ry(p)
    Mat3 Ry{};
    Ry(0,0)= cp; Ry(0,1)=0.0; Ry(0,2)= sp;
    Ry(1,0)=0.0; Ry(1,1)=1.0; Ry(1,2)=0.0;
    Ry(2,0)=-sp; Ry(2,1)=0.0; Ry(2,2)= cp;

    // Rx(r)
    Mat3 Rx{};
    Rx(0,0)=1.0; Rx(0,1)=0.0; Rx(0,2)= 0.0;
    Rx(1,0)=0.0; Rx(1,1)= cr; Rx(1,2)= -sr;
    Rx(2,0)=0.0; Rx(2,1)= sr; Rx(2,2)=  cr;

    return matmul(matmul(Rz, Ry), Rx);
}

// ─────────────────────────────────────────────────────────────────────────────
// rotmat2q  —  rotation matrix → quaternion  (numerically stable)
// ─────────────────────────────────────────────────────────────────────────────
/**
 * Shepperd method for stable rotation matrix → quaternion conversion.
 * Reference: https://github.com/harshjmodi1996/CAL_UB_harsh_src/...
 */
inline Quaternion rotmat2q(const Mat3& R)
{
    Quaternion q{};
    double tr = trace(R);

    if (tr > 0.0) {
        double S = std::sqrt(tr + 1.0) * 2.0;   // S = 4*w
        q[0] = 0.25 * S;
        q[1] = (R(2,1) - R(1,2)) / S;
        q[2] = (R(0,2) - R(2,0)) / S;
        q[3] = (R(1,0) - R(0,1)) / S;
    } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) {
        double S = std::sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2.0; // S = 4*x
        q[0] = (R(2,1) - R(1,2)) / S;
        q[1] = 0.25 * S;
        q[2] = (R(0,1) + R(1,0)) / S;
        q[3] = (R(0,2) + R(2,0)) / S;
    } else if (R(1,1) > R(2,2)) {
        double S = std::sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2.0; // S = 4*y
        q[0] = (R(0,2) - R(2,0)) / S;
        q[1] = (R(0,1) + R(1,0)) / S;
        q[2] = 0.25 * S;
        q[3] = (R(1,2) + R(2,1)) / S;
    } else {
        double S = std::sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2.0; // S = 4*z
        q[0] = (R(1,0) - R(0,1)) / S;
        q[1] = (R(0,2) + R(2,0)) / S;
        q[2] = (R(1,2) + R(2,1)) / S;
        q[3] = 0.25 * S;
    }
    return q;
}

// ─────────────────────────────────────────────────────────────────────────────
// rpy2q  —  roll, pitch, yaw → quaternion  (ZYX convention)
// ─────────────────────────────────────────────────────────────────────────────
inline Quaternion rpy2q(double r, double p, double y)
{
    return rotmat2q(rpy2rotmat(r, p, y));
}

// ─────────────────────────────────────────────────────────────────────────────
// q2rotmat  —  quaternion → rotation matrix
// ─────────────────────────────────────────────────────────────────────────────
inline Mat3 q2rotmat(const Quaternion& q)
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];

    Mat3 R{};
    R(0,0) = 2.0*(w*w + x*x) - 1.0;
    R(0,1) = 2.0*(x*y - w*z);
    R(0,2) = 2.0*(x*z + w*y);

    R(1,0) = 2.0*(x*y + w*z);
    R(1,1) = 2.0*(w*w + y*y) - 1.0;
    R(1,2) = 2.0*(y*z - w*x);

    R(2,0) = 2.0*(x*z - w*y);
    R(2,1) = 2.0*(y*z + w*x);
    R(2,2) = 2.0*(w*w + z*z) - 1.0;

    return R;
}

// ─────────────────────────────────────────────────────────────────────────────
// rotmat2rpy  —  rotation matrix → roll, pitch, yaw  (ZYX convention)
// ─────────────────────────────────────────────────────────────────────────────
/**
 * Reference: https://stackoverflow.com/questions/11514063/
 *            extract-yaw-pitch-and-roll-from-a-rotationmatrix
 */
inline RPY rotmat2rpy(const Mat3& R)
{
    double yaw   = std::atan2(R(1,0), R(0,0));
    double pitch = std::atan2(-R(2,0),
                              std::sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
    double roll  = std::atan2(R(2,1), R(2,2));
    return {roll, pitch, yaw};
}

// ─────────────────────────────────────────────────────────────────────────────
// convert_rotation  —  Pixhawk FRD → UAV-configuration FRD
// ─────────────────────────────────────────────────────────────────────────────
/**
 * Convert a measured rotation quaternion from the Pixhawk FRD frame into the
 * UAV-configuration FRD frame by applying a yaw rotation of -joint_angle.
 */
inline Quaternion convert_rotation(double joint_angle, const Quaternion& q)
{
    const double half = -joint_angle / 2.0;
    const double c = std::cos(half), s = std::sin(half);

    Mat3 yaw_rotation{};
    yaw_rotation(0,0)= c;  yaw_rotation(0,1)=-s;  yaw_rotation(0,2)=0.0;
    yaw_rotation(1,0)= s;  yaw_rotation(1,1)= c;  yaw_rotation(1,2)=0.0;
    yaw_rotation(2,0)=0.0; yaw_rotation(2,1)=0.0; yaw_rotation(2,2)=1.0;

    Mat3 rotation_matrix        = q2rotmat(q);
    Mat3 rotation_matrix_FRD_J  = matmul(rotation_matrix, yaw_rotation);
    return rotmat2q(rotation_matrix_FRD_J);
}

// ─────────────────────────────────────────────────────────────────────────────
// convert_rotation_back  —  UAV-configuration FRD → Pixhawk FRD
// ─────────────────────────────────────────────────────────────────────────────
/**
 * Convert quaternion commands from the UAV-configuration FRD frame back to
 * the Pixhawk FRD frame by applying a yaw rotation of +joint_angle.
 */
inline Quaternion convert_rotation_back(double joint_angle, const Quaternion& q)
{
    const double half = joint_angle / 2.0;
    const double c = std::cos(half), s = std::sin(half);

    Mat3 yaw_rotation{};
    yaw_rotation(0,0)= c;  yaw_rotation(0,1)=-s;  yaw_rotation(0,2)=0.0;
    yaw_rotation(1,0)= s;  yaw_rotation(1,1)= c;  yaw_rotation(1,2)=0.0;
    yaw_rotation(2,0)=0.0; yaw_rotation(2,1)=0.0; yaw_rotation(2,2)=1.0;

    Mat3 rotation_matrix             = q2rotmat(q);
    Mat3 rotation_matrix_FRD_Pixhawk = matmul(rotation_matrix, yaw_rotation);
    return rotmat2q(rotation_matrix_FRD_Pixhawk);
}

// ─────────────────────────────────────────────────────────────────────────────
// convert_axes  —  rotate a 3-D vector by a yaw of joint_angle/2
// ─────────────────────────────────────────────────────────────────────────────
/**
 * Convert measured rotation speeds / torques between Pixhawk FRD and
 * UAV-configuration FRD frames.
 *   +joint_angle : Pixhawk FRD → Joint FRD
 *   -joint_angle : Joint FRD → Pixhawk FRD
 */
inline Vec3 convert_axes(double joint_angle, const Vec3& v)
{
    const double half = joint_angle / 2.0;
    const double c = std::cos(half), s = std::sin(half);

    Mat3 yaw_rotation{};
    yaw_rotation(0,0)= c;  yaw_rotation(0,1)=-s;  yaw_rotation(0,2)=0.0;
    yaw_rotation(1,0)= s;  yaw_rotation(1,1)= c;  yaw_rotation(1,2)=0.0;
    yaw_rotation(2,0)=0.0; yaw_rotation(2,1)=0.0; yaw_rotation(2,2)=1.0;

    return matmul(yaw_rotation, v);
}

// ─────────────────────────────────────────────────────────────────────────────
// euclidean_distance  —  ‖p1 − p2‖₂
// ─────────────────────────────────────────────────────────────────────────────
inline double euclidean_distance(const Vec3& p1, const Vec3& p2)
{
    double dx = p1[0]-p2[0], dy = p1[1]-p2[1], dz = p1[2]-p2[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
