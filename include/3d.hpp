#ifndef CPP3D_HPP
#define CPP3D_HPP
#include <vector>
#include <stdexcept>

namespace cpp3d{

using std::vector;

class vec3d
{
public:
    float x, y, z, base_x, base_y, base_z;
    vec3d(float x, float y, float z);
    vec3d add(const vec3d &a) const;
    vec3d sub(const vec3d &a) const;
    vec3d scalar(float k) const;
    float dot(const vec3d &a) const;
    vec3d cross(const vec3d &a) const;
    float abs() const;
    vec3d normalize() const;
    float get_cos(const vec3d &a) const;
    float scale();
    float calc_point();
    void reset();

    vec3d operator+(const vec3d &a) const;
    vec3d operator-(const vec3d &a) const;
    vec3d operator*(float k) const;
    bool operator==(const vec3d &a) const;
};

class quaternion
{
public:
    float w, x, y, z, base_w, base_x, base_y, base_z;
    quaternion(float w, float x, float y, float z);
    quaternion add(const quaternion &a) const;
    quaternion sub(const quaternion &a) const;
    quaternion scalar(float k) const;
    quaternion mul(const quaternion &a) const;
    quaternion conjugate() const;
    float abs() const;
    quaternion normalize() const;
    void reset();

    quaternion operator+(const quaternion &a) const;
    quaternion operator-(const quaternion &a) const;
    quaternion operator*(float k) const;
    quaternion operator*(const quaternion &a) const;
    bool operator==(const quaternion &a) const;
};
}
#endif