#include "3d.hpp"
#include <cmath>
#include <stdexcept>


namespace cpp3d{

using std::exception;
using std::invalid_argument;

vec3d::vec3d(float x, float y, float z)
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->base_x = x;
    this->base_y = y;
    this->base_z = z;
}

vec3d vec3d::add(const vec3d &a) const
{
    return vec3d(this->x + a.x, this->y + a.y, this->z + a.z);
}

vec3d vec3d::sub(const vec3d &a) const
{
    return vec3d(this->x - a.x, this->y - a.y, this->z - a.z);
}

vec3d vec3d::scalar(float k) const
{
    return vec3d(this->x * k, this->y * k, this->z * k);
}

float vec3d::dot(const vec3d &a) const
{
    return this->x * a.x + this->y * a.y + this->z * a.z;
}

vec3d vec3d::cross(const vec3d &a) const
{
    return vec3d(
        this->y * a.z - this->z * a.y,
        this->z * a.x - this->x * a.z,
        this->x * a.y - this->y * a.x);
}

float vec3d::abs() const
{
    return sqrt(
        this->x * this->x +
        this->y * this->y +
        this->z * this->z);
}

vec3d vec3d::normalize() const
{
    float abs = this->abs();
    return vec3d(
        this->x / abs,
        this->y / abs,
        this->z / abs);
}

float vec3d::get_cos(const vec3d &a) const
{
    float dot = this->dot(a);
    float this_abs = this->abs();
    float a_abs = a.abs();
    return dot / (this_abs * a_abs);
}

void vec3d::reset()
{
    this->x = this->base_x;
    this->y = this->base_y;
    this->z = this->base_z;
}

vec3d vec3d::operator+(const vec3d &a) const
{
    return this->add(a);
}

vec3d vec3d::operator-(const vec3d &a) const
{
    return this->sub(a);
}

vec3d vec3d::operator*(float k) const
{
    return this->scalar(k);
}

bool vec3d::operator==(const vec3d &a) const
{
    const float epsilon = 1e-6f;
    return (std::abs(this->x - a.x) < epsilon &&
            std::abs(this->y - a.y) < epsilon &&
            std::abs(this->z - a.z) < epsilon);
}

quaternion::quaternion(float w, float x, float y, float z)
{
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
    this->base_w = w;
    this->base_x = x;
    this->base_y = y;
    this->base_z = z;
}

quaternion quaternion::add(const quaternion &a) const
{
    return quaternion(
        this->w + a.w,
        this->x + a.x,
        this->y + a.y,
        this->z + a.z);
}

quaternion quaternion::sub(const quaternion &a) const
{
    return quaternion(
        this->w - a.w,
        this->x - a.x,
        this->y - a.y,
        this->z - a.z);
}

quaternion quaternion::scalar(float k) const
{
    return quaternion(
        this->w * k,
        this->x * k,
        this->y * k,
        this->z * k);
}

quaternion quaternion::mul(const quaternion &a) const
{
    return quaternion(
        this->w * a.w - this->x * a.x - this->y * a.y - this->z * a.z,
        this->w * a.x + this->x * a.w + this->y * a.z - this->z * a.y,
        this->w * a.y + this->y * a.w + this->z * a.x - this->x * a.z,
        this->w * a.z + this->z * a.w + this->x * a.y - this->y * a.x);
}

quaternion quaternion::conjugate() const
{
    return quaternion(
        this->w,
        -this->x,
        -this->y,
        -this->z);
}

float quaternion::abs() const
{
    return sqrt(
        this->w * this->w +
        this->x * this->x +
        this->y * this->y +
        this->z * this->z);
}

quaternion quaternion::normalize() const
{
    float abs = this->abs();
    return quaternion(
        this->w / abs,
        this->x / abs,
        this->y / abs,
        this->z / abs);
}

void quaternion::reset()
{
    this->w = this->base_w;
    this->x = this->base_x;
    this->y = this->base_y;
    this->z = this->base_z;
}

quaternion quaternion::operator+(const quaternion &a) const
{
    return this->add(a);
}

quaternion quaternion::operator-(const quaternion &a) const
{
    return this->sub(a);
}

quaternion quaternion::operator*(float k) const
{
    return this->scalar(k);
}

quaternion quaternion::operator*(const quaternion &a) const
{
    return this->mul(a);
}

bool quaternion::operator==(const quaternion &a) const
{
    const float epsilon = 1e-6f;
    return (
        std::abs(this->w - a.w) < epsilon &&
        std::abs(this->x - a.x) < epsilon &&
        std::abs(this->y - a.y) < epsilon &&
        std::abs(this->z - a.z) < epsilon);
}
}