#include "pch.h"
#include "Vec2.h"

float Vec2::dot(const Vec2& other) const
{
    return x * other.x + y * other.y;
}

float Vec2::squared_length() const
{
    return x * x + y * y;
}

float Vec2::length() const
{
    return sqrtf(x * x + y * y);
}

float Vec2::cross(const Vec2& other) const
{
    return x * other.y - y * other.x;
}


void Vec2::normalize()
{
    //Multiply by reciprocal to prevent two divides
    float rl = 1.f / length();
    x *= rl;
    y *= rl;
}

Vec2 Vec2::normalized() const
{
    //Multiply by reciprocal to prevent two divides
    float rl = 1.f / length();
    return Vec2(x * rl, y * rl);
}

Vec2 Vec2::operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
Vec2 Vec2::operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
Vec2 Vec2::operator*(const float& scalar) const { return Vec2(x * scalar, y * scalar); }
Vec2 Vec2::operator/(const float& scalar) const { return Vec2(x / scalar, y / scalar); }

Vec2 Vec2::operator-() const { return Vec2(-x, -y); }

Vec2 Vec2::operator+=(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
Vec2 Vec2::operator-=(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
Vec2 Vec2::operator*=(const float& scalar) const { return Vec2(x * scalar, y * scalar); }
Vec2 Vec2::operator/=(const float& scalar) const { return Vec2(x / scalar, y / scalar); }

bool Vec2::operator==(const Vec2& operand) const { return nearly_equal(x, operand.x) && nearly_equal(y, operand.y); }
bool Vec2::operator!=(const Vec2& operand) const { return !(*this == operand); }

//Returns true if y is lower than y of operand, if y is equal, returns true if x is lower than operand x.
bool Vec2::operator<(const Vec2& operand) const
{
    if (nearly_equal(y, operand.y))
    {
        if (nearly_equal(x, operand.x))
        {
            return false;
        }

        return x < operand.x;
    }

    return y < operand.y;
}

bool Vec2::operator>(const Vec2& operand) const
{
    return !(*this == operand) && !(*this < operand);
}

Vec2 operator*(const float& scalar, const Vec2& vec) { return Vec2(vec.x * scalar, vec.y * scalar); }
Vec2 operator/(const float& scalar, const Vec2& vec) { return Vec2(vec.x / scalar, vec.y / scalar); }