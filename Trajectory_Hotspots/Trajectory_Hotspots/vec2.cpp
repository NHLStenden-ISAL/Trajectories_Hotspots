#include "pch.h"
#include "vec2.h"

Float Vec2::dot(const Vec2& other) const
{
    return x * other.x + y * other.y;
}

//Cross product of two dimensional vectors (aka perp dot)
Float Vec2::cross(const Vec2& other) const
{
    return (x * other.y) - (y * other.x);
}

Float Vec2::squared_length() const
{
    return x * x + y * y;
}

Float Vec2::length() const
{
    return sqrtf((x * x + y * y).get_value());
}

void Vec2::normalize()
{
    //Multiply by reciprocal to prevent two divides
    Float rl = 1.f / length();
    x *= rl;
    y *= rl;
}

Vec2 Vec2::normalized() const
{
    //Multiply by reciprocal to prevent two divides
    Float rl = 1.f / length();
    return Vec2(x * rl, y * rl);
}

Vec2 Vec2::operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
Vec2 Vec2::operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
Vec2 Vec2::operator*(const Float& scalar) const { return Vec2(x * scalar, y * scalar); }
Vec2 Vec2::operator/(const Float& scalar) const { return Vec2(x / scalar, y / scalar); }

Vec2 Vec2::operator-() const { return Vec2(-x, -y); }

Vec2 Vec2::operator+=(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
Vec2 Vec2::operator-=(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
Vec2 Vec2::operator*=(const Float& scalar) const { return Vec2(x * scalar, y * scalar); }
Vec2 Vec2::operator/=(const Float& scalar) const { return Vec2(x / scalar, y / scalar); }

bool Vec2::operator==(const Vec2& operand) const { return x == operand.x && y == operand.y; }
bool Vec2::operator!=(const Vec2& operand) const { return !(*this == operand); }

//Returns true if y is lower than y of operand, if y is equal, returns true if x is lower than operand x.
bool Vec2::operator<(const Vec2& operand) const
{
    if (y == operand.y)
    {
        if (x == operand.x)
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

bool Vec2::between(const Vec2& first, const Vec2& second) const
{
    if (first.x != second.x)
    {
        return ((first.x <= x) && (x <= second.x)) ||
            ((first.x >= x) && (x >= second.x));
    }
    else
    {
        return ((first.y <= y) && (y <= second.y)) ||
            ((first.y >= y) && (y >= second.y));
    }
}

Vec2 operator*(const Float& scalar, const Vec2& vec) { return Vec2(vec.x * scalar, vec.y * scalar); }
Vec2 operator/(const Float& scalar, const Vec2& vec) { return Vec2(vec.x / scalar, vec.y / scalar); }