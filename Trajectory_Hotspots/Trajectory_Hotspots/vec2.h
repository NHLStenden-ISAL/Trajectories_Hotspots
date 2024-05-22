#pragma once

//Represents a 2d vector
class Vec2
{
public:

    Vec2() = default;

    Vec2(const Float x, const Float y) : x(x), y(y)
    {
    }

    Float dot(const Vec2& other) const;

    Float cross(const Vec2& other) const;

    Float squared_length() const;

    Float length() const;

    void normalize();

    Vec2 normalized() const;

    Vec2 operator+(const Vec2& other) const;
    Vec2 operator-(const Vec2& other) const;
    Vec2 operator*(const Float& scalar) const;
    Vec2 operator/(const Float& scalar) const;

    Vec2 operator-() const;

    Vec2 operator+=(const Vec2& other) const;
    Vec2 operator-=(const Vec2& other) const;
    Vec2 operator*=(const Float& scalar) const;
    Vec2 operator/=(const Float& scalar) const;

    bool operator==(const Vec2& operand) const;
    bool operator!=(const Vec2& operand) const;

    bool operator<(const Vec2& operand) const;
    bool operator>(const Vec2& operand) const;

    //Checks if this point lies between two given points
    //Assumes the three points are collinear
    bool between(const Vec2& first, const Vec2& second) const;

    Float x;
    Float y;

    //Returns the pseudo angle rotating counter clockwise from given vertices a to b, around a given center.
    [[nodiscard]]
    static Float order_around_center(const Vec2& center, const Vec2& a, const Vec2& b)
    {
        Float angle_diff = pseudo_angle(center, b) - pseudo_angle(center, a);

        //pseudo_angle returns range [0,4), wrap around if the angle difference is lower than zero.
        if (angle_diff < 0.f)
        {
            return angle_diff + 4.f;
        }
        else
        {
            return angle_diff;
        }
    };

    //Returns the angle of a given vec around a given center.
    //Angle is represented in the range [0,4), starting at direction (0,-1) and increasing while rotating counterclockwise.
    //Note that it doesn't give an actual angle in degrees but it does preserve ordering.
    [[nodiscard]]
    static Float pseudo_angle(const Vec2& center, const Vec2& vec)
    {
        assert(center != vec);
        Vec2 angle_vec = vec - center;
        Float p = (angle_vec.y / (std::abs(angle_vec.x.get_value()) + std::abs(angle_vec.y.get_value()))) + 1.f;

        if (angle_vec.x < 0.f)
        {
            return 4.f - p;
        }

        return p;
    };

private:

};


Vec2 operator*(const Float& scalar, const Vec2& vec);
Vec2 operator/(const Float& scalar, const Vec2& vec);