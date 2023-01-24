#include "pch.h"
#include "vec2.h"
#include "aabb.h"
#include "segment.h"

Float Segment::length() const
{
    Vec2 distance_vector = start - end;
    return distance_vector.length();
}

Float Segment::squared_length() const
{
    Vec2 distance_vector = start - end;
    return distance_vector.squared_length();
}

const Vec2* Segment::get_bottom_point() const
{
    if (start.y > end.y)
    {
        return &end;
    }
    else
    {
        return &start;
    }

}

const Vec2* Segment::get_top_point() const
{
    if (start.y > end.y)
    {
        return &start;
    }
    else
    {
        return &end;
    }

}

//Check if segments share a y-axis
bool Segment::x_overlap(const Segment& other_segment) const
{
    //sort the endpoints
    const Vec2& left_vert = (start.x <= end.x ? start : end);
    const Vec2& right_vert = (start.x > end.x ? start : end);

    const Vec2& other_left_vert = (other_segment.start.x <= other_segment.end.x ? other_segment.start : other_segment.end);
    const Vec2& other_right_vert = (other_segment.start.x > other_segment.end.x ? other_segment.start : other_segment.end);

    return !(right_vert.x < other_left_vert.x || left_vert.x > other_right_vert.x);
}

//Check if segments share a x-axis
bool Segment::y_overlap(const Segment& other_segment) const
{
    //sort the endpoints
    const Vec2& lower_vert = (start.y <= end.y ? start : end);
    const Vec2& upper_vert = (start.y > end.y ? start : end);

    const Vec2& other_lower_vert = (other_segment.start.y <= other_segment.end.y ? other_segment.start : other_segment.end);
    const Vec2& other_upper_vert = (other_segment.start.y > other_segment.end.y ? other_segment.start : other_segment.end);

    return !(upper_vert.y < other_lower_vert.y || lower_vert.y > other_upper_vert.y);
}

AABB Segment::get_AABB() const
{
    return AABB(std::min(start.x, end.x), std::min(start.y, end.y), std::max(start.x, end.x), std::max(start.y, end.y));
}

//Finds the points on the given segments that share the same axis, and lie a given length away from each other
//When axis is set to true, uses the x-axis, else the y-axis
bool Segment::get_points_on_same_axis_with_distance_l(const Segment& start_segment, const Segment& end_segment, const Float length, const bool axis, Vec2& point_on_start_segment, Vec2& point_on_end_segment)
{
    if (axis)
    {
        if (!start_segment.x_overlap(end_segment))
        {
            return false;
        }
    }
    else
    {
        if (!start_segment.y_overlap(end_segment))
        {
            return false;
        }
    }

    const Float start_axis_difference = axis ? start_segment.end.x - start_segment.start.x : start_segment.end.y - start_segment.start.y;
    const Float end_axis_difference = axis ? end_segment.end.x - end_segment.start.x : end_segment.end.y - end_segment.start.y;

    Float start_length = start_segment.length();
    Float end_length = end_segment.length();

    const Float determinant = start_axis_difference * end_length - end_axis_difference * start_length;

    //Determinant is zero when the two segments lie on the same line, skip
    if (determinant == 0.f)
    {
        return false;
    }

    const Float edge_distance = (end_segment.start_t - start_segment.end_t);
    const Float remaining_length = length - edge_distance;

    const Float start_end_difference = axis ? start_segment.end.x - end_segment.start.x : start_segment.end.y - end_segment.start.y;

    //Calculate the scalar for the vectors pointing to points p and q
    const Float lambda = ((start_end_difference * end_length) - (end_axis_difference * remaining_length)) / determinant;
    const Float rho = ((start_axis_difference * remaining_length) - (start_end_difference * start_length)) / determinant;

    //Return false if p or q do not lie on their respective segment
    if (lambda < 0.f || lambda > 1.0f || rho < 0.f || rho > 1.0f)
    {
        return false;
    }

    //Calculate start and end point using the scalars
    point_on_start_segment = start_segment.end + (lambda * (start_segment.start - start_segment.end));
    point_on_end_segment = end_segment.start + (rho * (end_segment.end - end_segment.start));

    return true;
}

//Returns the y-coordinate of the intersection with the vertical line at x, or infinity if it lies on the segment
Float Segment::x_intersect(Float x) const
{
    Float x_diff = end.x - start.x;

    //Vertical segment, either no or infinite intersections
    if (x_diff == 0.f)
    {
        return std::numeric_limits<float>::infinity();
    }
    else
    {
        Float segment_to_x = x - start.x;
        Float intersection_y = start.y + ((segment_to_x / x_diff) * (end.y - start.y));

        return intersection_y;
    }
}

//Returns the x-coordinate of the intersection with the horizontal line at y, or infinity if it lies on the segment
Float Segment::y_intersect(Float y) const
{
    Float y_diff = end.y - start.y;

    //Horizontal segment, either no or infinite intersections
    if (y_diff == 0.f)
    {
        return std::numeric_limits<float>::infinity();
    }
    else
    {
        Float segment_to_y = y - start.y;
        Float intersection_x = start.x + ((segment_to_y / y_diff) * (end.x - start.x));

        return intersection_x;
    }
}

//If the vertical line at position x intersects the segment, gets the y coordinate of the intersection
bool Segment::x_intersects(const Float x, Float& intersection_point_y) const
{
    //If the query line is left or right of segment, return false
    if (x < start.x && x < end.x)
    {
        return false;
    }
    else if (x > start.x && x > end.x)
    {
        return false;
    }

    //Intersection, get intersection point and return true
    intersection_point_y = x_intersect(x);

    return true;
}

//If the horizontal line at position y intersects the segment, gets the x coordinate of the intersection
bool Segment::y_intersects(const Float y, Float& intersection_point_x) const
{
    //If the query line is above or below segment, return false
    if (y < start.y && y < end.y)
    {
        return false;
    }
    else if (y > start.y && y > end.y)
    {
        return false;
    }

    //Intersection, get intersection point and return true
    intersection_point_x = y_intersect(y);

    return true;
}

Float Segment::get_time_at_x(const Float x) const
{
    Float x_fraction = (x - start.x) / (end.x - start.x);
    return start_t + (x_fraction * (end_t - start_t));
}

Float Segment::get_time_at_y(const Float y) const
{
    Float y_fraction = (y - start.y) / (end.y - start.y);
    return start_t + (y_fraction * (end_t - start_t));
}

Float Segment::get_time_at_point(const Vec2& point) const
{
    Vec2 start_to_point = point - start;
    Float time_fraction = start_to_point.length() / length();
    return start_t + (time_fraction * (end_t - start_t));
}

//Returns the point on the segment at a given time
Vec2 Segment::get_point_at_time(const Float time) const
{
    Float fraction = (time - start_t) / (end_t - start_t);

    Vec2 vector_to_point = (end - start) * fraction;

    return (start + vector_to_point);
}

//Determine if a point lies to the left (false) or right (true) of a segment, oriented from start to end
//If the point lies on the segment this function will return true (right)
bool point_right_of_segment(const Segment& segment, const Vec2& point)
{
    //TODO: Force end always top so we can remove this check?
    const Vec2* top = segment.get_top_point();
    const Vec2* bottom = segment.get_bottom_point();

    Float direction = (point.x - bottom->x) * (top->y - bottom->y) - (point.y - bottom->y) * (top->x - bottom->x);

    return direction < 0 ? false : true;
}

bool Segment::operator==(const Segment& operand) const
{
    return (start == operand.start && end == operand.end) || (end == operand.start && start == operand.end);
}

bool Segment::operator!=(const Segment& operand) const
{
    return !(*this == operand);;
}