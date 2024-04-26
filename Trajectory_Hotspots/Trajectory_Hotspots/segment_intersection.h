#pragma once

#include "sweep_line_status_structure.h"

namespace Segment_Intersection_Sweep_Line
{
    template<typename SegmentT>
    class Sweep_Line_Status_structure; //forward declaration

    struct Event_Point_Comparer
    {
        //Higher (in the y-axis) events go first. If on the same y-axis, most left goes first (smaller x-axis).
        //Checks if a occurs before b
        bool operator()(const Vec2& a, const Vec2& b) const
        {
            if (a.y == b.y)
            {
                if (a.x == b.x)
                {
                    return false;
                }

                return a.x < b.x;
            }

            return a.y > b.y;
        }
    };

    //Event queue sorted on point, values are a segment list where the event point is the top point of the segment
    using Event_Queue = std::map<const Vec2, std::vector<int>, Event_Point_Comparer>;

    template<typename SegmentT>
    std::vector<Vec2> find_segment_intersections(const std::vector<SegmentT>& segments);

    //Check if two potentially intersecting segments intersect. Add the event to the event_queue if they do and the intersection point is in the future.
    template<typename SegmentT>
    void test_for_intersection(const std::vector<SegmentT>& segments, const int left_segment, const int right_segment, const Vec2& event_point, Event_Queue& event_queue);

    template<typename SegmentT, typename Event_Handler>
    void Handle_Event(
        Sweep_Line_Status_structure<SegmentT>& status_structure,
        Event_Queue& event_queue,
        const std::vector<SegmentT>& segments,
        const Vec2& event_point,
        std::vector<int>& top_segments,
        Event_Handler event_handler)
    {
        std::vector<int> inner_segments;
        std::vector<int> bottom_segments;

        Float line_pos = event_point.y;
        status_structure.set_line_position(line_pos);

        int left_neighbour = -1;
        int right_neighbour = -1;

        //Get all nodes containing segments that intersect this event point
        std::vector<int> ordered_intersecting_segments = status_structure.get_all_nodes_on_point(segments, event_point, left_neighbour, right_neighbour);

        int most_left_segment = -1;
        int most_right_segment = -1;

        //Remove all the intersecting segments, 
        //the inner intersections are reinsterted later (in reversed order)
        for (int segment : ordered_intersecting_segments)
        {
            status_structure.remove(segments, segment);
        }

        //Process the segments at this point with the given event handler
        event_handler(ordered_intersecting_segments, top_segments);

        //Gather all inner segments for re-insertion
        for (int segment : ordered_intersecting_segments)
        {
            if (*segments[segment].get_bottom_point() != event_point)
            {
                inner_segments.push_back(segment);
            }
        }

        //Reinsert the segments that still intersect the sweepline
        for (int segment : inner_segments)
        {
            status_structure.insert(segments, segment);
        }

        //Store the outer segments indices (they swapped after the event point)
        if (!inner_segments.empty())
        {
            most_left_segment = inner_segments.back();
            most_right_segment = inner_segments.front();
        }

        //If we only have top segments we did not find the neighboring nodes yet
        bool neighbours_found = !ordered_intersecting_segments.empty();

        //Insert the segments that intersect this event point with their top point,
        //finding the outer neighbours if we didn't already.
        int new_left_neighbour = -1;
        int new_right_neighbour = -1;
        for (int segment : top_segments)
        {
            status_structure.insert(segments, segment, new_left_neighbour, new_right_neighbour);

            //If we did not have intersecting or bottom segments we don't know the neighbours of this point.
            //In this case, the first top insert gives us these neighbours
            if (!neighbours_found)
            {
                left_neighbour = new_left_neighbour;
                right_neighbour = new_right_neighbour;
                neighbours_found = true;
            }

            //If the neighbour of this inserted segment does not intersect the event point, this segment is the new outer segment.
            //TODO: right side of these checks are useless?
            if (new_left_neighbour == left_neighbour || left_neighbour == -1)
            {
                most_left_segment = segment;
            }

            if (new_right_neighbour == right_neighbour || right_neighbour == -1)
            {
                most_right_segment = segment;
            }
        }

        //Check if there are new intersection events between the outer segments and their outer neighbours
        if (top_segments.empty() && inner_segments.empty())
        {
            //Only bottom segments on this point, after their removal we need to check if their neighbours intersect in the future
            if (left_neighbour != -1 && right_neighbour != -1)
            {
                test_for_intersection(segments, left_neighbour, right_neighbour, event_point, event_queue);
            }
        }
        else
        {
            //If we added or swapped segments we need to check if the most left and right of these segments intersects their left and right neighbour respectively.
            Vec2 intersection_point;
            if (left_neighbour != -1)
            {
                test_for_intersection(segments, left_neighbour, most_left_segment, event_point, event_queue);
            }

            if (right_neighbour != -1)
            {
                test_for_intersection(segments, most_right_segment, right_neighbour, event_point, event_queue);
            }
        }
    }

    template<typename SegmentT>
    void test_for_intersection(const std::vector<SegmentT>& segments, const int left_segment, const int right_segment, const Vec2& event_point, Event_Queue& event_queue)
    {
        Vec2 intersection_point;
        Segment::Intersection_Type intersection_type = segments.at(left_segment).intersects(segments.at(right_segment), intersection_point);

        //Report the intersection if the intersection is in the future, 
        //intersections above the sweep line (and left of this point) have been reported already
        switch (intersection_type)
        {

        case Segment::Intersection_Type::point:
        {
            if (Event_Point_Comparer comp; comp(event_point, intersection_point))
            {
                auto [event_point_key, event_top_segments] = event_queue.try_emplace(intersection_point);
            }

            break;
        }
        case Segment::Intersection_Type::collinear:
        {
            //TODO: This is just the end points of the segment, which should already in the queue. Isnt this just a break?

            //Collinear intersections are handled around the start and end of their overlap.
            //If these points are in the future, add them to the event queue
            Vec2 overlap_start;
            Vec2 overlap_end;

            if (collinear_overlap(segments.at(left_segment), segments.at(right_segment), overlap_start, overlap_start))
            {
                Event_Point_Comparer comp;
                if (comp(event_point, overlap_start))
                {
                    auto [event_point_key, event_top_segments] = event_queue.try_emplace(overlap_start);
                }

                if (comp(event_point, overlap_end))
                {
                    auto [event_point_key, event_top_segments] = event_queue.try_emplace(overlap_end);
                }
            }
            else
            {
                assert(false); //Collinear but no overlap? Should never happen.
            }

            break;
        }
        case Segment::Intersection_Type::parallel:
        case Segment::Intersection_Type::none:
        {
            break;
        }
        }
    }

    template<typename SegmentT>
    std::vector<Vec2> find_segment_intersections(const std::vector<SegmentT>& segments)
    {
        Event_Queue event_queue;

        //Initialize event queue with the start and endpoints of the segments
        for (int i = 0; i < segments.size(); i++)
        {
            auto event_pair = event_queue.emplace(*segments.at(i).get_top_point(), std::vector<int>());
            event_pair.first->second.push_back(i);

            event_pair = event_queue.emplace(*segments.at(i).get_bottom_point(), std::vector<int>());
        }

        //Initialize status structure with the highest event point
        Sweep_Line_Status_structure<SegmentT> status_structure(event_queue.begin()->first.y);

        std::vector<Vec2> intersections;

        //Sweep through the segments in order and report intersection points
        while (!event_queue.empty())
        {
            Handle_Event(status_structure, event_queue, segments, event_queue.begin()->first, event_queue.begin()->second, [&intersections, &event_queue](const std::vector<int>& intersecting_segments, const std::vector<int>& top_segments) {


                if (intersecting_segments.size() + top_segments.size() > 1)
                {
                    intersections.push_back(event_queue.begin()->first);
                }

                });


            event_queue.erase(event_queue.begin());
        }

        return intersections;
    }
}