#include "pch.h"
#include "dcel.h"

#include "segment.h"
#include "segment_intersection.h"

//Overlay the given DCEL over this DCEL, creating new vertices at the intersection points.
//Warning: This overlay function destroys the given DCEL.
void DCEL::overlay_dcel(DCEL& other_dcel)
{
    Overlay_Handler overlay_handler(*this, other_dcel);


}

void DCEL::insert_segment(const Segment& segment)
{

    if (half_edges.empty())
    {
        create_free_segment_records(segment);
        return;
    }
    else
    {
        DCEL dcel_with_single_segment;
        dcel_with_single_segment.insert_segment(segment);

        overlay_dcel(dcel_with_single_segment);
    }
}

//Creates two new half-edges and two new vertices based on the given segment.
//The two half edges are each others twin, prev, and next.
//Returns a pointer to one of the new half-edges.
DCEL::DCEL_Half_Edge* DCEL::create_free_segment_records(const Segment& segment)
{
    vertices.reserve(vertices.size() + 2);
    const auto& start_vertex = vertices.emplace_back(std::make_unique<DCEL_Vertex>(segment.start));
    const auto& end_vertex = vertices.emplace_back(std::make_unique<DCEL_Vertex>(segment.end));

    half_edges.reserve(half_edges.size() + 2);
    const auto& new_half_edge_1 = half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>(start_vertex.get()));
    const auto& new_half_edge_2 = half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>(end_vertex.get(), nullptr, new_half_edge_1.get(), new_half_edge_1.get(), new_half_edge_1.get()));

    new_half_edge_1->twin = new_half_edge_2.get();
    new_half_edge_1->next = new_half_edge_2.get();
    new_half_edge_1->prev = new_half_edge_2.get();

    start_vertex->incident_half_edge = new_half_edge_1.get();
    end_vertex->incident_half_edge = new_half_edge_2.get();

    return new_half_edge_1.get();
}

DCEL::DCEL_Vertex* DCEL::get_vertex_at_position(const Vec2& position)
{
    for (const auto& vert : vertices)
    {
        if (vert->position == position)
        {
            return vert.get();
        }
    }

    return nullptr;
}

void DCEL::Overlay_Handler::handle_point_intersection(const Vec2& intersection_point, DCEL::DCEL_Half_Edge* old_half_edge, DCEL::DCEL_Half_Edge* new_half_edge)
{
    DCEL_Vertex* old_dcel_endpoint = nullptr;
    DCEL_Vertex* new_dcel_endpoint = nullptr;
    intersection_on_endpoint(intersection_point, old_half_edge, new_half_edge, old_dcel_endpoint, new_dcel_endpoint);

    if (old_dcel_endpoint || new_dcel_endpoint)
    {
        if (old_dcel_endpoint && new_dcel_endpoint)
        {
            overlay_vertex_on_vertex(old_dcel_endpoint, new_dcel_endpoint);
        }
        else if (new_dcel_endpoint)
        {
            overlay_edge_on_vertex(old_half_edge, new_dcel_endpoint);
        }
        else if (old_dcel_endpoint)
        {
            overlay_edge_on_vertex(new_half_edge, old_dcel_endpoint);
        }
    }
    else
    {
        overlay_edge_on_edge(old_half_edge, new_half_edge, intersection_point);
    }
}

void DCEL::Overlay_Handler::intersection_on_endpoint(
    const Vec2& intersection_point,
    const DCEL::DCEL_Half_Edge* old_half_edge,
    const DCEL::DCEL_Half_Edge* new_half_edge,
    DCEL_Vertex*& old_overlapping_vertex,
    DCEL_Vertex*& new_overlapping_vertex) const
{
    //Check if the intersection point lies on one of the endpoints of either segment.

    if (intersection_point == new_half_edge->origin->position)
    {
        new_overlapping_vertex = new_half_edge->origin;
    }
    else if (intersection_point == new_half_edge->target()->position)
    {
        new_overlapping_vertex = new_half_edge->target();
    }

    if (intersection_point == old_half_edge->origin->position)
    {
        old_overlapping_vertex = old_half_edge->origin;
    }
    else if (intersection_point == old_half_edge->target()->position)
    {
        old_overlapping_vertex = old_half_edge->target();
    }
}

void DCEL::Overlay_Handler::overlay_edge_on_vertex(DCEL::DCEL_Half_Edge* edge, DCEL::DCEL_Vertex* vertex)
{
    //Note: Technically there's some duplicate code here because the handling of the original half-edges is almost identical.
    //Splitting this up gives some trouble with overwriting the next pointers though.
    //Also, this way we can start the search for the second adjacent half-edges where the first ended.

    //Shorten old half-edges to vertex and create new twins for them.

    original_dcel.half_edges.reserve(original_dcel.half_edges.size() + 2); //Reserve two extra spots to prevent invalidating the first reference when exceeding space on the second emplace_back
    const std::unique_ptr<DCEL_Half_Edge>& new_half_edge_1 = original_dcel.half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>()); //new twin of edge
    const std::unique_ptr<DCEL_Half_Edge>& new_half_edge_2 = original_dcel.half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>()); //new twin of edge->twin

    DCEL_Half_Edge* twin = edge->twin;

    //Old half_edges point to vertex, the new twins originate from it.
    new_half_edge_1->origin = vertex;
    new_half_edge_2->origin = vertex;

    //Switch twins to new half-edges
    edge->twin = new_half_edge_1.get();
    new_half_edge_1->twin = edge;

    twin->twin = new_half_edge_2.get();
    new_half_edge_2->twin = twin;

    //Set next of new to next of shortened half-edges
    new_half_edge_1->next = twin->next;
    new_half_edge_2->next = edge->next;
    new_half_edge_1->next->prev = new_half_edge_1.get();
    new_half_edge_2->next->prev = new_half_edge_2.get();

    //Update prev and next pointers around the vertex
    //Find positions of new half-edges around the vertex and insert

    DCEL_Half_Edge* CW_half_edge = nullptr; //Clockwise half-edge
    DCEL_Half_Edge* CCW_half_edge = nullptr; //Counter clockwise half-edge

    DCEL_Half_Edge* current_half_edge = vertex->incident_half_edge->twin->next;

    original_dcel.add_edge_to_vertex(*edge->twin, *vertex, *current_half_edge, CW_half_edge, CCW_half_edge);

    //Find the other side, start from the end of the previous rotation (it is impossible for it to lie between the just added and CW half-edge)
    original_dcel.add_edge_to_vertex(*twin->twin, *vertex, *CCW_half_edge, CW_half_edge, CCW_half_edge);

}

DCEL::DCEL_Vertex* DCEL::Overlay_Handler::overlay_edge_on_edge(DCEL_Half_Edge* edge_1, DCEL_Half_Edge* edge_2, const Vec2& intersection_point)
{
    const std::unique_ptr<DCEL_Vertex>& new_dcel_vertex = original_dcel.vertices.emplace_back(std::make_unique<DCEL_Vertex>(intersection_point));

    DCEL_Half_Edge* edge_1_old_twin = edge_1->twin;

    //Create two new twins for the first edge, 
    //these are outgoing from the new vertex so we set one of them as the incident half edge
    original_dcel.half_edges.reserve(original_dcel.half_edges.size() + 2); //Reserve two extra spots to prevent invalidating the first reference when exceeding space on the second emplace_back
    const std::unique_ptr<DCEL_Half_Edge>& edge_1_new_twin_1 = original_dcel.half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>());
    const std::unique_ptr<DCEL_Half_Edge>& edge_1_new_twin_2 = original_dcel.half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>());

    edge_1_new_twin_1->origin = new_dcel_vertex.get();
    edge_1_new_twin_2->origin = new_dcel_vertex.get();

    new_dcel_vertex->incident_half_edge = edge_1_new_twin_1.get();

    edge_1_new_twin_1->twin = edge_1;
    edge_1->twin = edge_1_new_twin_1.get();

    edge_1_new_twin_2->twin = edge_1_old_twin;
    edge_1_old_twin->twin = edge_1_new_twin_2.get();

    //Set next of new to next of shortened half-edges
    edge_1_new_twin_2->next = edge_1->next;
    edge_1_new_twin_2->next->prev = edge_1_new_twin_2.get();

    edge_1_new_twin_1->next = edge_1_old_twin->next;
    edge_1_new_twin_1->next->prev = edge_1_new_twin_1.get();

    //Chain the four half-edges under edge_1 together
    //so we can simply call the edge on vertex function for the second edge.
    edge_1_new_twin_1->prev = edge_1_old_twin;
    edge_1_old_twin->next = edge_1_new_twin_1.get();

    edge_1_new_twin_2->prev = edge_1;
    edge_1->next = edge_1_new_twin_2.get();

    //Insert second edge using the overlay edge over vertex function
    overlay_edge_on_vertex(edge_2, new_dcel_vertex.get());

    return new_dcel_vertex.get();
}

void DCEL::Overlay_Handler::overlay_vertex_on_vertex(DCEL_Vertex* vertex_1, DCEL_Vertex* vertex_2) const
{
    //Gather half-edges around second vertex
    //For each: Set origins to first vertex
    //          and switch prev, prev->next,
    //          twin->next, and twin->next->prev pointers

    std::vector<DCEL_Half_Edge*> incident_half_edges_v2 = vertex_2->get_incident_half_edges();

    for (DCEL_Half_Edge* incident_half_edge_v2 : incident_half_edges_v2)
    {
        original_dcel.add_edge_to_vertex(*incident_half_edge_v2, *vertex_1);
    }
}

void DCEL::add_edge_to_vertex(DCEL::DCEL_Half_Edge& incident_half_edge, DCEL::DCEL_Vertex& vertex) const
{
    DCEL_Half_Edge* CW_half_edge = nullptr; //Clockwise half-edge
    DCEL_Half_Edge* CCW_half_edge = nullptr; //Counter clockwise half-edge
    add_edge_to_vertex(incident_half_edge, vertex, *vertex.incident_half_edge, CW_half_edge, CCW_half_edge);
}

void DCEL::add_edge_to_vertex(DCEL::DCEL_Half_Edge& incident_half_edge, DCEL::DCEL_Vertex& vertex, DCEL::DCEL_Half_Edge& current_half_edge, DCEL::DCEL_Half_Edge*& CW_half_edge, DCEL::DCEL_Half_Edge*& CCW_half_edge) const
{
    incident_half_edge.origin = &vertex;

    vertex.find_adjacent_half_edges(incident_half_edge, current_half_edge, CW_half_edge, CCW_half_edge);

    //half-edges chain counter clockwise and adjacent half-edges point outward
    //so: CW half-edge is just the CW adjacent
    //    and CCW half-edge is the twin of CCW adjacent

    //(We can prevent the two twin writes by checking if the CCW is the previous added half-edge
    //but adding branching is probably slower)

    incident_half_edge.twin->next = CW_half_edge;
    CW_half_edge->prev = incident_half_edge.twin;

    incident_half_edge.prev = CCW_half_edge->twin;
    CCW_half_edge->twin->next = &incident_half_edge;
}

void DCEL::DCEL_Vertex::find_adjacent_half_edges(const DCEL::DCEL_Half_Edge& query_edge, DCEL::DCEL_Half_Edge& starting_half_edge) const
{
    DCEL_Half_Edge* CW_half_edge = nullptr; //Clockwise half-edge
    DCEL_Half_Edge* CCW_half_edge = nullptr; //Counter clockwise half-edge

    find_adjacent_half_edges(query_edge, starting_half_edge, CW_half_edge, CCW_half_edge);
}

void DCEL::DCEL_Vertex::find_adjacent_half_edges(const DCEL::DCEL_Half_Edge& query_edge, DCEL::DCEL_Half_Edge& starting_half_edge, DCEL::DCEL_Half_Edge*& CW_half_edge, DCEL::DCEL_Half_Edge*& CCW_half_edge) const
{
    DCEL::DCEL_Half_Edge* prev_half_edge = starting_half_edge.twin->next;
    DCEL::DCEL_Half_Edge* current_half_edge = &starting_half_edge;

    //Keep rotating counterclockwise until we find the first half-edges clock and counterclockwise from the queried half-edge

    Float prev_angle = Vec2::order_around_center(this->position, query_edge.target()->position, prev_half_edge->target()->position);

    do
    {
        Float new_angle = Vec2::order_around_center(this->position, query_edge.target()->position, current_half_edge->target()->position);

        //Keep rotating until the current half-edge has a lower counterclockwise angle than the previous, relative to the queried half-edge
        //(this means we passed the queried half-edges angle)
        if (new_angle < prev_angle)
        {
            CW_half_edge = prev_half_edge;
            CCW_half_edge = current_half_edge;
            return;
        }
        else
        {
            prev_half_edge = current_half_edge;
            prev_angle = new_angle;
            current_half_edge = current_half_edge->prev->twin;
        }
    } while (current_half_edge != &starting_half_edge);

    //If there is only one edge the above loop fails, assign to both CW and CCW
    if (current_half_edge == prev_half_edge)
    {
        CW_half_edge = current_half_edge;
        CCW_half_edge = prev_half_edge;
        return;
    }

    assert(false);
}

void DCEL::DCEL_Vertex::set_all_origins_to_this()
{
    std::vector<DCEL_Half_Edge*> incident_half_edges = get_incident_half_edges();
    for (auto& incident_he : incident_half_edges)
    {
        incident_he->origin = this;
    }
}

//Returns if the half-edges origin lies to the right of its destination, 
//if they share an x axis it returns true if its origin is below its destination.
bool DCEL::DCEL_Half_Edge::is_orientated_top_left() const
{
    return orientation_top_left(origin->position, twin->origin->position);
}

void DCEL::DCEL_Half_Edge::remove_from_cycle()
{
    twin->next->prev = prev;
    prev->next = twin->next;

    //If this half-edge was the first incident half-edge we need to assign it to an arbitrary one in the cycle
    if (origin->incident_half_edge == this)
    {
        if (next == this)
        {
            //If this half-edge was the only half-edge in the cycle, make this vertex a free vertex
            origin->incident_half_edge = nullptr;
        }
        else
        {
            origin->incident_half_edge = next;
        }
    }

    origin = nullptr;
}

//Returns the cycle of half-edges pointing each other, starting with this
std::vector<const DCEL::DCEL_Half_Edge*> DCEL::DCEL_Half_Edge::get_cycle() const
{
    std::vector<const DCEL_Half_Edge*> half_edge_cycle;

    const DCEL_Half_Edge* current_half_edge = this;
    do
    {
        assert(current_half_edge->next != nullptr); //Half-edge cycle is broken

        half_edge_cycle.push_back(current_half_edge);
        current_half_edge = current_half_edge->next;

    } while (current_half_edge != this);


    return half_edge_cycle;
}

//Returns the cycle of half-edges pointing each other, starting with this
std::vector<DCEL::DCEL_Half_Edge*> DCEL::DCEL_Half_Edge::get_cycle()
{
    std::vector<DCEL_Half_Edge*> half_edge_cycle;

    DCEL_Half_Edge* current_half_edge = this;
    do
    {
        assert(current_half_edge->next != nullptr); //Half-edge cycle is broken

        half_edge_cycle.push_back(current_half_edge);
        current_half_edge = current_half_edge->next;

    } while (current_half_edge != this);


    return half_edge_cycle;
}

Float DCEL::DCEL_Overlay_Edge_Wrapper::y_intersect(Float y) const
{
    return edge_segment.y_intersect(y);
}

const Vec2* DCEL::DCEL_Overlay_Edge_Wrapper::get_top_point() const
{
    return edge_segment.get_top_point();
}

const Vec2* DCEL::DCEL_Overlay_Edge_Wrapper::get_bottom_point() const
{
    return edge_segment.get_bottom_point();
}

const Vec2* DCEL::DCEL_Overlay_Edge_Wrapper::get_left_point() const
{
    return edge_segment.get_left_point();
}

const Vec2* DCEL::DCEL_Overlay_Edge_Wrapper::get_right_point() const
{
    return edge_segment.get_right_point();
}

DCEL::DCEL_Vertex* DCEL::DCEL_Overlay_Edge_Wrapper::get_top_dcel_vertex()
{
    const Vec2* top_point = edge_segment.get_top_point();

    if (underlying_half_edge->origin->position == *top_point)
    {
        return underlying_half_edge->origin;
    }
    else
    {
        assert(underlying_half_edge->target()->position == *edge_segment.get_top_point());
        return underlying_half_edge->target();
    }
}

DCEL::DCEL_Vertex* DCEL::DCEL_Overlay_Edge_Wrapper::get_bottom_dcel_vertex()
{
    const Vec2* bottom_point = edge_segment.get_bottom_point();

    if (underlying_half_edge->origin->position == *bottom_point)
    {
        return underlying_half_edge->origin;
    }
    else
    {
        assert(underlying_half_edge->target()->position == *edge_segment.get_bottom_point());
        return underlying_half_edge->target();
    }
}

Segment::Intersection_Type DCEL::DCEL_Overlay_Edge_Wrapper::intersects(const DCEL_Overlay_Edge_Wrapper& other, Vec2& intersection_point) const
{
    return edge_segment.intersects(other.edge_segment, intersection_point);
}

DCEL::DCEL_Vertex* DCEL::DCEL_Overlay_Edge_Wrapper::get_vertex_on_point(const Vec2& point)
{
    if (underlying_half_edge->origin->position == point)
    {
        return underlying_half_edge->origin;
    }
    else
    {
        return underlying_half_edge->target();
    }
}

bool collinear_overlap(const DCEL::DCEL_Overlay_Edge_Wrapper& segment1, const DCEL::DCEL_Overlay_Edge_Wrapper& segment2, Vec2& overlap_start, Vec2& overlap_end)
{
    return collinear_overlap(segment1.edge_segment, segment2.edge_segment, overlap_start, overlap_end);
}


void DCEL::Overlay_Handler::resolve_edge_intersections()
{
    //Find all edge intersections and for each intersection call appropriate overlay helper function

    namespace Sweep = Segment_Intersection_Sweep_Line;

    Sweep::Event_Queue event_queue;

    for (int i = 0; i < DCEL_edges.size(); i++)
    {
        auto event_pair = event_queue.try_emplace(*DCEL_edges.at(i).get_top_point());
        event_pair.first->second.push_back(i);

        event_pair = event_queue.try_emplace(*DCEL_edges.at(i).get_bottom_point());
    }

    //Initialize status structure with the highest event point
    Sweep::Sweep_Line_Status_structure<DCEL_Overlay_Edge_Wrapper> status_structure(event_queue.begin()->first.y);

    while (!event_queue.empty())
    {
        const Vec2 event_point = event_queue.begin()->first;

        auto overlay_event_handler = [this, &event_point](std::vector<int>& intersecting_segments, std::vector<int> top_segments)
            {
                handle_overlay_event(event_point, intersecting_segments, top_segments);
            };


        Handle_Event(status_structure, event_queue, DCEL_edges, event_point, event_queue.begin()->second, overlay_event_handler);



        event_queue.erase(event_queue.begin());
    }
}

void DCEL::Overlay_Handler::handle_overlay_event(const Vec2& event_point, std::vector<int>& intersecting_segments, std::vector<int>& top_segments)
{

    if (intersecting_segments.empty() && top_segments.empty())
    {
        return;
    }


    if (!overlay_event_contains_both_dcels(intersecting_segments, top_segments))
    {
        //If this event point is unique to the overlayed DCEL and doesn't intersect the original DCEL
        //we still need to copy it over into the original DCEL and re-assign its incident edges.
        //Note that this can never be an inner intersection point.
        overlay_handle_unique_vertex(intersecting_segments, top_segments, event_point);

        return;
    }

    //We are dealing with points from both dcels, we need to handle this by merging them

    DCEL_Vertex* vertex_at_event_point = nullptr;
    DCEL_Vertex* overlay_vertex_at_event_point = nullptr;

    overlay_create_intersection_info(intersecting_segments, top_segments, event_point);



    //TODO: Handle collinear - Compare tops with inners only if both dcels - do before other overlays - slope (use pseudo angle)?

    //Before we handle the other overlay cases we need to handle potential collinear segments.
    // ----
    //We only need to consider (parts of) segments below (or directly right of) the event point
    //because we handled the (parts of) segments above the event earlier.
    //Note: Each segment can be part of at most one collinear overlap per event point.

    //Test for collinear overlaps
    //Orientation of DCEL_Edges is top and left

    //Test if both top points or top point and inner segment.
    //Both top:
    //          Fuse both tops (don't forget to update top event, which is current, move one to later?)
    //          If: same bottoms - fuse
    //          Else: Split at first bottom
    //
    //Top and inner:
    //          (DONT CUT OVER THE LOWER EVENT POINT YET, IF OF OVERLAYING DCEL)
    //          Create middle point (if from overlaying, DONT CUT OVERLAYING /\)
    //          


    for (size_t top_segment = 0; top_segment < top_segments.size(); top_segment++)
    {
        Float top_angle = Vec2::pseudo_angle(event_point, *DCEL_edges[top_segment].get_bottom_point());

        for (size_t other_top_segment = 0; other_top_segment < top_segments.size(); other_top_segment++)
        {
            if (top_segment == other_top_segment)
            {
                continue;
            }

            Float other_top_angle = Vec2::pseudo_angle(event_point, *DCEL_edges[other_top_segment].get_bottom_point());

            if (top_angle == other_top_angle)
            {
                //Handle collinear overlap

                //Both top equal
                //If both bottom equal, fuse data, remove?
                //Else split at midpoint.


                //Return here
            }
        }

        for (size_t inner_segment = 0; inner_segment < inner_segments.size(); inner_segment++)
        {
            Float inner_angle = Vec2::pseudo_angle(event_point, *DCEL_edges[inner_segment].get_bottom_point());

            if (top_angle == inner_angle)
            {
                //Handle collinear overlap

                //Remove inner_segment from inner_segments
                //If new top, add new top (middle lower endpoint)

                //Return here
            }
        }
    }


    //----

    auto inner_it = inner_segments.begin();

    //Get or create the DCEL_Vertex at this event point so we can correct the records around it.
    if (bottom_segments.empty() && top_segments.empty())
    {
        //No bottom or top points at this event means we have to create a new DCEL_vertex using the first two segments.
        DCEL_Half_Edge* first_intersecting_half_edge = DCEL_edges[*inner_it].underlying_half_edge;
        ++inner_it;
        DCEL_Half_Edge* second_intersecting_half_edge = DCEL_edges[*inner_it].underlying_half_edge;
        ++inner_it;

        vertex_at_event_point = overlay_edge_on_edge(first_intersecting_half_edge, second_intersecting_half_edge, event_point);
    }
    else
    {
        //If this event contains a top or bottom point, just retrieve the DCEL_vertex at these points.
        get_vertices_from_top_segments(vertex_at_event_point, overlay_vertex_at_event_point);

        if (!vertex_at_event_point || !overlay_vertex_at_event_point)
        {
            get_vertices_from_bottom_segments(vertex_at_event_point, overlay_vertex_at_event_point);
        }

        //If this overlay only has a dcel_vertex from the overlaying dcel we need to copy it over
        if (!vertex_at_event_point && overlay_vertex_at_event_point)
        {
            vertex_at_event_point = overlay_copy_vertex_into_dcel(overlay_vertex_at_event_point);
        }
    }

    assert(vertex_at_event_point != nullptr);

    //If there are more internal intersecting segments we can add them one by one with overlay_edge_on_vertex.
    for (; inner_it != inner_segments.end(); ++inner_it)
    {
        DCEL_Half_Edge* intersecting_half_edge = DCEL_edges[*inner_it].underlying_half_edge;
        overlay_edge_on_vertex(intersecting_half_edge, vertex_at_event_point);
    }

    //The overlay_vertex_on_vertex function fuses the records of both dcel_vertices so we just need to call it once with the two different vertices,
    if (vertex_at_event_point && overlay_vertex_at_event_point)
    {
        overlay_vertex_on_vertex(vertex_at_event_point, overlay_vertex_at_event_point);
    }

}

void DCEL::Overlay_Handler::overlay_collinear_overlap(const int original_edge_index, const int overlay_edge_index, const Vec2& intersection_point)
{
    DCEL_Half_Edge* original_edge = original_dcel.half_edges[original_edge_index].get();
    DCEL_Half_Edge* overlay_edge = original_dcel.half_edges[overlay_edge_index].get();

    //Orientation of DCEL_Edges is top and left

    //Test if both top points or top point and inner segment.
    //Both top:
    //          Fuse both tops (don't forget to update top event, which is current, move one to later?)
    //          If: same bottoms - fuse
    //          Else: Split at first bottom
    //
    //Top and inner:
    //          (DONT CUT OVER THE LOWER EVENT POINT YET, IF OF OVERLAYING DCEL)
    //          Create middle point (if from overlaying, DONT CUT OVERLAYING /\)
    //          
    if (original_edge->target() == overlay_edge->target())
    {
        if (original_edge->origin == overlay_edge->origin)
        {
            //Perfect overlap, fuse

            overlay_collinear_overlap_both_endpoints(overlay_edge, original_edge, original_edge_index, overlay_edge_index);

            //Remove both from top list
        }
        else
        {
            //Same top points
            overlay_collinear_overlap_same_endpoint(original_edge, overlay_edge);

            //Remove longer from top list and add to event point at new top (shorters bottom)
        }
    }
    else
    {
        if (original_edge->origin == overlay_edge->origin)
        {
            //Same bottom points
            overlay_collinear_overlap_same_endpoint(original_edge, overlay_edge);

            //Remove longer from inner list
        }
        else
        {
            //No endpoints equal
            overlay_collinear_overlap_partial_or_embedded(original_edge, overlay_edge);

            //Note:
            //New edge above sweepline is already handled
            //Vertices get copied or fused later.

            //Remove Top from top list and add to top event point at bottom of embedded
            //Embedded can stay in top list.
        }
    }
}

void DCEL::Overlay_Handler::overlay_collinear_overlap_both_endpoints(DCEL::DCEL_Half_Edge* overlay_edge, DCEL::DCEL_Half_Edge* original_edge, const int original_edge_index, const int overlay_edge_index)
{
    //We can reuse the twins of both half-edges to form the new edge
    //This works because of the assumption that both dcels were properly formed and had no overlaps
    // If this is the case both edges have no other edges intersecting them so we never need the indices again.
    // If one of the edges would have an intersecting edge it would've been split at an earlier overlap and we wouldn't detect a perfect overlap now.
    //We can also already handle the bottom vertex. 
    //Because the original half-edges won't be involved in any future intersections we can safely mark them for deletion.


    //Remove the overlay_edge from the vertices in the overlaying dcel so we dont try to add them again at another event type
    overlay_edge->remove_from_cycle();
    overlay_edge->twin->remove_from_cycle();

    //Make the overlayed edges twin the new main edge (it points up left after this)
    DCEL_Half_Edge* new_original_half_edge = overlay_edge->twin;
    *new_original_half_edge = *original_edge;

    //Set the adjacent pointers
    original_edge->twin->twin = new_original_half_edge;
    original_edge->next->prev = new_original_half_edge;
    original_edge->prev->next = new_original_half_edge;

    //Delete both half-edges, we re-use the twins of each edge to form the new edge
    to_delete.push_back(original_edge_index);
    to_delete.push_back(overlay_edge_index);
}

void DCEL::Overlay_Handler::overlay_collinear_overlap_partial_or_embedded(DCEL::DCEL_Half_Edge* original_edge, DCEL::DCEL_Half_Edge* overlay_edge)
{
    //Handle partial overlay (no endpoints overlap)
    DCEL_Half_Edge* top_edge = nullptr;
    DCEL_Half_Edge* lower_edge = nullptr;

    //Which edge lies higher based on the ordering of the top endpoints
    if (original_edge->target()->position.y >= overlay_edge->target()->position.y && original_edge->target()->position.x < overlay_edge->target()->position.x)
    {
        top_edge = original_edge;
        lower_edge = overlay_edge;
    }
    else
    {
        //Should never reach if called with (top, internal)?
        top_edge = overlay_edge;
        lower_edge = original_edge;
    }

    DCEL_Vertex* middle_top_vertex = lower_edge->target();
    DCEL_Vertex* middle_bottom_vertex = nullptr;

    DCEL_Half_Edge* bottom_edge = nullptr;

    //Determine the new middle and bottom half-edges based on the ordering of the bottom endpoints
    if (lower_edge->origin->position.y >= top_edge->origin->position.y && lower_edge->origin->position.x < top_edge->origin->position.x)
    {
        //bottom is fully embedded
        //middle_edge = lower_edge;
        bottom_edge = top_edge;
        middle_bottom_vertex = bottom_edge->origin;
    }
    else
    {
        //Both segments partially overlap
        DCEL_Half_Edge* middle_edge = top_edge;
        bottom_edge = lower_edge;
        middle_bottom_vertex = top_edge->origin;

        //If partial, shorten top to the top endpoint of the bottom edge
        middle_edge->twin = bottom_edge->twin;

        //If partial, Shorten bottom twin to the bottom endpoint of the top edge
        bottom_edge->twin->twin = middle_edge;

    }

    //This operation creates two new half edges (we split and combine the two edges into three)
    original_dcel.half_edges.reserve(original_dcel.half_edges.size() + 2);


    const std::unique_ptr<DCEL_Half_Edge>& new_top_edge = original_dcel.half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>(
        middle_top_vertex,     //Origin
        nullptr,               //Face
        top_edge->twin,        //Twin
        top_edge->next,        //Next
        nullptr)); //Prev is on the cycle of the bottom edge, top endpoint

    top_edge->next->prev = new_top_edge.get();

    const std::unique_ptr<DCEL_Half_Edge>& new_bottom_twin_edge = original_dcel.half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>(
        middle_bottom_vertex,    //Origin
        nullptr,                 //Face
        bottom_edge,             //Twin
        bottom_edge->twin->next, //Next
        nullptr)); //Prev is on the cycle of the top edge, bottom endpoint

    bottom_edge->twin->next->prev = new_bottom_twin_edge.get();

    //Shorten top twin to the upper middle endpoint
    top_edge->twin->twin = new_top_edge.get();

    //Shorten bottom to the lower middle endpoint
    bottom_edge->twin = new_bottom_twin_edge.get();

    //Add the new top half-edge to the cycle around top vertex of the bottom edge
    original_dcel.add_edge_to_vertex(*new_top_edge.get(), *new_top_edge->origin);

    //Add the new twin of the bottom half-edge to the bottom vertex of the top edge
    original_dcel.add_edge_to_vertex(*new_bottom_twin_edge.get(), *new_bottom_twin_edge->origin);
}

void DCEL::Overlay_Handler::overlay_collinear_overlap_same_endpoint(DCEL_Half_Edge* original_edge, DCEL_Half_Edge* overlay_edge)
{
    Float original_edge_sqrd_length = (original_edge->target()->position - original_edge->origin->position).squared_length();
    Float overlay_edge_sqrd_length = (overlay_edge->target()->position - overlay_edge->origin->position).squared_length();

    //First check if the same endpoint is at the top or bottom
    //(We assume the given half edges are pointing up, or to the left if horizontal)
    if (original_edge->target() == overlay_edge->target())
    {
        //Top vertices overlap
        if (original_edge_sqrd_length > overlay_edge_sqrd_length)
        {
            //Original is longer, remove originals twin from top vertex and add to middle vertex (shortens original_edge)
            original_edge->twin->remove_from_cycle();
            original_dcel.add_edge_to_vertex(*original_edge->twin, *overlay_edge->origin);
        }
        else
        {
            //Overlay is longer, remove overlays twin from top vertex and add to middle vertex (shortens overlay_edge)
            overlay_edge->twin->remove_from_cycle();
            original_dcel.add_edge_to_vertex(*overlay_edge->twin, *original_edge->origin);
        }
    }
    else
    {
        //Bottom vertices overlap
        if (original_edge_sqrd_length > overlay_edge_sqrd_length)
        {
            //Original is longer, remove original from bottom vertex and add to middle vertex (shortens original_edge)
            original_edge->remove_from_cycle();
            original_dcel.add_edge_to_vertex(*original_edge, *overlay_edge->origin);
        }
        else
        {
            //Overlay is longer, remove overlay from bottom vertex and add to middle vertex (shortens overlay_edge)
            overlay_edge->remove_from_cycle();
            original_dcel.add_edge_to_vertex(*overlay_edge, *original_edge->origin);
        }
    }
}

std::vector<Vec2> DCEL::DCEL_Face::get_vertices() const
{
    const DCEL_Half_Edge* starting_half_edge = outer_component;
    const DCEL_Half_Edge* current_half_edge = starting_half_edge;

    std::vector<Vec2> boundary_vertices;
    do
    {
        //Report vertex and move to the next half-edge in the chain
        boundary_vertices.push_back(current_half_edge->origin->position);
        current_half_edge = current_half_edge->next;

    } while (current_half_edge != starting_half_edge);

    return boundary_vertices;
}

std::vector<const DCEL::DCEL_Half_Edge*> DCEL::DCEL_Vertex::get_incident_half_edges() const
{
    const DCEL::DCEL_Half_Edge* starting_half_edge = incident_half_edge;
    DCEL::DCEL_Half_Edge* current_half_edge = incident_half_edge;

    std::vector<const DCEL_Half_Edge*> incident_half_edges;

    do
    {
        incident_half_edges.emplace_back(current_half_edge);
        current_half_edge = current_half_edge->twin->next;

    } while (current_half_edge != starting_half_edge);

    return incident_half_edges;
}

std::vector<DCEL::DCEL_Half_Edge*> DCEL::DCEL_Vertex::get_incident_half_edges()
{
    const DCEL::DCEL_Half_Edge* starting_half_edge = incident_half_edge;
    DCEL::DCEL_Half_Edge* current_half_edge = incident_half_edge;

    std::vector<DCEL_Half_Edge*> incident_half_edges;

    do
    {
        incident_half_edges.emplace_back(current_half_edge);
        current_half_edge = current_half_edge->twin->next;

    } while (current_half_edge != starting_half_edge);

    return incident_half_edges;
}

void DCEL::Overlay_Handler::overlay_handle_unique_vertex(std::vector<int>& intersecting_segments, std::vector<int>& top_segments, const Vec2& event_point)
{
    //If the vertex is unique to the overlaying DCEL, copy it over, otherwise, do nothing

    const DCEL_Vertex* old_vertex = nullptr;
    if (!top_segments.empty())
    {
        if (DCEL_edges[top_segments.front()].original_dcel)
        {
            //Only the original dcel contributes, no records need to be updated.
            return;
        }

        old_vertex = DCEL_edges[top_segments.front()].get_top_dcel_vertex();
    }
    else
    {
        if (DCEL_edges[intersecting_segments.front()].original_dcel)
        {
            //Only the original dcel contributes, no records need to be updated.
            return;
        }

        old_vertex = DCEL_edges[intersecting_segments.front()].get_vertex_on_point(event_point);

    }

    assert(old_vertex->position == event_point);

    overlay_copy_vertex_into_dcel(old_vertex);
}

//Checks if the given event contains elements from multiple DCELs
bool DCEL::Overlay_Handler::overlay_event_contains_both_dcels(const std::vector<int>& intersecting_segments, const std::vector<int>& top_segments) const
{
    if (intersecting_segments.size() + top_segments.size() <= 1)
    {
        return false;
    }

    auto dcel_origin_comparer = [this](int l, int r)
        {
            return DCEL_edges[l].original_dcel == DCEL_edges[r].original_dcel;
        };

    bool both_dcels = false;

    //First check if all intersecting_segments originate from the same dcel
    if (intersecting_segments.size() > 1)
    {
        both_dcels = !std::equal(intersecting_segments.begin() + 1, intersecting_segments.end(), intersecting_segments.begin(), dcel_origin_comparer);

        if (both_dcels)
        {
            return true;
        }
    }

    //Then check if all top_segments originate from the same dcel
    if (top_segments.size() > 1)
    {
        both_dcels = !std::equal(top_segments.begin() + 1, top_segments.end(), top_segments.begin(), dcel_origin_comparer);

        if (both_dcels)
        {
            return true;
        }
    }

    //Finally if both lists contain the same values, check if these same values differ between the two lists
    if (!intersecting_segments.empty() && !top_segments.empty())
    {
        return DCEL_edges[intersecting_segments.front()].original_dcel != DCEL_edges[top_segments.front()].original_dcel;
    }

    return false;
}


//Splits the overlaying_segments based on the location of the intersection point on each segment
void DCEL::Overlay_Handler::overlay_create_intersection_info(
    const std::vector<int>& new_intersecting_segments,
    const std::vector<int>& new_top_segments,
    const Vec2& event_point)
{
    this->top_segments = new_top_segments;

    for (int i : new_intersecting_segments)
    {
        if (*(DCEL_edges[i].get_bottom_point()) == event_point)
        {
            this->bottom_segments.push_back(i);
        }
        else
        {
            this->inner_segments.push_back(i);
        }
    }
}

DCEL::DCEL_Vertex* DCEL::Overlay_Handler::overlay_copy_vertex_into_dcel(const DCEL_Vertex* vertex)
{
    const std::unique_ptr<DCEL_Vertex>& new_vertex = original_dcel.vertices.emplace_back(std::make_unique<DCEL_Vertex>(*vertex));
    new_vertex->set_all_origins_to_this();

    return new_vertex.get();
}

void DCEL::Overlay_Handler::get_vertices_from_top_segments(DCEL_Vertex*& original_vertex, DCEL_Vertex*& overlay_vertex)
{
    for (auto const& i : top_segments)
    {
        if (DCEL_edges[i].original_dcel)
        {
            original_vertex = DCEL_edges[i].get_top_dcel_vertex();
        }
        else
        {
            overlay_vertex = DCEL_edges[i].get_top_dcel_vertex();
        }

        if (original_vertex && overlay_vertex)
        {
            break;
        }
    }
}

void DCEL::Overlay_Handler::get_vertices_from_bottom_segments(DCEL_Vertex*& original_vertex, DCEL_Vertex*& overlay_vertex)
{
    for (auto const& i : bottom_segments)
    {
        if (DCEL_edges[i].original_dcel)
        {
            original_vertex = DCEL_edges[i].get_bottom_dcel_vertex();
        }
        else
        {
            overlay_vertex = DCEL_edges[i].get_bottom_dcel_vertex();
        }

        if (original_vertex && overlay_vertex)
        {
            break;
        }
    }
}

DCEL::Overlay_Handler::Overlay_Handler(DCEL& original_dcel, DCEL& overlaying_dcel) : original_dcel(original_dcel), overlaying_dcel(overlaying_dcel)
{
    size_t original_half_edge_count = original_dcel.half_edge_count();

    //Move all the half edges from the other dcel into this one.
    original_dcel.half_edges.insert(original_dcel.half_edges.end(),
        std::make_move_iterator(overlaying_dcel.half_edges.begin()),
        std::make_move_iterator(overlaying_dcel.half_edges.end()));


    ////Construct the edges representing the half edge twins for use in the sweep line algorithm.
    DCEL_edges.reserve(original_dcel.half_edge_count() / 2);

    for (size_t i = 0; i < original_dcel.half_edge_count(); i++)
    {
        if (original_dcel.half_edges[i]->is_orientated_top_left())
        {
            bool is_original_half_edge = i < original_half_edge_count;
            DCEL_edges.emplace_back(original_dcel.half_edges[i].get(), is_original_half_edge);
        }
    }

    //Call the sweep line algorithm handling overlay cases from top to bottom.
    resolve_edge_intersections();

    overlaying_dcel.clear();
}
