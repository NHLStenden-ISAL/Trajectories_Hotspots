#include "pch.h"
#include "dcel.h"


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

DCEL::DCEL_Vertex* DCEL::get_vertex_at_position(const Vec2& position) const
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

void DCEL::Overlay_Handler::overlay_vertex_on_vertex(DCEL_Vertex* original_vertex, DCEL_Vertex* overlay_vertex) const
{
    //Gather half-edges around second vertex
    //For each: Set origins to first vertex
    //          and switch prev, prev->next,
    //          twin->next, and twin->next->prev pointers

    std::vector<DCEL_Half_Edge*> incident_half_edges_v2 = overlay_vertex->get_incident_half_edges();

    for (DCEL_Half_Edge* incident_half_edge_v2 : incident_half_edges_v2)
    {
        original_dcel.add_edge_to_vertex(*incident_half_edge_v2, *original_vertex);
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

    if (vertex.incident_half_edge == nullptr)
    {
        vertex.incident_half_edge = &incident_half_edge;
        CW_half_edge = &incident_half_edge;
        CCW_half_edge = &incident_half_edge;
    }
    else
    {
        vertex.find_adjacent_half_edges(incident_half_edge, current_half_edge, CW_half_edge, CCW_half_edge);
    }

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
        if (twin->next == this)
        {
            //If this half-edge was the only half-edge in the cycle, make this vertex a free vertex
            origin->incident_half_edge = nullptr;
        }
        else
        {
            origin->incident_half_edge = twin->next;
        }
    }

    prev = twin;
    twin->next = this;
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


    for (int i = 0; i < DCEL_edges.size(); i++)
    {
        auto event_pair = event_queue.try_emplace(*DCEL_edges.at(i).get_top_point());
        event_pair.first->second.push_back(i);

        event_pair = event_queue.try_emplace(*DCEL_edges.at(i).get_bottom_point());
    }

    //Initialize status structure with the highest event point
    namespace Sweep = Segment_Intersection_Sweep_Line;
    Sweep::Sweep_Line_Status_structure<DCEL_Overlay_Edge_Wrapper> status_structure(event_queue.begin()->first.y);

    while (!event_queue.empty())
    {
        const Vec2 event_point = event_queue.begin()->first;

        auto overlay_event_handler = [this, &event_point](std::vector<int>& new_intersecting_segments, std::vector<int>& new_top_segments)
            {
                handle_overlay_event(event_point, new_intersecting_segments, new_top_segments);

                if (inner_segments_updated)
                {
                    new_intersecting_segments = inner_segments;
                    inner_segments_updated = false;
                }

                if (top_segments_updated)
                {
                    new_top_segments = top_segments;
                    top_segments_updated = false;
                }
            };


        Handle_Event(status_structure, event_queue, DCEL_edges, event_point, event_queue.begin()->second, overlay_event_handler);

        event_queue.erase(event_queue.begin());
    }

    //In a rare case we register half-edges for deletion, we can handle those now
    delete_registered_half_edges();
}

void DCEL::Overlay_Handler::handle_overlay_event(const Vec2& event_point, std::vector<int>& new_intersecting_segments, std::vector<int>& new_top_segments)
{

    if (new_intersecting_segments.empty() && new_top_segments.empty())
    {
        return;
    }

    if (!overlay_event_contains_both_dcels(new_intersecting_segments, new_top_segments))
    {
        //If this event point is unique to the overlayed DCEL and doesn't intersect the original DCEL
        //we still need to copy it over into the original DCEL and re-assign its incident edges.
        //Note that this can never be an inner intersection point.
        overlay_handle_unique_vertex(new_intersecting_segments, new_top_segments, event_point);

        return;
    }

    //We are dealing with points from both dcels, we need to handle this by merging them

    DCEL_Vertex* vertex_at_event_point = nullptr;
    DCEL_Vertex* overlay_vertex_at_event_point = nullptr;

    overlay_create_intersection_info(new_intersecting_segments, new_top_segments, event_point);

    //Handle any potential collinear overlaps, these resolve the other cases as well so we can return
    if (overlay_handle_collinear_overlaps(event_point))
    {
        return;
    }

    //Now process the other events
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

bool DCEL::Overlay_Handler::overlay_handle_collinear_overlaps(const Vec2& event_point)
{
    // ----
    //We only need to consider (parts of) segments below (or directly right of) the event point
    //because we handled the (parts of) segments above the event earlier.
    //Note: Each segment can be part of at most one collinear overlap per event point.

    //Test for collinear overlaps
    //Orientation of DCEL_Edges is top and left

    //We only need to compare the tops with the other tops and tops with inners.
    //Collinear overlaps with two inners should've been handled at the lowest top endpoint.

    //A dcel can only have one inner segment at this event point (they will have split in an earlier overlay)
    //The DCEL with the inner segment will have no tops here (again, the inner would have split already)
    //Also, the DCEL with tops can't have collinear segments (also already handeled in an earlier overlay)
    //This means that if we find a collinear overlap with an inner and a top segment there can be no more collinear overlaps and we can return.
    for (size_t inner_segment_index = 0; inner_segment_index < inner_segments.size(); inner_segment_index++)
    {
        int inner_segment = inner_segments[inner_segment_index];
        Float inner_angle = Vec2::pseudo_angle(event_point, *DCEL_edges[inner_segment].get_bottom_point());

        for (size_t top_segment_index = 0; top_segment_index < top_segments.size(); top_segment_index++)
        {
            int top_segment = top_segments[top_segment_index];
            Float top_angle = Vec2::pseudo_angle(event_point, *DCEL_edges[top_segment].get_bottom_point());

            if (top_angle == inner_angle)
            {
                if (DCEL_edges[top_segment].underlying_half_edge->origin->position == DCEL_edges[inner_segment].underlying_half_edge->origin->position)
                {
                    //Collinear overlap with bottom endpoints overlapping
                    assert(DCEL_edges[top_segment].original_dcel != DCEL_edges[inner_segment].original_dcel);

                    const int original_index = DCEL_edges[top_segment].original_dcel ? top_segment : inner_segment;
                    const int overlay_index = DCEL_edges[top_segment].original_dcel ? inner_segment : top_segment;

                    DCEL_Half_Edge* original_half_edge = DCEL_edges[original_index].underlying_half_edge;
                    DCEL_Half_Edge* overlay_half_edge = DCEL_edges[overlay_index].underlying_half_edge;

                    overlay_collinear_overlap_bottom_endpoint(original_half_edge, overlay_half_edge);

                    //If original longer, remove original from inner, add original to top, and remove overlay from top
                    //If original shorter and remove overlay from inner

                    inner_segments.erase(inner_segments.begin() + inner_segment_index);
                    inner_segment_index--;
                    inner_segments_updated = true;

                    if (DCEL_edges[inner_segment].original_dcel)
                    {
                        //Remove overlay from top list, it was shortened/moved to A-B above the sweepline
                        top_segments.erase(top_segments.begin() + top_segment_index);
                        top_segment_index--;
                        top_segments_updated = true;

                        top_segments.push_back(original_index);
                    }
                }
                else
                {
                    bool top_is_embedded = overlay_collinear_overlap_partial_or_embedded(top_segment, inner_segment);

                    if (top_is_embedded)
                    {
                        DCEL_edges[inner_segment].edge_segment.start = DCEL_edges[inner_segment].underlying_half_edge->origin->position;
                        DCEL_edges[inner_segment].edge_segment.end = DCEL_edges[inner_segment].underlying_half_edge->target()->position;

                        //Top stays in the top list, its the middle of the (now) three segments
                        //Inner becomes a top in the future, remove from inner and add to top event point at bottom of embedded segment
                        inner_segments.erase(inner_segments.begin() + inner_segment_index);
                        event_queue[DCEL_edges[top_segment].underlying_half_edge->origin->position].push_back(inner_segment);
                        inner_segments_updated = true;
                    }
                    else
                    {
                        //Top becomes a top in the future (at the 3rd vertex)
                        top_segments.erase(top_segments.begin() + top_segment_index);

                        event_queue[DCEL_edges[inner_segment].underlying_half_edge->origin->position].push_back(top_segment);
                        top_segment_index--;

                        DCEL_edges[inner_segment].edge_segment.start = DCEL_edges[inner_segment].underlying_half_edge->origin->position;
                        DCEL_edges[inner_segment].edge_segment.end = DCEL_edges[inner_segment].underlying_half_edge->target()->position;

                        DCEL_edges[top_segment].edge_segment.start = DCEL_edges[top_segment].underlying_half_edge->origin->position;
                        DCEL_edges[top_segment].edge_segment.end = DCEL_edges[top_segment].underlying_half_edge->target()->position;

                        //Inner is now a top at current
                        inner_segments.erase(inner_segments.begin() + inner_segment_index);
                        inner_segments_updated = true;
                        top_segments.push_back(inner_segment);
                        top_segments_updated = true;
                    }
                }

                return true;
            }
        }
    }

    bool top_collinear_found = false;
    for (size_t top_segment_index = 0; top_segment_index < top_segments.size(); top_segment_index++)
    {
        int top_segment = top_segments[top_segment_index];
        Float top_angle = Vec2::pseudo_angle(event_point, *DCEL_edges[top_segment].get_bottom_point());

        for (size_t other_top_segment_index = 0; other_top_segment_index < top_segments.size(); other_top_segment_index++)
        {
            int other_top_segment = top_segments[other_top_segment_index];

            if (top_segment == other_top_segment)
            {
                continue;
            }

            Float other_top_angle = Vec2::pseudo_angle(event_point, *DCEL_edges[other_top_segment].get_bottom_point());

            if (top_angle == other_top_angle)
            {
                //Collinear overlap with both top endpoints overlapping
                //TODO: Only this case can have multiple collinear overlaps.
                //TODO: We can have multiple top overlays, care with vertex_on_vertex call.

                assert(DCEL_edges[top_segment].original_dcel != DCEL_edges[other_top_segment].original_dcel);

                //Check if bottom endpoints also overlap
                if (DCEL_edges[top_segment].underlying_half_edge->origin->position == DCEL_edges[other_top_segment].underlying_half_edge->origin->position)
                {
                    const int original_index = DCEL_edges[top_segment].original_dcel ? top_segment : other_top_segment;
                    const int overlay_index = DCEL_edges[top_segment].original_dcel ? other_top_segment : top_segment;

                    overlay_collinear_overlap_both_endpoints(original_index, overlay_index);

                    //Remove the overlay segment from the top list
                    if (!DCEL_edges[top_segment].original_dcel)
                    {
                        top_segments.erase(top_segments.begin() + top_segment_index);
                        top_segment_index--;
                    }
                    else
                    {
                        top_segments.erase(top_segments.begin() + other_top_segment_index);
                        other_top_segment_index--;
                    }



                    top_segments_updated = true;
                }
                else
                {
                    //Collinear overlap with top endpoints overlapping

                    Float top_edge_sqrd_length = (DCEL_edges[top_segment].underlying_half_edge->target()->position - DCEL_edges[top_segment].underlying_half_edge->origin->position).squared_length();
                    Float other_top_edge_sqrd_length = (DCEL_edges[other_top_segment].underlying_half_edge->target()->position - DCEL_edges[other_top_segment].underlying_half_edge->origin->position).squared_length();

                    int longer_half_edge_index = top_edge_sqrd_length > other_top_edge_sqrd_length ? top_segment : other_top_segment;
                    int shorter_half_edge_index = top_edge_sqrd_length > other_top_edge_sqrd_length ? other_top_segment : top_segment;

                    Vec2 middle_vertex;
                    overlay_collinear_overlap_top_endpoint(longer_half_edge_index, shorter_half_edge_index, middle_vertex);

                    //Remove longer from top list and add to event point at middle (longer was shortened)
                    //There can be no more event on the segment between top and middle but there can be one
                    //on the segment between middle and bottom so we have to add a new top eventpoint at the middle vertex.
                    if (longer_half_edge_index == top_segment_index)
                    {
                        top_segments.erase(top_segments.begin() + top_segment_index);
                        top_segment_index--;

                        event_queue[middle_vertex].push_back(top_segment);
                    }
                    else
                    {
                        top_segments.erase(top_segments.begin() + other_top_segment_index);
                        event_queue[middle_vertex].push_back(other_top_segment);
                    }

                    top_segments_updated = true;
                }

                top_collinear_found = true;
                break;
            }
        }

        if (top_collinear_found)
        {
            //Skip check with inners if we found an overlap with a top, both can't happen at the same time
            continue;
        }


    }

    return top_collinear_found;
}

void DCEL::Overlay_Handler::overlay_collinear_overlap_both_endpoints(const int original_edge_index, const int overlay_edge_index)
{
    //We can reuse the twins of both half-edges to form the new edge
    //This works because of the assumption that both dcels were properly formed and had no overlaps
    // If this is the case both edges have no other edges intersecting them so we never need the indices again.
    // If one of the edges would have an intersecting edge it would've been split at an earlier overlap and we wouldn't detect a perfect overlap now.
    //We can also already handle the bottom vertex. 
    //Because the original half-edges won't be involved in any future intersections we can safely mark them for deletion.

    DCEL_Half_Edge* original_edge = DCEL_edges[original_edge_index].underlying_half_edge;
    DCEL_Half_Edge* overlay_edge = DCEL_edges[overlay_edge_index].underlying_half_edge;

    DCEL_Vertex* original_top_vertex = DCEL_edges[original_edge_index].underlying_half_edge->target();
    DCEL_Vertex* overlay_top_vertex = DCEL_edges[overlay_edge_index].underlying_half_edge->target();

    //Remove the overlay_edge from the vertices in the overlaying dcel so we don't try to add them again at another event type 
    overlay_edge->remove_from_cycle();
    overlay_edge->twin->remove_from_cycle();

    //Remove the original half-edge from the vertex, we will replace it
    DCEL_Vertex* original_edge_origin = original_edge->origin;
    original_edge->remove_from_cycle();

    //Make the overlayed edges twin the new main edge (it points up left after this)
    DCEL_Half_Edge* new_original_half_edge = overlay_edge->twin;
    *new_original_half_edge->twin = *original_edge->twin;

    //Set the adjacent pointers
    original_edge->twin->twin = new_original_half_edge;
    original_edge->next->prev = new_original_half_edge;
    original_edge->prev->next = new_original_half_edge;

    new_original_half_edge->twin = original_edge->twin;
    new_original_half_edge->next = original_edge->next;
    new_original_half_edge->prev = original_edge->prev;

    //Add it to the cycle of the old half-edge
    original_dcel.add_edge_to_vertex(*new_original_half_edge, *original_edge_origin);

    //Delete both half-edges, we re-use the twins of each edge to form the new edge
    to_delete.push_back(DCEL_edges[original_edge_index].new_edge_index);
    to_delete.push_back(DCEL_edges[overlay_edge_index].new_edge_index);

    //Set the underlying pointer to the new half-edge, 
    // we can do this because the index of the to deleted half-edges stay the same
    // and the new half-edge will only be used in a bottom event which has no delete cases.
    // (Note: this feels off, can we make this cleaner?)
    DCEL_edges[original_edge_index].underlying_half_edge = new_original_half_edge;

    //Finish this event point by merging both top vertices into the original vertex
    overlay_vertex_on_vertex(original_top_vertex, overlay_top_vertex);
}

void DCEL::Overlay_Handler::overlay_collinear_overlap_top_endpoint(const int longer_edge_index, const int shorter_edge_index, Vec2& middle_vertex)
{
    DCEL::DCEL_Half_Edge* longer_edge = DCEL_edges[longer_edge_index].underlying_half_edge;
    DCEL::DCEL_Half_Edge* shorter_edge = DCEL_edges[shorter_edge_index].underlying_half_edge;

    DCEL_Vertex* original_vertex_A;
    DCEL_Vertex* overlay_vertex_A;

    if (DCEL_edges[longer_edge_index].original_dcel)
    {
        original_vertex_A = longer_edge->target();
        overlay_vertex_A = shorter_edge->target();
    }
    else
    {
        original_vertex_A = shorter_edge->target();
        overlay_vertex_A = longer_edge->target();
    }

    //Create a new vertex in the dcel of the longer edge, this becomes the top of the new bottom edge
    middle_vertex = shorter_edge->origin->position;
    DCEL_Vertex* new_vertex_B = create_new_vertex(middle_vertex, DCEL_edges[longer_edge_index].original_dcel);

    //remove the twin of the longer edges from top vertex and add to middle vertex (shortens longer edge)
    longer_edge->twin->remove_from_cycle();
    original_dcel.add_edge_to_vertex(*longer_edge->twin, *new_vertex_B);
    DCEL_edges[longer_edge_index].edge_segment.end = middle_vertex;

    ////Finish this event point by merging both top vertices into the original vertex
    overlay_vertex_on_vertex(original_vertex_A, overlay_vertex_A);
}

void DCEL::Overlay_Handler::overlay_collinear_overlap_bottom_endpoint(DCEL_Half_Edge* original_edge, DCEL_Half_Edge* overlay_edge)
{
    Float original_edge_sqrd_length = (original_edge->target()->position - original_edge->origin->position).squared_length();
    Float overlay_edge_sqrd_length = (overlay_edge->target()->position - overlay_edge->origin->position).squared_length();

    DCEL_Vertex* vertex_B = overlay_edge->target();

    //To reduce the complexity of the overlay algorithm we need to make sure the bottom segment is from the original dcel
    //(This is because we want to keep the vertex that originated from the original dcel)
    if (original_edge_sqrd_length > overlay_edge_sqrd_length)
    {
        DCEL_Vertex* vertex_A = original_edge->target();

        original_edge->twin->remove_from_cycle();
        overlay_edge->twin->remove_from_cycle();

        original_dcel.add_edge_to_vertex(*original_edge->twin, *vertex_B);
        original_dcel.add_edge_to_vertex(*overlay_edge->twin, *vertex_A);

        //The middle vertex is unique to the overlay dcel, copy the middle vertex into the original dcel
        vertex_B = overlay_copy_vertex_into_dcel(vertex_B);
    }

    //Overlay is (now) longer, remove overlay from bottom vertex and add to middle vertex (shortens overlay_edge)
    overlay_edge->remove_from_cycle();
    original_dcel.add_edge_to_vertex(*overlay_edge, *vertex_B);
}

bool DCEL::Overlay_Handler::overlay_collinear_overlap_partial_or_embedded(const int top_edge_index, const int inner_edge_index)
{
    //Vertices from top/left to bottom/right: A-B-C-D

    const DCEL_Half_Edge* higher_edge = DCEL_edges[inner_edge_index].underlying_half_edge;
    const DCEL_Half_Edge* lower_edge = DCEL_edges[top_edge_index].underlying_half_edge;

    bool is_embedded;
    if (lower_edge->origin->position.y == higher_edge->origin->position.y)
    {
        if (lower_edge->origin->position.x == higher_edge->origin->position.x)
        {
            is_embedded = false;
        }
        else
        {
            is_embedded = lower_edge->origin->position.x < higher_edge->origin->position.x;
        }
    }
    else
    {
        is_embedded = lower_edge->origin->position.y > higher_edge->origin->position.y;
    }

    //Determine the new middle and bottom half-edges based on the ordering of the bottom endpoints
    if (is_embedded)
    {
        overlay_collinear_overlap_embedded(inner_edge_index, top_edge_index);
    }
    else
    {
        overlay_collinear_overlap_partial(inner_edge_index, top_edge_index);
    }

    return is_embedded;
}

void DCEL::Overlay_Handler::overlay_collinear_overlap_partial(const int higher_edge_index, const int lower_edge_index)
{
    //Vertices from top/left to bottom/right: A-B-C-D
    //DCEL_Vertex* vertex_B = DCEL_edges[lower_edge_index].underlying_half_edge->target();

    //In this case one of the dcels only contributing segment is a singly inner segment.
    //This means we can copy over the vertex if it belongs to the overlay DCEL,
    //split the single edge, and add these to the vertex cycle to complete the whole event point.
    DCEL_Vertex* vertex_B = nullptr;
    if (DCEL_edges[lower_edge_index].original_dcel)
    {
        vertex_B = DCEL_edges[lower_edge_index].underlying_half_edge->target();
    }
    else
    {
        vertex_B = create_new_vertex(DCEL_edges[lower_edge_index].underlying_half_edge->target()->position, true);
        overlay_vertex_on_vertex(vertex_B, DCEL_edges[lower_edge_index].underlying_half_edge->target());
    }

    //Vertex C and D are always in different DCELs, to make the overlay algorithm work at the lower event point 
    //we create a new vertex in the dcel of D and connect the third edge to the new C
    DCEL_Vertex* vertex_C = create_new_vertex(DCEL_edges[higher_edge_index].underlying_half_edge->origin->position, DCEL_edges[lower_edge_index].original_dcel);

    DCEL_Half_Edge* higher_edge = DCEL_edges[higher_edge_index].underlying_half_edge;
    DCEL_Half_Edge* bottom_edge = DCEL_edges[lower_edge_index].underlying_half_edge;

    //This operation creates two new half edges (we split and combine the two edges into three edges)
    original_dcel.half_edges.reserve(original_dcel.half_edges.size() + 2);

    const std::unique_ptr<DCEL_Half_Edge>& new_top_edge = original_dcel.half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>(
        vertex_B,     //Origin set to vertex B
        nullptr,                  //Face
        higher_edge->twin,        //Twin
        higher_edge->next,        //Next
        nullptr)); //Prev is on the cycle of vertex B

    //Replace the old top_edge with the new one in the cycle around A
    higher_edge->next->prev = new_top_edge.get();

    const std::unique_ptr<DCEL_Half_Edge>& new_bottom_twin_edge = original_dcel.half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>(
        vertex_C,    //Origin set to vertex C
        nullptr,                 //Face
        bottom_edge,             //Twin
        bottom_edge->twin->next, //Next
        nullptr)); //Prev is on the cycle vertex C

    //Replace the old bottom twin with the new one in the cycle around D
    bottom_edge->twin->next->prev = new_bottom_twin_edge.get();

    //Shorten top twin to vertex B
    new_top_edge->twin->twin = new_top_edge.get();

    //shorten top to vertex B
    higher_edge->twin = bottom_edge->twin;

    //shorten twin of bottom to vertex C
    bottom_edge->twin->twin = higher_edge;

    //Shorten bottom to vertex C
    bottom_edge->twin = new_bottom_twin_edge.get();

    //Fix the cycle around vertex B by setting the next and prev of the new middle
    higher_edge->next = bottom_edge->next;
    higher_edge->next->prev = higher_edge;

    //Fix the cycle around vertex C by setting the next and prev of the middle twin
    higher_edge->twin->next = new_top_edge->twin->next;
    higher_edge->twin->next->prev = higher_edge->twin;

    //Add the new top half-edge to the cycle around vertex B
    original_dcel.add_edge_to_vertex(*new_top_edge.get(), *new_top_edge->origin);

    //Add the new twin of the bottom half-edge to vertex C
    original_dcel.add_edge_to_vertex(*new_bottom_twin_edge.get(), *vertex_C);
}

void DCEL::Overlay_Handler::overlay_collinear_overlap_embedded(const int higher_edge_index, const int middle_edge_index)
{
    //Vertices from top/left to bottom/right: A-B-C-D

    //In this case one of the dcels only contributing segment is a singly inner segment.
    //This means we can copy over the vertex if it belongs to the overlay DCEL,
    //split the single edge, and add these to the vertex cycle to complete the whole event point.
    DCEL_Vertex* vertex_B = nullptr;
    if (DCEL_edges[middle_edge_index].original_dcel)
    {
        vertex_B = DCEL_edges[middle_edge_index].underlying_half_edge->target();
    }
    else
    {
        vertex_B = create_new_vertex(DCEL_edges[middle_edge_index].underlying_half_edge->target()->position, true);
        overlay_vertex_on_vertex(vertex_B, DCEL_edges[middle_edge_index].underlying_half_edge->target());
    }

    //Vertex C and D are always in different DCELs, to make the overlay algorithm work at the lower event point 
    //we create a new vertex in the dcel of D and connect the third edge to the new C
    DCEL_Vertex* vertex_C = create_new_vertex(DCEL_edges[middle_edge_index].underlying_half_edge->origin->position, DCEL_edges[higher_edge_index].original_dcel);

    DCEL_Half_Edge* higher_edge = DCEL_edges[higher_edge_index].underlying_half_edge;
    DCEL_Half_Edge* bottom_edge = DCEL_edges[higher_edge_index].underlying_half_edge;

    //This operation creates two new half edges (we split and combine the two edges into three edges)
    original_dcel.half_edges.reserve(original_dcel.half_edges.size() + 2);

    const std::unique_ptr<DCEL_Half_Edge>& new_top_edge = original_dcel.half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>(
        vertex_B,     //Origin set to vertex B
        nullptr,                  //Face
        higher_edge->twin,        //Twin
        higher_edge->next,        //Next
        nullptr)); //Prev is on the cycle of vertex B

    //Replace the old top_edge with the new one in the cycle around A
    higher_edge->next->prev = new_top_edge.get();

    const std::unique_ptr<DCEL_Half_Edge>& new_bottom_twin_edge = original_dcel.half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>(
        vertex_C,    //Origin set to vertex C
        nullptr,                 //Face
        bottom_edge,             //Twin
        bottom_edge->twin->next, //Next
        nullptr)); //Prev is on the cycle vertex C

    //Replace the old bottom twin with the new one in the cycle around D
    bottom_edge->twin->next->prev = new_bottom_twin_edge.get();

    //Shorten top twin to vertex B
    new_top_edge->twin->twin = new_top_edge.get();

    //Shorten bottom to vertex C
    bottom_edge->twin = new_bottom_twin_edge.get();

    //Add the new top half-edge to the cycle around vertex B
    original_dcel.add_edge_to_vertex(*new_top_edge.get(), *new_top_edge->origin);

    //Add the new twin of the bottom half-edge to vertex C
    original_dcel.add_edge_to_vertex(*new_bottom_twin_edge.get(), *vertex_C);
}

DCEL::DCEL_Vertex* DCEL::Overlay_Handler::create_new_vertex(const Vec2& position, bool target_DCEL)
{
    if (target_DCEL)
    {
        return original_dcel.vertices.emplace_back(std::make_unique<DCEL_Vertex>(position)).get();
    }
    else
    {
        return overlaying_dcel.vertices.emplace_back(std::make_unique<DCEL_Vertex>(position)).get();
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
    if (incident_half_edge == nullptr)
    {
        return std::vector<DCEL::DCEL_Half_Edge*>();
    }

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

void DCEL::Overlay_Handler::overlay_handle_unique_vertex(std::vector<int>& new_intersecting_segments, std::vector<int>& new_top_segments, const Vec2& event_point)
{
    //If the vertex is unique to the overlaying DCEL, copy it over, otherwise, do nothing

    const DCEL_Vertex* old_vertex = nullptr;
    if (!new_top_segments.empty())
    {
        if (DCEL_edges[new_top_segments.front()].original_dcel)
        {
            //Only the original dcel contributes, no records need to be updated.
            return;
        }

        old_vertex = DCEL_edges[new_top_segments.front()].get_top_dcel_vertex();
    }
    else
    {
        if (DCEL_edges[new_intersecting_segments.front()].original_dcel)
        {
            //Only the original dcel contributes, no records need to be updated.
            return;
        }

        old_vertex = DCEL_edges[new_intersecting_segments.front()].get_vertex_on_point(event_point);

    }

    assert(old_vertex->position == event_point);

    overlay_copy_vertex_into_dcel(old_vertex);
}

//Checks if the given event contains elements from multiple DCELs
bool DCEL::Overlay_Handler::overlay_event_contains_both_dcels(const std::vector<int>& new_intersecting_segments, const std::vector<int>& new_top_segments) const
{
    if (new_intersecting_segments.size() + new_top_segments.size() <= 1)
    {
        return false;
    }

    auto dcel_origin_comparer = [this](int l, int r)
        {
            return DCEL_edges[l].original_dcel == DCEL_edges[r].original_dcel;
        };

    bool both_dcels = false;

    //First check if all intersecting_segments originate from the same dcel
    if (new_intersecting_segments.size() > 1)
    {
        both_dcels = !std::equal(new_intersecting_segments.begin() + 1, new_intersecting_segments.end(), new_intersecting_segments.begin(), dcel_origin_comparer);

        if (both_dcels)
        {
            return true;
        }
    }

    //Then check if all top_segments originate from the same dcel
    if (new_top_segments.size() > 1)
    {
        both_dcels = !std::equal(new_top_segments.begin() + 1, new_top_segments.end(), new_top_segments.begin(), dcel_origin_comparer);

        if (both_dcels)
        {
            return true;
        }
    }

    //Finally if both lists contain the same values, check if both lists differ from each other
    if (!new_intersecting_segments.empty() && !new_top_segments.empty())
    {
        return DCEL_edges[new_intersecting_segments.front()].original_dcel != DCEL_edges[new_top_segments.front()].original_dcel;
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

void DCEL::Overlay_Handler::delete_registered_half_edges()
{
    //Sort in reverse order
    std::sort(to_delete.begin(), to_delete.end(), std::greater<>());

    //Swap to remove objects to the back and resize to delete
    size_t new_size = original_dcel.half_edges.size();

    for (size_t i = 0; i < to_delete.size(); i++)
    {
        std::swap(original_dcel.half_edges[to_delete[i]], original_dcel.half_edges[original_dcel.half_edges.size() - 1 - i]);
        new_size--;
    }

    original_dcel.half_edges.resize(new_size);
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
            DCEL_edges.emplace_back(original_dcel.half_edges[i].get(), is_original_half_edge, i);
        }
    }

    //Call the sweep line algorithm handling overlay cases from top to bottom.
    resolve_edge_intersections();

    overlaying_dcel.clear();
}
