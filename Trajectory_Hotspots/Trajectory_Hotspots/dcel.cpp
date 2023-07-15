#include "pch.h"
#include "dcel.h"

#include "segment.h"
#include "segment_intersection.h"

void DCEL::overlay_dcel(DCEL& other_dcel)
{
    //TODO:
    //-Call sweep line algorithm
    //For each intersection call appropriate overlay helper function
    //Not sure if we want to just make a new DCEL..

    size_t original_half_edge_count = this->half_edge_count();

    //Move all the half edges from the other dcel into this one.
    this->half_edges.insert(this->half_edges.end(),
        std::make_move_iterator(other_dcel.half_edges.begin()),
        std::make_move_iterator(other_dcel.half_edges.end()));


    ////Construct the edges representing the half edge twins for use in the sweep line algorithm.
    std::vector<DCEL_Overlay_Edge_Wrapper> DCEL_edges;
    DCEL_edges.reserve(half_edge_count() / 2);

    for (size_t i = 0; i < this->half_edge_count(); i++)
    {
        if (this->half_edges[i]->is_orientated_left_right())
        {
            bool is_original_half_edge = i < original_half_edge_count;
            DCEL_edges.emplace_back(this->half_edges[i].get(), is_original_half_edge);
        }
    }

    //Call the sweep line algorithm handling overlay cases from top to bottom.
    //Segment_Intersection_Sweep_Line::find_segment_intersections(DCEL_edges);

    resolve_edge_intersections(DCEL_edges);
}

void DCEL::insert_segment(const Segment& segment)
{
    //Create the records for a single segment
    //The two half edges are each others twin, prev, and next
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

    if (half_edges.size() == 2)
    {
        return;
    }
    else if (half_edges.size() == 4)
    {
        Segment other_segment(half_edges[0]->origin->position, half_edges[1]->origin->position);

        Vec2 intersection_point;
        Segment::Intersection_Type segment_intersection_type = segment.intersects(other_segment, intersection_point);

        switch (segment_intersection_type)
        {
        case Segment::Intersection_Type::point:
            handle_point_intersection(intersection_point, half_edges[0].get(), new_half_edge_1.get());
            break;
        case Segment::Intersection_Type::collinear:
            break;
        case Segment::Intersection_Type::parallel:
        case Segment::Intersection_Type::none:
            return;
            break;
        }
    }
}

void DCEL::handle_point_intersection(const Vec2& intersection_point, DCEL::DCEL_Half_Edge* old_half_edge, DCEL::DCEL_Half_Edge* new_half_edge)
{
    DCEL_Vertex* old_dcel_endpoint = nullptr;
    DCEL_Vertex* new_dcel_endpoint = nullptr;
    intersection_on_endpoint(intersection_point, old_half_edge, new_half_edge, old_dcel_endpoint, new_dcel_endpoint);

    if (old_dcel_endpoint || new_dcel_endpoint)
    {
        if (old_dcel_endpoint && new_dcel_endpoint)
        {
            overlay_vertex_on_vertex(old_dcel_endpoint, new_dcel_endpoint);

            //Overlay_vertex_on_vertex makes one of the newly added vertices redundant, so we can delete it.
            if (new_dcel_endpoint == new_half_edge->origin)
            {
                vertices.erase(vertices.end() - 2);
            }
            else
            {
                vertices.pop_back();
            }
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

void DCEL::intersection_on_endpoint(
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

void DCEL::overlay_edge_on_vertex(DCEL_Half_Edge* edge, DCEL_Vertex* vertex)
{
    //Note: Technically there's some duplicate code here because the handling of the original half-edges is almost identical.
    //Splitting this up gives some trouble with overwriting the next pointers though.
    //Also, this way we can start the search for the second adjacent half-edges where the first ended.

    half_edges.reserve(half_edges.size() + 2); //Reserve two extra spots to prevent invalidating the first reference when exceeding space on the second emplace_back
    const std::unique_ptr<DCEL_Half_Edge>& new_half_edge_1 = half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>()); //new twin of edge
    const std::unique_ptr<DCEL_Half_Edge>& new_half_edge_2 = half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>()); //new twin of edge->twin

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

    vertex->find_adjacent_half_edges(edge, current_half_edge, CW_half_edge, CCW_half_edge);

    //face is left of halfedge, so halfedge with vertex as origin has CCW half-edge as prev 
    new_half_edge_1->prev = CCW_half_edge->twin;
    CCW_half_edge->twin->next = new_half_edge_1.get();

    //and halfedge with vertex as target has CW half-edge as next
    edge->next = CW_half_edge;
    CW_half_edge->prev = edge;

    //Find the other side, start from the end of the previous rotation (it is impossible for it to lie between the just added and CW half-edge)
    vertex->find_adjacent_half_edges(twin, CW_half_edge, CW_half_edge, CCW_half_edge);

    new_half_edge_2->prev = CCW_half_edge->twin;
    CCW_half_edge->twin->next = new_half_edge_2.get();

    twin->next = CW_half_edge;
    CW_half_edge->prev = twin;
}

DCEL::DCEL_Vertex* DCEL::overlay_edge_on_edge(DCEL_Half_Edge* edge_1, DCEL_Half_Edge* edge_2, const Vec2& intersection_point)
{
    const std::unique_ptr<DCEL_Vertex>& new_dcel_vertex = vertices.emplace_back(std::make_unique<DCEL_Vertex>(intersection_point));

    DCEL_Half_Edge* edge_1_old_twin = edge_1->twin;

    //Create two new twins for the first edge, 
    //these are outgoing from the new vertex so we set one of them as the incident half edge
    half_edges.reserve(half_edges.size() + 2); //Reserve two extra spots to prevent invalidating the first reference when exceeding space on the second emplace_back
    const std::unique_ptr<DCEL_Half_Edge>& edge_1_new_twin_1 = half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>());
    const std::unique_ptr<DCEL_Half_Edge>& edge_1_new_twin_2 = half_edges.emplace_back(std::make_unique<DCEL_Half_Edge>());

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

void DCEL::overlay_vertex_on_vertex(DCEL_Vertex* vertex_1, const DCEL_Vertex* vertex_2) const
{
    //Gather half-edges around second vertex
    //For each: Set origins to first vertex
    //          and switch prev, prev->next,
    //          twin->next, and twin->next->prev pointers

    std::vector<DCEL_Half_Edge*> incident_half_edges_v2 = vertex_2->get_incident_half_edges();

    DCEL_Half_Edge* CW_half_edge = nullptr;
    DCEL_Half_Edge* CCW_half_edge = nullptr;

    DCEL_Half_Edge* current_half_edge = vertex_1->incident_half_edge;

    for (auto& incident_half_edge_v2 : incident_half_edges_v2)
    {
        incident_half_edge_v2->origin = vertex_1;

        vertex_1->find_adjacent_half_edges(incident_half_edge_v2->twin, current_half_edge, CW_half_edge, CCW_half_edge);

        //half-edges chain counter clockwise and adjacent half-edges point outward
        //so: CW half-edge is just the CW adjacent
        //    and CCW half-edge is the twin of CCW adjacent

        //(We can prevent the two twin writes by checking if the CCW is the previous added half-edge
        //but adding branching is probably slower)

        incident_half_edge_v2->twin->next = CW_half_edge;
        CW_half_edge->prev = incident_half_edge_v2->twin;

        incident_half_edge_v2->prev = CCW_half_edge->twin;
        CCW_half_edge->twin->next = incident_half_edge_v2;
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

void DCEL::DCEL_Vertex::find_adjacent_half_edges(const DCEL::DCEL_Half_Edge* query_edge, DCEL::DCEL_Half_Edge* starting_half_edge, DCEL::DCEL_Half_Edge*& CW_half_edge, DCEL::DCEL_Half_Edge*& CCW_half_edge) const
{
    DCEL::DCEL_Half_Edge* prev_half_edge = starting_half_edge->prev->twin;
    DCEL::DCEL_Half_Edge* current_half_edge = starting_half_edge;

    //Keep rotating clockwise until we find the first half-edges clock and counter-clockwise from the queried half-edge

    Float prev_order = Vec2::order_around_center(this->position, query_edge->origin->position, prev_half_edge->target()->position);

    do
    {
        Float new_order = Vec2::order_around_center(this->position, query_edge->origin->position, current_half_edge->target()->position);

        //A positive order is counterclockwise and negative is clockwise.
        //When prev and next are CCW and CW respectively, the queried half-edge is in between these two.
        if (prev_order > 0.f && new_order < 0.f)
        {
            CW_half_edge = current_half_edge;
            CCW_half_edge = prev_half_edge;
            return;
        }
        else
        {
            prev_half_edge = current_half_edge;
            prev_order = new_order;
            current_half_edge = current_half_edge->twin->next;
        }
    } while (current_half_edge != starting_half_edge);

    if (current_half_edge == prev_half_edge)
    {
        CW_half_edge = current_half_edge;
        CCW_half_edge = prev_half_edge;
        return;
    }

    assert(false);
}

std::vector<DCEL::DCEL_Half_Edge*> DCEL::DCEL_Vertex::get_incident_half_edges() const
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

//Returns if the half-edges origin lies to the left of its destination, 
//if they share an x axis it returns true if its origin is above its destination.
bool DCEL::DCEL_Half_Edge::is_orientated_left_right() const
{
    return orientation_left_right(origin->position, twin->origin->position);
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

bool collinear_overlap(const DCEL::DCEL_Overlay_Edge_Wrapper& segment1, const DCEL::DCEL_Overlay_Edge_Wrapper& segment2, Vec2& overlap_start, Vec2& overlap_end)
{
    return collinear_overlap(segment1.edge_segment, segment2.edge_segment, overlap_start, overlap_end);
}


void DCEL::resolve_edge_intersections(std::vector<DCEL_Overlay_Edge_Wrapper>& DCEL_edges)
{
    namespace Sweep = Segment_Intersection_Sweep_Line;

    Sweep::map event_queue;

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
        Sweep::Intersection_Info intersection_results = Handle_Event(status_structure, event_queue, DCEL_edges, event_queue.begin()->first, event_queue.begin()->second);

        handle_overlay_event(DCEL_edges, intersection_results, event_queue.begin()->first);

        event_queue.erase(event_queue.begin());
    }
}

void DCEL::handle_overlay_event(std::vector<DCEL::DCEL_Overlay_Edge_Wrapper>& DCEL_edges, Segment_Intersection_Sweep_Line::Intersection_Info& intersection_results, const Vec2& event_point)
{
    if (intersection_results.segment_count() < 2)
    {
        return;
    }

    if (!overlay_event_contains_both_dcels(DCEL_edges, intersection_results))
    {
        //Only one dcel contributes, no records need to be updated.
        return;
    }

    auto interior_it = intersection_results.interior_segments.begin();
    DCEL_Vertex* dcel_vertex_at_event_point = nullptr;

    //Get or create the DCEL_Vertex at this event point so we can correct the records around it.
    if (intersection_results.bottom_segments.empty() && intersection_results.top_segments.empty())
    {
        //No bottom or top points at this event means we have to create a new DCEL_vertex using the first two segments.
        DCEL_Half_Edge* first_intersecting_half_edge = DCEL_edges[*interior_it].underlying_half_edge;
        ++interior_it;
        DCEL_Half_Edge* second_intersecting_half_edge = DCEL_edges[*interior_it].underlying_half_edge;

        dcel_vertex_at_event_point = overlay_edge_on_edge(first_intersecting_half_edge, second_intersecting_half_edge, event_point);
    }
    else
    {
        //If this event contains a top or bottom point, just retrieve the DCEL_vertex at these points.

        if (!intersection_results.bottom_segments.empty())
        {
            dcel_vertex_at_event_point = DCEL_edges[intersection_results.bottom_segments[0]].get_bottom_dcel_vertex();
        }
        else if (!intersection_results.top_segments.empty())
        {
            dcel_vertex_at_event_point = DCEL_edges[intersection_results.top_segments[0]].get_top_dcel_vertex();
        }

    }

    assert(dcel_vertex_at_event_point != nullptr);

    //If there are more internal intersecting segments we can add them one by one with overlay_edge_on_vertex.
    for (; interior_it != intersection_results.interior_segments.end(); ++interior_it)
    {
        DCEL_Half_Edge* intersecting_half_edge = DCEL_edges[*interior_it].underlying_half_edge;
        overlay_edge_on_vertex(intersecting_half_edge, dcel_vertex_at_event_point);
    }

    //The overlay_vertex_on_vertex function fuses the records of both dcel_vertices so we just need to call it once with the two different vertices,
    //Hence the return statement.
    for (int bottom_segment_index : intersection_results.bottom_segments)
    {
        const DCEL_Vertex* next_dcel_vertex = DCEL_edges[bottom_segment_index].get_bottom_dcel_vertex();

        if (dcel_vertex_at_event_point != next_dcel_vertex)
        {
            overlay_vertex_on_vertex(dcel_vertex_at_event_point, next_dcel_vertex);
            return;
        }
    }


    for (int top_segment_index : intersection_results.top_segments)
    {
        const DCEL_Vertex* next_dcel_vertex = DCEL_edges[top_segment_index].get_top_dcel_vertex();

        if (dcel_vertex_at_event_point != next_dcel_vertex)
        {
            overlay_vertex_on_vertex(dcel_vertex_at_event_point, next_dcel_vertex);
            return;
        }
    }


    //TODO: Handle collinear
}

bool DCEL::overlay_event_contains_both_dcels(const std::vector<DCEL_Overlay_Edge_Wrapper>& DCEL_edges, const Segment_Intersection_Sweep_Line::Intersection_Info& intersection_results) const
{
    bool original_dcel = false;
    bool overlaying_dcel = false;

    check_dcel_versions(DCEL_edges, intersection_results.interior_segments.begin(), intersection_results.interior_segments.end(), original_dcel, overlaying_dcel);

    if (original_dcel && overlaying_dcel)
    {
        return true;
    }

    check_dcel_versions(DCEL_edges, intersection_results.top_segments.begin(), intersection_results.top_segments.end(), original_dcel, overlaying_dcel);

    if (original_dcel && overlaying_dcel)
    {
        return true;
    }

    check_dcel_versions(DCEL_edges, intersection_results.bottom_segments.begin(), intersection_results.bottom_segments.end(), original_dcel, overlaying_dcel);

    if (original_dcel && overlaying_dcel)
    {
        return true;
    }

    check_dcel_versions(DCEL_edges, intersection_results.collinear_segments.begin(), intersection_results.collinear_segments.end(), original_dcel, overlaying_dcel);

    if (original_dcel && overlaying_dcel)
    {
        return true;
    }

    return false;
}