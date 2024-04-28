#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/dcel.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace Microsoft::VisualStudio::CppUnitTestFramework
{
    template<> static std::wstring ToString<DCEL::DCEL_Half_Edge>(class DCEL::DCEL_Half_Edge* t) { return L"DCEL::DCEL_Half_Edge*"; }
    template<> static std::wstring ToString<DCEL::DCEL_Half_Edge>(const class DCEL::DCEL_Half_Edge& t) { return L"DCEL::DCEL_Half_Edge&"; }

    template<> static std::wstring ToString<DCEL::DCEL_Vertex>(class DCEL::DCEL_Vertex* t) { return L"DCEL::DCEL_Vertex*"; }
    template<> static std::wstring ToString<DCEL::DCEL_Vertex>(const class DCEL::DCEL_Vertex& t) { return L"DCEL::DCEL_Vertex&"; }
}

namespace TestTrajectoryHotspots
{
    TEST_CLASS(TestTrajectoryHotspotsDCEL)
    {
        TEST_METHOD(DCEL_Construction_single_segment)
        {
            DCEL test_dcel;

            const Segment new_segment(Vec2(6.f, 6.f), Vec2(20.f, 20.f));

            test_dcel.insert_segment(new_segment);

            Assert::AreEqual((size_t)2, test_dcel.vertex_count());
            Assert::AreEqual((size_t)2, test_dcel.half_edge_count());

            Assert::AreEqual(test_dcel.half_edges[0]->twin, test_dcel.half_edges[1].get());
            Assert::AreEqual(test_dcel.half_edges[1]->twin, test_dcel.half_edges[0].get());

            //With a single segment the half edges should be each others next and prev
            Assert::AreEqual(test_dcel.half_edges[0]->next, test_dcel.half_edges[1].get());
            Assert::AreEqual(test_dcel.half_edges[1]->next, test_dcel.half_edges[0].get());
            Assert::AreEqual(test_dcel.half_edges[0]->prev, test_dcel.half_edges[1].get());
            Assert::AreEqual(test_dcel.half_edges[1]->prev, test_dcel.half_edges[0].get());

            //Both half edges should have a unique origin vertex, which of the two new vertices doesn't matter
            Assert::IsTrue((test_dcel.half_edges[0]->origin == test_dcel.vertices[0].get() && test_dcel.half_edges[1]->origin == test_dcel.vertices[1].get())
                || (test_dcel.half_edges[1]->origin == test_dcel.vertices[0].get() && test_dcel.half_edges[0]->origin == test_dcel.vertices[1].get()));

            //And the two vertices should each have a unique incident half edge
            Assert::IsTrue((test_dcel.vertices[0]->incident_half_edge == test_dcel.half_edges[0].get() && test_dcel.vertices[1]->incident_half_edge == test_dcel.half_edges[1].get())
                || (test_dcel.vertices[0]->incident_half_edge == test_dcel.half_edges[1].get() && test_dcel.vertices[1]->incident_half_edge == test_dcel.half_edges[0].get()));
        }

        TEST_METHOD(DCEL_Construction_two_internal_intersecting_segments)
        {
            DCEL test_dcel;

            const Segment new_segment(Vec2(6.f, 6.f), Vec2(20.f, 20.f));
            const Segment new_segment_2(Vec2(4.f, 20.f), Vec2(22.f, 8.f));

            test_dcel.insert_segment(new_segment);
            test_dcel.insert_segment(new_segment_2);

            //Four endpoints plus the intersection point
            Assert::AreEqual((size_t)5, test_dcel.vertex_count(), L"Incorrect amount of total DCEL vertices.");

            //Every segment consists of two half-edges, 
            //both segments split into two at the intersection point, doubling the amount of half-edge to 8
            Assert::AreEqual((size_t)8, test_dcel.half_edge_count(), L"Incorrect amount of total DCEL half-edges.");

            //The intersection point should be a new vertex in the DCEL
            Vec2 intersection_point;
            new_segment.intersects(new_segment_2, intersection_point);

            Assert::IsTrue(std::find_if(test_dcel.vertices.begin(), test_dcel.vertices.end(), [&intersection_point](const std::unique_ptr<DCEL::DCEL_Vertex>& vert) { return vert->position == intersection_point; }) != test_dcel.vertices.end(), L"Intersection point not in DCEL.");


            //Both segments get split into two, so there should be four incident half-edges around the vertex at the intersection point
            DCEL::DCEL_Vertex* intersection_vertex = test_dcel.get_vertex_at_position(intersection_point);

            std::vector<DCEL::DCEL_Half_Edge*> incident_half_edges = intersection_vertex->get_incident_half_edges();

            Assert::AreEqual((size_t)4, incident_half_edges.size(), L"Incorrect amount of incident half-edges.");

            //All endpoints should show up once around the intersection vertex
            std::vector<Vec2> incident_targets;
            for (auto& incident_half_edge : incident_half_edges)
            {
                incident_targets.push_back(incident_half_edge->target()->position);
            }

            std::vector<Vec2> endpoints{ new_segment.start, new_segment_2.start, new_segment.end, new_segment_2.end };

            Assert::IsTrue(std::is_permutation(incident_targets.begin(), incident_targets.end(), endpoints.begin()), L"Incorrect endpoints around vertex.");
        }

        TEST_METHOD(DCEL_Construction_two_segments_intersecting_at_endpoints)
        {
            DCEL test_dcel;

            const Segment new_segment(Vec2(6.f, 6.f), Vec2(20.f, 20.f));
            const Segment new_segment_2(Vec2(4.f, 20.f), Vec2(20.f, 20.f));

            test_dcel.insert_segment(new_segment);
            test_dcel.insert_segment(new_segment_2);

            Assert::AreEqual((size_t)3, test_dcel.vertex_count(), L"Incorrect amount of total DCEL vertices.");

            //The targets of the incident half-edge of the middle vertex should be the other two endpoints

            DCEL::DCEL_Vertex* middle_vert = nullptr;
            for (const auto& vert : test_dcel.vertices)
            {
                if (vert->position == new_segment.end)
                {
                    middle_vert = vert.get();
                }
            }

            if (middle_vert != nullptr)
            {
                std::vector<DCEL::DCEL_Half_Edge*> incident_half_edges_middle = middle_vert->get_incident_half_edges();

                std::vector<Vec2> incident_targets;
                for (auto& incident_half_edge : incident_half_edges_middle)
                {
                    incident_targets.push_back(incident_half_edge->target()->position);
                }

                std::vector<Vec2> endpoints{ new_segment.start, new_segment_2.start };

                Assert::IsTrue(std::is_permutation(incident_targets.begin(), incident_targets.end(), endpoints.begin()), L"Incorrect endpoints around vertex.");
            }
            else
            {
                Assert::Fail(L"middle_vert was not present");
            }

            //The half-edge loop should contain four unique half-edges, all four half-edges in the current dcel

            const DCEL::DCEL_Half_Edge* he0_next = test_dcel.half_edges[0].get();
            const DCEL::DCEL_Half_Edge* he0_prev = test_dcel.half_edges[0].get();

            std::vector<const DCEL::DCEL_Half_Edge*> half_edge_chain_next;
            std::vector<const DCEL::DCEL_Half_Edge*> half_edge_chain_prev;

            for (size_t i = 0; i < 4; i++)
            {
                half_edge_chain_next.emplace_back(he0_next->next);
                half_edge_chain_prev.emplace_back(he0_prev->prev);

                he0_next = he0_next->next;
                he0_prev = he0_prev->prev;
            }

            std::vector<const DCEL::DCEL_Half_Edge*> all_half_edges;
            for (const auto& he : test_dcel.half_edges)
            {
                all_half_edges.emplace_back(he.get());
            }

            Assert::IsTrue(std::is_permutation(half_edge_chain_next.begin(), half_edge_chain_next.end(), half_edge_chain_prev.begin()), L"Half-edge chain next is not equal to the prev chain.");
            Assert::IsTrue(std::is_permutation(half_edge_chain_next.begin(), half_edge_chain_next.end(), all_half_edges.begin()), L"Half-edge chain is incorrect.");
        }

        TEST_METHOD(DCEL_Construction_two_segments_intersecting_vertex_on_edge)
        {
            DCEL test_dcel;

            const Segment new_segment(Vec2(6.f, 20.f), Vec2(20.f, 20.f));
            const Segment new_segment_2(Vec2(12.f, 20.f), Vec2(12.f, 0.f));

            test_dcel.insert_segment(new_segment);
            test_dcel.insert_segment(new_segment_2);

            //Four endpoints with one being the intersection point
            Assert::AreEqual((size_t)4, test_dcel.vertex_count(), L"Incorrect amount of total DCEL vertices.");
        }

        TEST_METHOD(DCEL_overlay_vertex_on_vertex_two_on_one_side)
        {
            //This tests if the clockwise and counter clockwise neighbours 
            //are properly detected when they are both on the same side of the inserted segment. e.g:
            // ./
            // |\
            //   \ <-

            DCEL test_dcel;

            const Segment segment_1(Vec2(6.f, 6.f), Vec2(8.f, 6.f));
            const Segment segment_2(Vec2(6.f, 4.f), Vec2(6.f, 6.f));

            test_dcel.insert_segment(segment_1);
            test_dcel.insert_segment(segment_2);

            const Segment new_segment(Vec2(4.f, 4.f), Vec2(6.f, 6.f));

            test_dcel.insert_segment(new_segment);

            size_t vertex_count = test_dcel.vertex_count();

            Assert::AreEqual((size_t)4, vertex_count, L"Incorrect amount of total DCEL vertices.");

            DCEL::DCEL_Vertex* middle_vertex = nullptr;

            //Check around the middle vertex:
            for (const auto& v : test_dcel.vertices)
            {
                if (v->position == Vec2(6.f, 6.f))
                {
                    middle_vertex = v.get();
                    break;
                }
            }

            if (middle_vertex != nullptr)
            {
                std::vector<Vec2> correct_endpoints{ new_segment.start, segment_1.end, segment_2.start };

                //The target vertices of the adjacent half edges should include all other vertices
                std::vector<DCEL::DCEL_Half_Edge*> incident_half_edges_middle = middle_vertex->get_incident_half_edges();

                size_t first_index = 0;
                std::vector<Vec2> incident_targets;
                for (size_t i = 0; i < incident_half_edges_middle.size(); i++)
                {
                    incident_targets.push_back(incident_half_edges_middle[i]->target()->position);

                    if (incident_half_edges_middle[i]->target()->position == correct_endpoints[0])
                    {
                        first_index = i;
                    }

                }


                Assert::IsTrue(std::is_permutation(incident_targets.begin(), incident_targets.end(), correct_endpoints.begin()), L"Incorrect endpoints around vertex.");

                //The origin vertex of the adjacent half edges should be the middle
                for (auto& incident_half_edge : incident_half_edges_middle)
                {
                    Assert::AreEqual(middle_vertex->position, incident_half_edge->origin->position);
                }

                //Check if the ordering is also correct
                std::rotate(incident_targets.begin(), incident_targets.begin() + first_index, incident_targets.end());

                Assert::AreEqual(correct_endpoints, incident_targets);
            }
            else
            {
                Assert::Fail(L"middle_vert was not present");
            }
        }

        TEST_METHOD(DCEL_overlay_diamond)
        {
            Vec2 north(10.f, 14.f);
            Vec2 west(8.f, 10.f);
            Vec2 south(10.f, 6.f);
            Vec2 east(12.f, 10.f);

            std::vector west_edges{ Segment(north, west), Segment(west, south) };
            std::vector east_edges{ Segment(north, east), Segment(east, south) };

            DCEL west_side;
            west_side.insert_segment(west_edges[0]);
            west_side.insert_segment(west_edges[1]);

            DCEL east_side;
            east_side.insert_segment(east_edges[0]);
            east_side.insert_segment(east_edges[1]);

            west_side.overlay_dcel(east_side);

            Assert::AreEqual(static_cast<size_t>(8), west_side.half_edge_count(), L"Incorrect amount of half-edges.");
            Assert::AreEqual(static_cast<size_t>(4), west_side.vertex_count(), L"Incorrect amount of vertices.");

            const DCEL::DCEL_Vertex* north_vertex = west_side.get_vertex_at_position(north);
            const DCEL::DCEL_Vertex* east_vertex = west_side.get_vertex_at_position(east);

            Assert::IsNotNull(north_vertex);

            std::vector<const DCEL::DCEL_Half_Edge*> adjacent_half_edges_north = north_vertex->get_incident_half_edges();
            std::vector<const DCEL::DCEL_Half_Edge*> outer_half_edge_cycle;

            //Half-edges travel around a hole in clockwise order, so the half-edge pointing from north to east is on the outer cycle
            for (const DCEL::DCEL_Half_Edge*& adjacent_half_edge : adjacent_half_edges_north)
            {
                if (adjacent_half_edge->target() == east_vertex)
                {
                    outer_half_edge_cycle = adjacent_half_edge->get_cycle();
                    break;
                }
            }

            Assert::IsFalse(outer_half_edge_cycle.empty(), L"No outer half-edge cycle");

            const DCEL::DCEL_Half_Edge* inner_half_edge = outer_half_edge_cycle.at(0)->twin;
            std::vector<const DCEL::DCEL_Half_Edge*> inner_half_edge_cycle = inner_half_edge->get_cycle();

            Assert::AreEqual(size_t(4), outer_half_edge_cycle.size(), L"Outer cycle has the incorrect size.");
            Assert::AreEqual(size_t(4), inner_half_edge_cycle.size(), L"Inner cycle has the incorrect size.");

            for (auto& inner_he : inner_half_edge_cycle)
            {
                Assert::IsFalse(contains(outer_half_edge_cycle, inner_he));
            }

            for (auto& outer_he : outer_half_edge_cycle)
            {
                Assert::IsFalse(contains(inner_half_edge_cycle, outer_he));
            }
        }



        void test_dcel_colinear_partial_overlap(const DCEL& dcel, const Vec2& pA, const Vec2& pB, const Vec2& pC, const Vec2& pD) const
        {
            Assert::AreEqual(size_t(6), dcel.half_edge_count(), L"DCEL has an incorrect amount of half edges.");
            Assert::AreEqual(size_t(4), dcel.vertex_count(), L"DCEL has an incorrect amount of half vertices.");

            DCEL::DCEL_Vertex* vertex_a = dcel.get_vertex_at_position(pA);

            std::vector<Vec2> correct_order = { pA, pB,pC,pD,pC,pB,pA };
            std::vector<Vec2> current_next_order;
            std::vector<Vec2> current_prev_order;

            //Test the next cycles
            const DCEL::DCEL_Half_Edge* current_half_edge = vertex_a->incident_half_edge;
            for (size_t i = 0; i < 7; i++)
            {
                current_next_order.push_back(current_half_edge->origin->position);
                current_half_edge = current_half_edge->next;
            }

            Assert::AreEqual(correct_order, current_next_order);

            //Test the prev cycles
            current_half_edge = vertex_a->incident_half_edge;
            for (size_t i = 0; i < 7; i++)
            {
                current_prev_order.push_back(current_half_edge->origin->position);
                current_half_edge = current_half_edge->prev;
            }

            Assert::AreEqual(correct_order, current_prev_order);

            //Test if the incident half_edges of the vertices are set correctly
            DCEL::DCEL_Vertex* vertex_b = dcel.get_vertex_at_position(pB);
            DCEL::DCEL_Vertex* vertex_c = dcel.get_vertex_at_position(pC);
            DCEL::DCEL_Vertex* vertex_d = dcel.get_vertex_at_position(pD);

            auto incident_he_a = vertex_a->get_incident_half_edges();
            auto incident_he_b = vertex_b->get_incident_half_edges();
            auto incident_he_c = vertex_c->get_incident_half_edges();
            auto incident_he_d = vertex_d->get_incident_half_edges();

            Assert::AreEqual(size_t(1), incident_he_a.size());
            Assert::AreEqual(size_t(2), incident_he_b.size());
            Assert::AreEqual(size_t(2), incident_he_c.size());
            Assert::AreEqual(size_t(1), incident_he_d.size());

            Assert::AreEqual(vertex_b, incident_he_a[0]->twin->origin);

            std::vector<DCEL::DCEL_Vertex*> expected_neighbours_b = { vertex_a, vertex_c };
            std::vector<DCEL::DCEL_Vertex*> expected_neighbours_c = { vertex_b, vertex_d };
            std::vector<DCEL::DCEL_Vertex*> neighbours_b = { incident_he_b[0]->twin->origin, incident_he_b[1]->twin->origin };
            std::vector<DCEL::DCEL_Vertex*> neighbours_c = { incident_he_c[0]->twin->origin, incident_he_c[1]->twin->origin };

            Assert::IsTrue(std::is_permutation(expected_neighbours_b.begin(), expected_neighbours_b.end(), neighbours_b.begin()));
            Assert::IsTrue(std::is_permutation(expected_neighbours_c.begin(), expected_neighbours_c.end(), neighbours_c.begin()));

            Assert::AreEqual(vertex_c, incident_he_d[0]->twin->origin);
        }

        TEST_METHOD(DCEL_colinear_partial_overlap)
        {
            Vec2 pA(2.f, 6.f);
            Vec2 pB(4.f, 6.f);
            Vec2 pC(6.f, 6.f);
            Vec2 pD(9.f, 6.f);

            Segment f(pA, pC);
            Segment g(pB, pD);

            DCEL dcel;

            dcel.insert_segment(f);
            dcel.insert_segment(g);

            test_dcel_colinear_partial_overlap(dcel, pA, pB, pC, pD);
        }

        TEST_METHOD(DCEL_colinear_partial_overlap_reverse)
        {
            Vec2 pA(2.f, 6.f);
            Vec2 pB(4.f, 6.f);
            Vec2 pC(6.f, 6.f);
            Vec2 pD(9.f, 6.f);

            Segment f(pA, pC);
            Segment g(pB, pD);

            DCEL dcel_reverse;

            dcel_reverse.insert_segment(g);
            dcel_reverse.insert_segment(f);

            test_dcel_colinear_partial_overlap(dcel_reverse, pA, pB, pC, pD);
        }

        TEST_METHOD(DCEL_colinear_embedded)
        {
            Vec2 pA(2.f, 6.f);
            Vec2 pD(9.f, 6.f);

            Vec2 pB(4.f, 6.f);
            Vec2 pC(6.f, 6.f);

            Segment f(pA, pD);
            Segment g(pB, pC);

            DCEL dcel;

            dcel.insert_segment(f);
            dcel.insert_segment(g);

            Assert::AreEqual(size_t(6), dcel.half_edge_count(), L"DCEL has an incorrect amount of half edges.");
            Assert::AreEqual(size_t(4), dcel.vertex_count(), L"DCEL has an incorrect amount of half vertices.");

            std::vector<Vec2> correct_order = { pA, pB,pC,pD,pC,pB,pA };
            std::vector<Vec2> current_next_order;
            std::vector<Vec2> current_prev_order;

            DCEL::DCEL_Vertex* vertex_a = dcel.get_vertex_at_position(pA);

            //Test the next cycles
            const DCEL::DCEL_Half_Edge* current_half_edge = vertex_a->incident_half_edge;
            for (size_t i = 0; i < 7; i++)
            {
                current_next_order.push_back(current_half_edge->origin->position);
                current_half_edge = current_half_edge->next;
            }

            Assert::AreEqual(correct_order, current_next_order);

            //Test the prev cycles
            current_half_edge = vertex_a->incident_half_edge;
            for (size_t i = 0; i < 7; i++)
            {
                current_prev_order.push_back(current_half_edge->origin->position);
                current_half_edge = current_half_edge->prev;
            }

            Assert::AreEqual(correct_order, current_prev_order);

            DCEL::DCEL_Vertex* vertex_b = dcel.get_vertex_at_position(pB);
            DCEL::DCEL_Vertex* vertex_c = dcel.get_vertex_at_position(pC);
            DCEL::DCEL_Vertex* vertex_d = dcel.get_vertex_at_position(pD);

            auto incident_he_a = vertex_a->get_incident_half_edges();
            auto incident_he_b = vertex_b->get_incident_half_edges();
            auto incident_he_c = vertex_c->get_incident_half_edges();
            auto incident_he_d = vertex_d->get_incident_half_edges();

            Assert::AreEqual(size_t(1), incident_he_a.size());
            Assert::AreEqual(size_t(2), incident_he_b.size());
            Assert::AreEqual(size_t(2), incident_he_c.size());
            Assert::AreEqual(size_t(1), incident_he_d.size());

            Assert::AreEqual(vertex_b, incident_he_a[0]->twin->origin);

            std::vector<DCEL::DCEL_Vertex*> expected_neighbours_b = { vertex_a, vertex_c };
            std::vector<DCEL::DCEL_Vertex*> expected_neighbours_c = { vertex_b, vertex_d };
            std::vector<DCEL::DCEL_Vertex*> neighbours_b = { incident_he_b[0]->twin->origin, incident_he_b[1]->twin->origin };
            std::vector<DCEL::DCEL_Vertex*> neighbours_c = { incident_he_c[0]->twin->origin, incident_he_c[1]->twin->origin };

            Assert::IsTrue(std::is_permutation(expected_neighbours_b.begin(), expected_neighbours_b.end(), neighbours_b.begin()));
            Assert::IsTrue(std::is_permutation(expected_neighbours_c.begin(), expected_neighbours_c.end(), neighbours_c.begin()));

            Assert::AreEqual(vertex_c, incident_he_d[0]->twin->origin);
        }

        TEST_METHOD(DCEL_colinear_embedded_reverse)
        {
            Vec2 pA(2.f, 6.f);
            Vec2 pD(9.f, 6.f);

            Vec2 pB(4.f, 6.f);
            Vec2 pC(6.f, 6.f);

            Segment f(pA, pD);
            Segment g(pB, pC);

            DCEL dcel_reverse;

            dcel_reverse.insert_segment(g);
            dcel_reverse.insert_segment(f);

            Assert::AreEqual(size_t(6), dcel_reverse.half_edge_count(), L"DCEL has an incorrect amount of half edges.");
            Assert::AreEqual(size_t(4), dcel_reverse.vertex_count(), L"DCEL has an incorrect amount of half vertices.");

            std::vector<Vec2> correct_order = { pA, pB,pC,pD,pC,pB,pA };
            std::vector<Vec2> current_next_order;
            std::vector<Vec2> current_prev_order;

            DCEL::DCEL_Vertex* vertex_a = dcel_reverse.get_vertex_at_position(pA);

            //Test the next cycles
            const DCEL::DCEL_Half_Edge* current_half_edge = vertex_a->incident_half_edge;
            for (size_t i = 0; i < 7; i++)
            {
                current_next_order.push_back(current_half_edge->origin->position);
                current_half_edge = current_half_edge->next;
            }

            Assert::AreEqual(correct_order, current_next_order);

            //Test the prev cycles
            current_half_edge = vertex_a->incident_half_edge;
            for (size_t i = 0; i < 7; i++)
            {
                current_prev_order.push_back(current_half_edge->origin->position);
                current_half_edge = current_half_edge->prev;
            }

            Assert::AreEqual(correct_order, current_prev_order);

            DCEL::DCEL_Vertex* vertex_b = dcel_reverse.get_vertex_at_position(pB);
            DCEL::DCEL_Vertex* vertex_c = dcel_reverse.get_vertex_at_position(pC);
            DCEL::DCEL_Vertex* vertex_d = dcel_reverse.get_vertex_at_position(pD);

            auto incident_he_a = vertex_a->get_incident_half_edges();
            auto incident_he_b = vertex_b->get_incident_half_edges();
            auto incident_he_c = vertex_c->get_incident_half_edges();
            auto incident_he_d = vertex_d->get_incident_half_edges();

            Assert::AreEqual(size_t(1), incident_he_a.size());
            Assert::AreEqual(size_t(2), incident_he_b.size());
            Assert::AreEqual(size_t(2), incident_he_c.size());
            Assert::AreEqual(size_t(1), incident_he_d.size());

            Assert::AreEqual(vertex_b, incident_he_a[0]->twin->origin);

            std::vector<DCEL::DCEL_Vertex*> expected_neighbours_b = { vertex_a, vertex_c };
            std::vector<DCEL::DCEL_Vertex*> expected_neighbours_c = { vertex_b, vertex_d };
            std::vector<DCEL::DCEL_Vertex*> neighbours_b = { incident_he_b[0]->twin->origin, incident_he_b[1]->twin->origin };
            std::vector<DCEL::DCEL_Vertex*> neighbours_c = { incident_he_c[0]->twin->origin, incident_he_c[1]->twin->origin };

            Assert::IsTrue(std::is_permutation(expected_neighbours_b.begin(), expected_neighbours_b.end(), neighbours_b.begin()));
            Assert::IsTrue(std::is_permutation(expected_neighbours_c.begin(), expected_neighbours_c.end(), neighbours_c.begin()));

            Assert::AreEqual(vertex_c, incident_he_d[0]->twin->origin);
        }

        TEST_METHOD(DCEL_colinear_top_point_overlap)
        {
            Vec2 pA(2.f, 6.f);

            Vec2 pB(4.f, 6.f);
            Vec2 pC(6.f, 6.f);

            Segment f(pA, pB);
            Segment g(pA, pC);

            DCEL dcel;

            dcel.insert_segment(f);
            dcel.insert_segment(g);

            DCEL dcel_reverse;

            dcel.insert_segment(g);
            dcel.insert_segment(f);
        }

        TEST_METHOD(DCEL_colinear_bottom_point_overlap)
        {
            Vec2 pA(2.f, 6.f);

            Vec2 pB(4.f, 6.f);
            Vec2 pC(6.f, 6.f);

            Segment f(pA, pC);
            Segment g(pB, pC);

            DCEL dcel;

            dcel.insert_segment(f);
            dcel.insert_segment(g);

            DCEL dcel_reverse;

            dcel.insert_segment(g);
            dcel.insert_segment(f);
        }

        //TEST_METHOD(DCEL_colinear_both_points_overlap)
        //{
            //TODO: Infinite loop?
            //Vec2 pA(2.f, 6.f);
            //Vec2 pB(9.f, 6.f);
        //
            //Segment f(pA, pB);
            //Segment g(pA, pB);

            //DCEL dcel;
        //
            //dcel.insert_segment(f);
            //dcel.insert_segment(g);
        //
            //DCEL dcel_reverse;
        //
            //dcel.insert_segment(g);
            //dcel.insert_segment(f);
        //}

        TEST_METHOD(DCEL_Overlap_Same_DCELs)
        {
            //Implement test that overlaps the same DCEL layouts
        }

        TEST_METHOD(DCEL_large_test)
        {
            Vec2 pA(16.f, 16.f);
            Vec2 pB(20.f, 28.f);
            Vec2 pC(36.f, 28.f);
            Vec2 pD(34.f, 32.f);
            Vec2 pE(28.f, 22.f);
            Vec2 pF(28.f, 32.f);
            Vec2 pG(38.f, 30.f);
            Vec2 pH(32.f, 12.f);
            Vec2 pI(14.f, 28.f);
            Vec2 pJ(42.f, 26.f);



        }
    };
}