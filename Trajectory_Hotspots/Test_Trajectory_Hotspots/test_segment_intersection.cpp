#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/pch.h"
#include "../Trajectory_Hotspots/vec2.h"
#include "../Trajectory_Hotspots/segment.h"
#include "../Trajectory_Hotspots/sweep_line_status_structure.h"
#include "../Trajectory_Hotspots/segment_intersection.h"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using namespace Segment_Intersection_Sweep_Line;


namespace TestTrajectoryHotspots
{
    TEST_CLASS(TestSegmentintersection)
    {
    public:

        TEST_METHOD(build_tree_with_intersection)
        {
            std::vector<Segment> test_segments;
            //test_segments.emplace_back(Vec2(3.2f, 5.28f), Vec2(4.43f, 8.73f)); //g
            //test_segments.emplace_back(Vec2(9.2f, 4.35f), Vec2(8.01f, 8.22f)); //h
            //test_segments.emplace_back(Vec2(12.0f, 4.44f), Vec2(11.29f, 8.39f)); //i
            //test_segments.emplace_back(Vec2(1.0f, 4.0f), Vec2(2.0f, 6.0f)); //j
            //test_segments.emplace_back(Vec2(9.0f, 7.0f), Vec2(6.0f, 2.0f)); //f
            test_segments.emplace_back(Vec2(9.0f, 7.0f), Vec2(6.0f, 2.0f)); //f
            test_segments.emplace_back(Vec2(9.2f, 4.35f), Vec2(8.01f, 8.22f)); //g
            //status_structure.set_line_position(line_pos);
            std::vector<Vec2> test = find_segment_intersections(test_segments);
            std::vector<Vec2> correct;
            correct.emplace_back(Vec2(8.593479498861f, 6.3224658314351f));
            Assert::AreEqual(test, correct);

        }

        TEST_METHOD(build_tree_with_four_segments_with_two_intersections)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(5.0f, 7.0f), Vec2(9.0f, 4.0f)); //f
            test_segments.emplace_back(Vec2(5.0f, 4.0f), Vec2(9.0f, 7.0f)); //g
            test_segments.emplace_back(Vec2(12.0f, 6.0f), Vec2(15.0f, 3.0f)); //h
            test_segments.emplace_back(Vec2(12.0f, 3.0f), Vec2(15.0f, 6.0f)); //i

            std::vector<Vec2> test = find_segment_intersections(test_segments);
            std::vector<Vec2> correct;
            correct.emplace_back(Vec2(7.0, 5.5f));
            correct.emplace_back(Vec2(13.5, 4.5f));
            Assert::AreEqual(test, correct);

        }

        TEST_METHOD(build_tree_with_three_segments_one_intersection)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(2.0f, 7.0f), Vec2(6.0f, 3.0f)); //f
            test_segments.emplace_back(Vec2(2.0f, 3.0f), Vec2(6.0f, 7.0f)); //g
            test_segments.emplace_back(Vec2(4.0f, 7.0f), Vec2(4.0f, 6.0f)); //h

            std::vector<Vec2> test = find_segment_intersections(test_segments);
            std::vector<Vec2> correct;
            correct.emplace_back(Vec2(4.0f, 5.0f));
            Assert::AreEqual(test, correct);
        }

        TEST_METHOD(two_segments_top_intersecting)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(4.0f, 4.0f), Vec2(2.0f, 2.0f)); //a
            test_segments.emplace_back(Vec2(4.0f, 4.0f), Vec2(5.0f, 1.0f)); //b

            std::vector<Vec2> test = find_segment_intersections(test_segments);
            std::vector<Vec2> correct;
            correct.emplace_back(Vec2(4.0f, 4.0f));
            Assert::AreEqual(test, correct);
        }

        TEST_METHOD(two_segments_bottom_intersecting)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(2.98f, 0.39f), Vec2(1.56f, 3.49f)); //a
            test_segments.emplace_back(Vec2(2.98f, 0.39f), Vec2(4.98f, 2.87f)); //b

            std::vector<Vec2> test = find_segment_intersections(test_segments);
            std::vector<Vec2> correct;
            correct.emplace_back(Vec2(2.98f, 0.39f));
            Assert::AreEqual(test, correct);
        }

        TEST_METHOD(two_segments_top_and_bottom_intersecting)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(4.26f, 4.79f), Vec2(1.84f, 6.63f)); //a
            test_segments.emplace_back(Vec2(4.98f, 2.87f), Vec2(4.26f, 4.79f)); //b

            std::vector<Vec2> test = find_segment_intersections(test_segments);
            std::vector<Vec2> correct;
            correct.emplace_back(Vec2(4.26f, 4.79f));
            Assert::AreEqual(test, correct);
        }

        TEST_METHOD(eight_segments_sixteen_intersections_three_lines_crossing)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(34.f, 10.f), Vec2(4.f, 16.f)); //l0
            test_segments.emplace_back(Vec2(28.f, 18.f), Vec2(20.f, 6.f)); //l1
            test_segments.emplace_back(Vec2(30.f, 16.f), Vec2(12.f, 4.f)); //l2
            test_segments.emplace_back(Vec2(10.843268f, 16.685045f), Vec2(8.1157327f, 7.74877901f)); //l3
            test_segments.emplace_back(Vec2(18.f, 2.f), Vec2(12.f, 16.f)); //l4
            test_segments.emplace_back(Vec2(6.0f, 12.f), Vec2(20.f, 2.f)); //l5
            test_segments.emplace_back(Vec2(6.895519f, 12.521965f), Vec2(24.480944f, 8.50244f)); //l6
            test_segments.emplace_back(Vec2(18.f, 16.f), Vec2(16.f, 2.f)); //l7

            std::vector<Vec2> test = find_segment_intersections(test_segments);
            std::vector<Vec2> correct;
            correct.reserve(16);

            correct.emplace_back(Vec2(10.2524992273747f, 14.7495001545251f));
            correct.emplace_back(Vec2(12.75f, 14.25f));
            correct.emplace_back(Vec2(17.6111111111111f, 13.2777777777778f));
            correct.emplace_back(Vec2(24.f, 12.f));
            correct.emplace_back(Vec2(9.3980223384619f, 11.9499643385698f));
            correct.emplace_back(Vec2(14.2067927050596f, 10.8508170215275f));
            correct.emplace_back(Vec2(17.1677191367719f, 10.1740339574032f));
            correct.emplace_back(Vec2(8.8023418718738f, 9.9983272343759f));
            correct.emplace_back(Vec2(20.2159450830875f, 9.4772967220583f));
            correct.emplace_back(Vec2(22.0402140252922f, 9.0603210379384f));
            correct.emplace_back(Vec2(16.7368421052632f, 7.1578947368421f));
            correct.emplace_back(Vec2(16.f, 6.6666666666667f));
            correct.emplace_back(Vec2(14.6896551724138f, 5.7931034482759f));
            correct.emplace_back(Vec2(16.5f, 5.5f));
            correct.emplace_back(Vec2(16.3703703703704f, 4.5925925925926f));
            correct.emplace_back(Vec2(17.1176470588235f, 4.0588235294118f));

            Assert::AreEqual(test, correct);
        }

        TEST_METHOD(four_segments_on_one_point_one_horizontal)
        {
            namespace Sweep = Segment_Intersection_Sweep_Line;

            Sweep::Event_Queue event_queue;

            //All segments intersect on a point with one segment horizontal
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(12.f, 4.f), Vec2(4.f, 10.f));
            test_segments.emplace_back(Vec2(4.f, 4.f), Vec2(12.f, 10.f));
            test_segments.emplace_back(Vec2(12.f, 5.f), Vec2(4.f, 9.f));
            test_segments.emplace_back(Vec2(11.f, 7.f), Vec2(2.f, 7.f)); //Horizontal

            for (int i = 0; i < test_segments.size(); i++)
            {
                std::vector<int>& top_event_segments = event_queue[*test_segments.at(i).get_top_point()];
                top_event_segments.push_back(i);

                //If it doesn't exist, create a bottom event
                event_queue[*test_segments.at(i).get_bottom_point()];
            }

            Sweep::Sweep_Line_Status_structure<Segment> status_structure(event_queue.begin()->first);

            int event_count = 0;
            std::vector<int> test;
            while (!event_queue.empty())
            {
                Sweep::Handle_Event(status_structure, event_queue, test_segments, event_queue.begin()->first, event_queue.begin()->second, [&event_count, &test](const std::vector<int>& intersecting_segments, const std::vector<int>& top_segments)
                    {
                        ++event_count;

                        if (event_count == 5)
                        {
                            test = intersecting_segments;

                            Assert::IsTrue(top_segments.empty());
                        }
                    });

                event_queue.erase(event_queue.begin());
            }

            std::vector<int> correct{ 3, 2, 0, 1 };

            Assert::IsTrue(test == correct);
        }
    };
}