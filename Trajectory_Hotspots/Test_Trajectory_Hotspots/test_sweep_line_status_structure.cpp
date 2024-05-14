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

    TEST_CLASS(TestSweepLineStatusStructure)
    {
    public:
        TEST_METHOD(construction)
        {
            Sweep_Line_Status_structure<Segment> sweep_line;

            Assert::IsTrue(sweep_line.empty());
        }

        TEST_METHOD(insert_non_intersecting_segments)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(5.183123123f, 2.f), Vec2(5.22f, 11.27898f));    //3
            test_segments.emplace_back(Vec2(4.3813123f, 9.77978f), Vec2(2.8f, 3.5798f));    //2
            test_segments.emplace_back(Vec2(7.0267869f, 8.46879f), Vec2(7.24f, 0.7792f));   //4
            test_segments.emplace_back(Vec2(10.48974f, 10.287976f), Vec2(9.16f, 2.97974f)); //6
            test_segments.emplace_back(Vec2(1.f, 11.7932f), Vec2(1.9f, 1.78964f));          //1
            test_segments.emplace_back(Vec2(14.47892f, 11.08984f), Vec2(8.7798f, 1.5794f)); //7
            test_segments.emplace_back(Vec2(7.6784f, 10.2974f), Vec2(8.27894f, 0.7975f));   //5


            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());

            for (int i = 0; i < test_segments.size(); i++)
            {
                Vec2 event_point = *test_segments.at(i).get_top_point();
                status_structure.set_current_event_position(event_point);
                int left_node = -1;
                int right_node = -1;
                status_structure.insert(test_segments, i, left_node, right_node);

                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }

            for (int i = 0; i < test_segments.size(); i++)
            {
                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }
        }

        TEST_METHOD(remove)
        {
            //TODO: Change test segments
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(5.183123123f, 2.f), Vec2(5.22f, 11.27898f));    //3
            test_segments.emplace_back(Vec2(4.3813123f, 9.77978f), Vec2(2.8f, 3.5798f));    //2
            test_segments.emplace_back(Vec2(7.0267869f, 8.46879f), Vec2(7.24f, 0.7792f));   //4
            test_segments.emplace_back(Vec2(10.48974f, 10.287976f), Vec2(9.16f, 2.97974f)); //6
            test_segments.emplace_back(Vec2(1.f, 11.7932f), Vec2(1.9f, 1.78964f));          //1
            test_segments.emplace_back(Vec2(14.47892f, 11.08984f), Vec2(8.7798f, 1.5794f)); //7
            test_segments.emplace_back(Vec2(7.6784f, 10.2974f), Vec2(8.27894f, 0.7975f));   //5


            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());

            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.set_current_event_position(*test_segments.at(i).get_top_point());
                int left_node;
                int right_node;
                status_structure.insert(test_segments, i, left_node, right_node);
            }
            int left_segment = -1;
            int right_segment = -1;
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.remove(test_segments, i, left_segment, right_segment);
                Assert::IsFalse(status_structure.contains(test_segments, &test_segments.at(i)));
            }

            Assert::IsTrue(status_structure.empty());
        }

        TEST_METHOD(contains)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(3, 8), Vec2(7, 3));
            test_segments.emplace_back(Vec2(3, 2), Vec2(6, 9));

            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());

            for (int i = 0; i < test_segments.size(); i++)
            {
                int left_node;
                int right_node;
                status_structure.insert(test_segments, i, left_node, right_node);
            }

            for (int i = 0; i < test_segments.size(); i++)
            {
                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }

            std::vector<Segment> contain_segment;
            contain_segment.emplace_back(Vec2(3, 8), Vec2(7, 3));
            std::vector<Segment> not_contained_segment;
            not_contained_segment.emplace_back(Vec2(1, 8), Vec2(7, 3));

            Assert::IsTrue(status_structure.contains(test_segments, &contain_segment.at(0)));
            Assert::IsFalse(status_structure.contains(test_segments, &not_contained_segment.at(0)));

        }

        TEST_METHOD(destruction)
        {

        }

        TEST_METHOD(get_left_neighbour_crash_case)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(7, 2), Vec2(8, 6));
            test_segments.emplace_back(Vec2(3, 3), Vec2(5, 6));

            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());

            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.set_current_event_position(*test_segments.at(i).get_top_point());
                int left_node = -1;
                int right_node = -1;
                status_structure.insert(test_segments, i, left_node, right_node);

                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }

            test_segments.emplace_back(Vec2(12, 2), Vec2(10, 5));
            int left_node = -1;
            int right_node = -1;
            status_structure.set_current_event_position(*test_segments.at(2).get_top_point());
            status_structure.insert(test_segments, 2, left_node, right_node);

            std::vector<Segment> test;
            test.emplace_back(Vec2(7, 2), Vec2(8, 6));
            int test12 = 0;
            int test13 = -1;
            Assert::AreEqual(test12, left_node);

        }

        TEST_METHOD(get_left_neighbour)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(3, 3), Vec2(5, 6));
            test_segments.emplace_back(Vec2(7, 2), Vec2(8, 6));


            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());
            int left_node = -1;
            int right_node = -1;
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.set_current_event_position(*test_segments.at(i).get_top_point());

                status_structure.insert(test_segments, i, left_node, right_node);

                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }
            int first_element = 0;
            Assert::AreEqual(first_element, left_node);

        }

        TEST_METHOD(get_right_neighbour_is_empty)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(7, 2), Vec2(8, 6));
            test_segments.emplace_back(Vec2(3, 3), Vec2(5, 6));

            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());
            int left_node = -1;
            int right_node = -1;
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.set_current_event_position(*test_segments.at(i).get_top_point());

                status_structure.insert(test_segments, i, left_node, right_node);

                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }
            int first_element = -1;
            Assert::AreEqual(first_element, left_node);

        }

        TEST_METHOD(get_left_neighbour_case_1)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(3.2f, 5.28f), Vec2(4.43f, 8.73f)); //g
            test_segments.emplace_back(Vec2(9.2f, 4.35f), Vec2(8.01f, 8.22f)); //h
            test_segments.emplace_back(Vec2(12.f, 4.44f), Vec2(11.29f, 8.39f)); //i
            test_segments.emplace_back(Vec2(1.f, 4.f), Vec2(2.f, 6.f)); //j
            test_segments.emplace_back(Vec2(6.f, 6.f), Vec2(6.f, 2.f)); //f


            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());
            int left_node = -1;
            int right_node = -1;
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.set_current_event_position(*test_segments.at(i).get_top_point());

                status_structure.insert(test_segments, i, left_node, right_node);

                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }
            int left_element = 0;
            Assert::AreEqual(left_element, left_node);

        }

        TEST_METHOD(get_left_and_right_neighbour_case_1)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(3.2f, 5.28f), Vec2(4.43f, 8.73f)); //g
            test_segments.emplace_back(Vec2(9.2f, 4.35f), Vec2(8.01f, 8.22f)); //h
            test_segments.emplace_back(Vec2(12.f, 4.44f), Vec2(11.29f, 8.39f)); //i
            test_segments.emplace_back(Vec2(1.f, 4.f), Vec2(2, 6)); //j
            test_segments.emplace_back(Vec2(6.f, 6.f), Vec2(6, 2)); //f


            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());
            int left_node = -1;
            int right_node = -1;
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.set_current_event_position(*test_segments.at(i).get_top_point());

                status_structure.insert(test_segments, i, left_node, right_node);

                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }
            int left_element = 0;
            Assert::AreEqual(left_element, left_node);
            int right_element = 1;
            Assert::AreEqual(right_element, right_node);

        }

        TEST_METHOD(get_left_and_right_neighbour_case_2)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(5.f, 5.f), Vec2(6.f, 10.f)); //f
            test_segments.emplace_back(Vec2(7.f, 3.f), Vec2(7.f, 8.f)); //h
            test_segments.emplace_back(Vec2(3.04f, 3.13f), Vec2(3.f, 8.f)); //i
            test_segments.emplace_back(Vec2(12.f, 10.f), Vec2(14.f, 5.f)); //j
            test_segments.emplace_back(Vec2(9.f, 8.f), Vec2(10.f, 3.f)); //g


            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());
            int left_node = -1;
            int right_node = -1;
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.set_current_event_position(*test_segments.at(i).get_top_point());

                status_structure.insert(test_segments, i, left_node, right_node);

                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }
            int left_element = 1;
            Assert::AreEqual(left_element, left_node);
            int right_element = 3;
            Assert::AreEqual(right_element, right_node);

        }

        //make a tree with root having only one left node
        TEST_METHOD(remove_root_one_left_node)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(3, 3), Vec2(5, 6));
            test_segments.emplace_back(Vec2(7, 2), Vec2(8, 6));
            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.set_current_event_position(*test_segments.at(i).get_top_point());
                int left_node;
                int right_node;
                status_structure.insert(test_segments, i, left_node, right_node);

                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }
            int left_segment = -1;
            int right_segment = -1;
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.remove(test_segments, i, left_segment, right_segment);
                Assert::IsFalse(status_structure.contains(test_segments, &test_segments.at(i)));
            }

            Assert::IsTrue(status_structure.empty());
        }

        // make a tree with root having one left node and one right node
        TEST_METHOD(remove_with_one_left_one_right)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(3, 3), Vec2(5, 6));
            test_segments.emplace_back(Vec2(7, 2), Vec2(8, 6));
            test_segments.emplace_back(Vec2(12, 2), Vec2(10, 5));
            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.set_current_event_position(*test_segments.at(i).get_top_point());
                int left_node;
                int right_node;
                status_structure.insert(test_segments, i, left_node, right_node);

                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }
            int left_segment = -1;
            int right_segment = -1;
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.remove(test_segments, i, left_segment, right_segment);
                Assert::IsFalse(status_structure.contains(test_segments, &test_segments.at(i)));
            }

            Assert::IsTrue(status_structure.empty());
        }

        //make a tree with root adding to left node of root
        TEST_METHOD(remove_with_one_left_and_one_left)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(3, 3), Vec2(5, 6));
            test_segments.emplace_back(Vec2(7, 2), Vec2(8, 6));
            test_segments.emplace_back(Vec2(1, 4), Vec2(2, 6));
            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.set_current_event_position(*test_segments.at(i).get_top_point());
                int left_node;
                int right_node;
                status_structure.insert(test_segments, i, left_node, right_node);

                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }
            int left_segment = -1;
            int right_segment = -1;
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.remove(test_segments, i, left_segment, right_segment);
                Assert::IsFalse(status_structure.contains(test_segments, &test_segments.at(i)));
            }

            Assert::IsTrue(status_structure.empty());
        }

        TEST_METHOD(remove_large_tree)
        {
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(3, 3), Vec2(5, 6)); //f
            test_segments.emplace_back(Vec2(7, 2), Vec2(8, 6)); //g
            test_segments.emplace_back(Vec2(12, 2), Vec2(10, 5)); //h
            test_segments.emplace_back(Vec2(1, 4), Vec2(2, 6)); //i
            test_segments.emplace_back(Vec2(4.64f, 1.05f), Vec2(1.38f, 2.63f)); //j
            test_segments.emplace_back(Vec2(7.34f, 0.49f), Vec2(9.62f, 2.95f)); //k
            test_segments.emplace_back(Vec2(6.3f, 7.81f), Vec2(10.92f, 7.45f)); //n
            test_segments.emplace_back(Vec2(1.38f, 7.55f), Vec2(5.4f, 4.39f)); //m
            test_segments.emplace_back(Vec2(9.42f, 5.55f), Vec2(8.0f, 9.0f));  // l
            test_segments.emplace_back(Vec2(6.46f, 5.13f), Vec2(4.44f, 9.23f)); //p
            test_segments.emplace_back(Vec2(6.42f, 3.87f), Vec2(5.5f, 2.49f)); //q

            Sweep_Line_Status_structure<Segment> status_structure(*test_segments.at(0).get_top_point());
            for (int i = 0; i < test_segments.size(); i++)
            {
                int left_node;
                int right_node;
                status_structure.insert(test_segments, i, left_node, right_node);

                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }
            int left_segment = -1;
            int right_segment = -1;
            for (int i = 0; i < test_segments.size(); i++)
            {
                status_structure.remove(test_segments, i, left_segment, right_segment);
                Assert::IsFalse(status_structure.contains(test_segments, &test_segments.at(i)));
            }

            Assert::IsTrue(status_structure.empty());
        }

        TEST_METHOD(horizontal_last)
        {
            //All segments intersect on a point with one segment horizontal
            //The status structure should store the horizontal last at the intersection point
            std::vector<Segment> test_segments;
            test_segments.emplace_back(Vec2(12.f, 4.f), Vec2(4.f, 10.f));
            test_segments.emplace_back(Vec2(4.f, 4.f), Vec2(12.f, 10.f));
            test_segments.emplace_back(Vec2(12.f, 5.f), Vec2(4.f, 9.f));
            test_segments.emplace_back(Vec2(11.f, 7.f), Vec2(2.f, 7.f)); //Horizontal

            Sweep_Line_Status_structure<Segment> status_structure(*test_segments[0].get_top_point());

            int left_node;
            int right_node;

            for (int i = 0; i < test_segments.size() - 1; i++)
            {
                status_structure.set_current_event_position(*test_segments[i].get_top_point());

                status_structure.insert(test_segments, i, left_node, right_node);

                Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(i)));
            }

            status_structure.set_current_event_position(Vec2(8.f, 7.f));

            status_structure.insert(test_segments, 3, left_node, right_node);

            Assert::IsTrue(status_structure.contains(test_segments, &test_segments.at(3)));

            int left_neighbour;
            int right_neighbour;
            std::vector<int> segments_on_point = status_structure.get_all_nodes_on_point(test_segments, left_neighbour, right_neighbour);

            Assert::AreEqual(segments_on_point.size(), (size_t)4);
            Assert::AreEqual(segments_on_point[3], 3);
        }
    };
}