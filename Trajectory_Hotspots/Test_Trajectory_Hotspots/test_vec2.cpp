#include "pch.h"
#include "CppUnitTest.h"

#include "../Trajectory_Hotspots/pch.h"
#include "../Trajectory_Hotspots/vec2.h"
#include "../Trajectory_Hotspots/segment.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace TestTrajectoryHotspots
{

    TEST_CLASS(TestTrajectoryHotspotsVec2)
    {
    public:

        TEST_METHOD(vector_length)
        {
            Vec2 a(0.0f, 1.0f);
            Vec2 b(1.0f, 0.f);

            Float length_a = a.length();
            Assert::IsTrue(length_a == 1.0f);

            Vec2 c = a + b;

            Float length_c = c.length();

            Assert::IsTrue(length_c == 1.4142135623731f);
        }

        TEST_METHOD(vector_normalize)
        {
            Vec2 a(1, 1);

            a.normalize();

            Float a_length = a.length();
            Assert::IsTrue(a_length == 1.0f);


            Assert::IsTrue((a.x == (1.f / sqrtf(2.0f))));
            Assert::IsTrue((a.y == (1.f / sqrtf(2.0f))));
        }

        TEST_METHOD(vector_dot)
        {
            Vec2 a(2.f, 3.f);
            Vec2 b(-4.f, 3.f);

            Float dotAB = a.dot(b);
            Assert::IsTrue(dotAB == 1.0f);

            Vec2 c(2.f, 3.f);

            Float dotAC = a.dot(c);
            Float dotCA = c.dot(a);
            Assert::IsTrue(dotAC == 13.0f);
            Assert::IsTrue(dotCA == 13.0f);


            Assert::IsTrue(a.normalized().dot(c.normalized()) == 1.0f);

            Vec2 neg_c = -c;

            Float dot_negC_A = neg_c.dot(c);
            Assert::IsTrue(dot_negC_A == -13.0f);


            Vec2 a_big(82723.f, 78343.f);
            Vec2 b_big(48943.f, 1880880.f);

            Assert::IsTrue(a_big.dot(b_big) == 151402493629.0f);

            Vec2 a_low(.0082723f, .0078343f);
            Vec2 b_low(.0048943f, .1880880f);

            Assert::IsTrue(a_low.dot(b_low) == 0.001514024936290000188088f);
        }

        TEST_METHOD(vector_negative)
        {
            Vec2 a(3123.f, 31232.f);

            Vec2 a_neg = -a;

            a_neg.x = -3123.f;
            a_neg.y = -31232.f;
        }

        TEST_METHOD(vector_subtract)
        {
            Vec2 a(0.00032f, 3.232f);
            Vec2 b(1.0f, 1232.f);

            Vec2 c = a - b;

            Assert::IsTrue(c.x == -0.9996799f);
            Assert::IsTrue(c.y == -1228.76790f);
        }

        TEST_METHOD(vector_add)
        {
            Vec2 a(0.2367f, 0.0006f);
            Vec2 b(2381.2f, 832.232f);

            Vec2 c = a + b;

            Assert::IsTrue(c.x == 2381.4367f);
            Assert::IsTrue(c.y == 832.23260f);
        }

        TEST_METHOD(vector_between_upwards)
        {
            Vec2 A = Vec2(-4.0f, -1.0f);
            Vec2 B = Vec2(2.0f, 3.0f);
            Vec2 C = Vec2(-1.f, 1.f);
            Vec2 D = Vec2(5.f, 5.f);

            Assert::IsFalse(A.between(C, D));
            Assert::IsTrue(B.between(C, D));
            Assert::IsTrue(C.between(A, B));
            Assert::IsFalse(D.between(A, B));
        }

        TEST_METHOD(vector_between_downwards)
        {
            Vec2 A = Vec2(-3.0f, 6.0f);
            Vec2 B = Vec2(1.0f, 2.0f);
            Vec2 C = Vec2(-1.f, 4.f);
            Vec2 D = Vec2(3.f, 0.f);

            Assert::IsFalse(A.between(C, D));
            Assert::IsTrue(B.between(C, D));
            Assert::IsTrue(C.between(A, B));
            Assert::IsFalse(D.between(A, B));
        }

        TEST_METHOD(vector_between_vertical)
        {
            Vec2 A = Vec2(3.0f, -2.0f);
            Vec2 B = Vec2(3.f, 4.f);
            Vec2 C = Vec2(3.0f, 3.0f);
            Vec2 D = Vec2(3.f, 6.f);

            Assert::IsFalse(A.between(C, D));
            Assert::IsTrue(B.between(C, D));
            Assert::IsTrue(C.between(A, B));
            Assert::IsFalse(D.between(A, B));
        }

        TEST_METHOD(vector_between_horizontal)
        {
            Vec2 A = Vec2(-3.0f, 2.0f);
            Vec2 B = Vec2(7.f, 2.f);
            Vec2 C = Vec2(1.0f, 2.0f);
            Vec2 D = Vec2(8.f, 2.f);

            Assert::IsFalse(A.between(C, D));
            Assert::IsTrue(B.between(C, D));
            Assert::IsTrue(C.between(A, B));
            Assert::IsFalse(D.between(A, B));
        }

        TEST_METHOD(pseudo_angle)
        {
            //North
            Vec2 C = Vec2(3.f, 1.f);
            Vec2 A = Vec2(3.f, 2.f);

            Float pa = Vec2::pseudo_angle(C, A);

            Assert::AreEqual(Float(2.f), pa);

            //East
            C = Vec2(15.f, 3.f);
            A = Vec2(18.f, 3.f);

            pa = Vec2::pseudo_angle(C, A);

            Assert::AreEqual(Float(1.f), pa);

            //South
            C = Vec2(13.f, 1.f);
            A = Vec2(13.f, 0.f);

            pa = Vec2::pseudo_angle(C, A);

            Assert::AreEqual(Float(0.f), pa);

            //West
            C = Vec2(4.f, 5.f);
            A = Vec2(0.f, 5.f);

            pa = Vec2::pseudo_angle(C, A);

            Assert::AreEqual(Float(3.f), pa);

        }

        TEST_METHOD(pseudo_angle_ordering)
        {
            Vec2 C = Vec2(3.f, 5.f);
            Vec2 A = Vec2(3.f, 7.f);
            Vec2 B = Vec2(2.f, 2.f);

            Float ca = Vec2::pseudo_angle(C, A);
            Float cb = Vec2::pseudo_angle(C, B);

            Assert::IsTrue(ca < cb);

            C = Vec2(15.f, 3.f);
            A = Vec2(18.f, 3.f);
            B = Vec2(17.f, 2.f);

            ca = Vec2::pseudo_angle(C, A);
            cb = Vec2::pseudo_angle(C, B);

            Assert::IsTrue(ca > cb);

            C = Vec2(13.f, 1.f);
            A = Vec2(13.f, 0.f);
            B = Vec2(13.f, 0.f);

            ca = Vec2::pseudo_angle(C, A);
            cb = Vec2::pseudo_angle(C, B);

            Assert::IsTrue(ca == cb);

            C = Vec2(4.f, 5.f);
            A = Vec2(4.f, 4.f);
            B = Vec2(3.8410f, 4.0504f);

            ca = Vec2::pseudo_angle(C, A);
            cb = Vec2::pseudo_angle(C, B);

            Assert::IsTrue(ca < cb);

        }

        TEST_METHOD(pseudo_angle_order_same_bottom)
        {
            Vec2 A(4.f, 8.f);
            Vec2 B(6.f, 4.f);
            Vec2 C(8.f, 8.f);

            Float ab = Vec2::pseudo_angle(B, A);
            Float cb = Vec2::pseudo_angle(B, C);
                    
            Assert::IsTrue(ab > cb);
        }
    };


}
