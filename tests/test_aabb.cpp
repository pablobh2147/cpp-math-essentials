#include <gtest/gtest.h>

#include <limits>
#include <span>
#include <vector>

#include "BoundingBox.hpp"

#ifdef MATH_ESSENTIALS_NAMESPACE
#define NAMESPACE_NAME MATH_ESSENTIALS_NAMESPACE
#else
#define NAMESPACE_NAME
#endif

#ifdef MATH_ESSENTIALS_NAMESPACE
using namespace NAMESPACE_NAME;
#endif

constexpr float ABS_TOLERANCE = std::numeric_limits<float>::epsilon();

// ---------------------- Constructors ----------------------

TEST(AABB, Constructor_Default) {
    AABBf box;
    EXPECT_EQ(box.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box.Max(), Vector3f(0.0f, 0.0f, 0.0f));
}

TEST(AABB, Constructor_WithValidBounds) {
    Vector3f min(1.0f, 2.0f, 3.0f);
    Vector3f max(4.0f, 5.0f, 6.0f);
    AABBf box(min, max);

    EXPECT_EQ(box.Min(), min);
    EXPECT_EQ(box.Max(), max);
}

TEST(AABB, Constructor_WithInvertedBounds) {
    Vector3f min(4.0f, 5.0f, 6.0f);
    Vector3f max(1.0f, 2.0f, 3.0f);
    AABBf box(min, max);

    EXPECT_EQ(box.Min(), Vector3f(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(box.Max(), Vector3f(4.0f, 5.0f, 6.0f));
}

TEST(AABB, Constructor_WithEqualBounds) {
    Vector3f point(2.0f, 3.0f, 4.0f);
    AABBf box(point, point);

    EXPECT_EQ(box.Min(), point);
    EXPECT_EQ(box.Max(), point);
}

TEST(AABB, Constructor_WithNegativeCoordinates) {
    Vector3f min(-5.0f, -3.0f, -1.0f);
    Vector3f max(-1.0f, 0.0f, 2.0f);
    AABBf box(min, max);

    EXPECT_EQ(box.Min(), min);
    EXPECT_EQ(box.Max(), max);
}

TEST(AABB, Constructor_IntegerType) {
    Vector3<int32_t> min(1, 2, 3);
    Vector3<int32_t> max(4, 5, 6);
    AABB<int32_t> box(min, max);

    EXPECT_EQ(box.Min(), min);
    EXPECT_EQ(box.Max(), max);
}

// ---------------------- Getters ----------------------

TEST(AABB, Center_ValidBox) {
    Vector3f min(0.0f, 0.0f, 0.0f);
    Vector3f max(10.0f, 20.0f, 30.0f);
    AABBf box(min, max);

    Vector3f expected(5.0f, 10.0f, 15.0f);
    EXPECT_NEAR(box.Center().x, expected.x, ABS_TOLERANCE);
    EXPECT_NEAR(box.Center().y, expected.y, ABS_TOLERANCE);
    EXPECT_NEAR(box.Center().z, expected.z, ABS_TOLERANCE);
}

TEST(AABB, Center_PointBox) {
    Vector3f point(2.0f, 3.0f, 4.0f);
    AABBf box(point, point);

    EXPECT_NEAR(box.Center().x, point.x, ABS_TOLERANCE);
    EXPECT_NEAR(box.Center().y, point.y, ABS_TOLERANCE);
    EXPECT_NEAR(box.Center().z, point.z, ABS_TOLERANCE);
}

TEST(AABB, Center_NegativeCoordinates) {
    Vector3f range(10.0f, 20.0f, 30.0f);
    AABBf box(-range, range);

    Vector3f expected(0.0f, 0.0f, 0.0f);
    EXPECT_NEAR(box.Center().x, expected.x, ABS_TOLERANCE);
    EXPECT_NEAR(box.Center().y, expected.y, ABS_TOLERANCE);
    EXPECT_NEAR(box.Center().z, expected.z, ABS_TOLERANCE);
}

TEST(AABB, Size_ValidBox) {
    Vector3f min(1.0f, 2.0f, 3.0f);
    Vector3f max(4.0f, 6.0f, 9.0f);
    AABBf box(min, max);

    Vector3f expected(3.0f, 4.0f, 6.0f);
    EXPECT_NEAR(box.Size().x, expected.x, ABS_TOLERANCE);
    EXPECT_NEAR(box.Size().y, expected.y, ABS_TOLERANCE);
    EXPECT_NEAR(box.Size().z, expected.z, ABS_TOLERANCE);
}

TEST(AABB, Size_PointBox) {
    Vector3f point(2.0f, 3.0f, 4.0f);
    AABBf box(point, point);

    Vector3f expected(0.0f, 0.0f, 0.0f);
    EXPECT_NEAR(box.Size().x, expected.x, ABS_TOLERANCE);
    EXPECT_NEAR(box.Size().y, expected.y, ABS_TOLERANCE);
    EXPECT_NEAR(box.Size().z, expected.z, ABS_TOLERANCE);
}

TEST(AABB, Extents_ValidBox) {
    Vector3f min(0.0f, 0.0f, 0.0f);
    Vector3f max(10.0f, 20.0f, 30.0f);
    AABBf box(min, max);

    Vector3f expected(5.0f, 10.0f, 15.0f);
    EXPECT_NEAR(box.Extents().x, expected.x, ABS_TOLERANCE);
    EXPECT_NEAR(box.Extents().y, expected.y, ABS_TOLERANCE);
    EXPECT_NEAR(box.Extents().z, expected.z, ABS_TOLERANCE);
}

TEST(AABB, Extents_PointBox) {
    Vector3f point(2.0f, 3.0f, 4.0f);
    AABBf box(point, point);

    Vector3f expected(0.0f, 0.0f, 0.0f);
    EXPECT_NEAR(box.Extents().x, expected.x, ABS_TOLERANCE);
    EXPECT_NEAR(box.Extents().y, expected.y, ABS_TOLERANCE);
    EXPECT_NEAR(box.Extents().z, expected.z, ABS_TOLERANCE);
}

TEST(AABB, Volume_ValidBox) {
    Vector3f min(0.0f, 0.0f, 0.0f);
    Vector3f max(2.0f, 3.0f, 4.0f);
    AABBf box(min, max);

    float expected = 2.0f * 3.0f * 4.0f;
    EXPECT_NEAR(box.Volume(), expected, ABS_TOLERANCE);
}

TEST(AABB, Volume_PointBox) {
    Vector3f point(2.0f, 3.0f, 4.0f);
    AABBf box(point, point);

    EXPECT_FLOAT_EQ(box.Volume(), 0.0f);
}

TEST(AABB, Volume_FlatBox_XY) {
    Vector3f min(0.0f, 0.0f, 5.0f);
    Vector3f max(2.0f, 3.0f, 5.0f);
    AABBf box(min, max);

    EXPECT_FLOAT_EQ(box.Volume(), 0.0f);
}

TEST(AABB, Volume_FlatBox_XZ) {
    Vector3f min(0.0f, 5.0f, 0.0f);
    Vector3f max(2.0f, 5.0f, 3.0f);
    AABBf box(min, max);

    EXPECT_FLOAT_EQ(box.Volume(), 0.0f);
}

TEST(AABB, Volume_FlatBox_YZ) {
    Vector3f min(5.0f, 0.0f, 0.0f);
    Vector3f max(5.0f, 2.0f, 3.0f);
    AABBf box(min, max);

    EXPECT_FLOAT_EQ(box.Volume(), 0.0f);
}

TEST(AABB, SurfaceArea_ValidBox) {
    Vector3f min(0.0f, 0.0f, 0.0f);
    Vector3f max(2.0f, 3.0f, 4.0f);
    AABBf box(min, max);

    float expected = 2.0f * (2.0f * 3.0f + 3.0f * 4.0f + 4.0f * 2.0f);
    EXPECT_NEAR(box.SurfaceArea(), expected, ABS_TOLERANCE);
}

TEST(AABB, SurfaceArea_PointBox) {
    Vector3f point(2.0f, 3.0f, 4.0f);
    AABBf box(point, point);

    EXPECT_FLOAT_EQ(box.SurfaceArea(), 0.0f);
}

TEST(AABB, SurfaceArea_FlatBox) {
    Vector3f min(0.0f, 0.0f, 5.0f);
    Vector3f max(2.0f, 3.0f, 5.0f);
    AABBf box(min, max);

    float expected = 2.0f * (2.0f * 3.0f);
    EXPECT_NEAR(box.SurfaceArea(), expected, ABS_TOLERANCE);
}

// ---------------------- Setters ----------------------

TEST(AABB, SetMin_UpdatesBounds) {
    AABBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    Vector3f new_min(0.0f, 1.0f, 2.0f);
    box.SetMin(new_min);

    EXPECT_EQ(box.Min(), new_min);
    EXPECT_EQ(box.Max(), Vector3f(4.0f, 5.0f, 6.0f));
}

TEST(AABB, SetMin_WithInversion) {
    AABBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    Vector3f new_min(5.0f, 6.0f, 7.0f);
    box.SetMin(new_min);

    EXPECT_EQ(box.Min(), Vector3f(4.0f, 5.0f, 6.0f));
    EXPECT_EQ(box.Max(), Vector3f(5.0f, 6.0f, 7.0f));
}

TEST(AABB, SetMax_UpdatesBounds) {
    AABBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    Vector3f new_max(5.0f, 6.0f, 7.0f);
    box.SetMax(new_max);

    EXPECT_EQ(box.Min(), Vector3f(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(box.Max(), new_max);
}

TEST(AABB, SetMax_WithInversion) {
    AABBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    Vector3f new_max(0.0f, 1.0f, 2.0f);
    box.SetMax(new_max);

    EXPECT_EQ(box.Min(), Vector3f(0.0f, 1.0f, 2.0f));
    EXPECT_EQ(box.Max(), Vector3f(1.0f, 2.0f, 3.0f));
}

TEST(AABB, SetBounds_NormalCase) {
    AABBf box;
    Vector3f min(1.0f, 2.0f, 3.0f);
    Vector3f max(4.0f, 5.0f, 6.0f);
    box.SetBounds(min, max);

    EXPECT_EQ(box.Min(), min);
    EXPECT_EQ(box.Max(), max);
}

TEST(AABB, SetBounds_InvertedInputs) {
    AABBf box;
    Vector3f min(4.0f, 5.0f, 6.0f);
    Vector3f max(1.0f, 2.0f, 3.0f);
    box.SetBounds(min, max);

    EXPECT_EQ(box.Min(), Vector3f(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(box.Max(), Vector3f(4.0f, 5.0f, 6.0f));
}

TEST(AABB, SetBounds_EqualInputs) {
    AABBf box;
    Vector3f point(2.0f, 3.0f, 4.0f);
    box.SetBounds(point, point);

    EXPECT_EQ(box.Min(), point);
    EXPECT_EQ(box.Max(), point);
}

TEST(AABB, SetBounds_IntegerType) {
    AABB<int32_t> box;
    Vector3<int32_t> min(1, 2, 3);
    Vector3<int32_t> max(4, 5, 6);
    box.SetBounds(min, max);

    EXPECT_EQ(box.Min(), min);
    EXPECT_EQ(box.Max(), max);
}

// ---------------------- Merge ----------------------

TEST(AABB, Merge_TwoSeparateBoxes) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f));
    AABBf box2(Vector3f(2.0f, 2.0f, 2.0f), Vector3f(3.0f, 3.0f, 3.0f));

    box1.Merge(box2);

    EXPECT_EQ(box1.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box1.Max(), Vector3f(3.0f, 3.0f, 3.0f));
}

TEST(AABB, Merge_OverlappingBoxes) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    AABBf box2(Vector3f(1.0f, 1.0f, 1.0f), Vector3f(3.0f, 3.0f, 3.0f));

    box1.Merge(box2);

    EXPECT_EQ(box1.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box1.Max(), Vector3f(3.0f, 3.0f, 3.0f));
}

TEST(AABB, Merge_ContainedBox) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    AABBf box2(Vector3f(1.0f, 1.0f, 1.0f), Vector3f(2.0f, 2.0f, 2.0f));

    box1.Merge(box2);

    EXPECT_EQ(box1.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box1.Max(), Vector3f(5.0f, 5.0f, 5.0f));
}

TEST(AABB, Merge_WithPointBox) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    AABBf box2(Vector3f(3.0f, 3.0f, 3.0f), Vector3f(3.0f, 3.0f, 3.0f));

    box1.Merge(box2);

    EXPECT_EQ(box1.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box1.Max(), Vector3f(3.0f, 3.0f, 3.0f));
}

TEST(AABB, Merge_NullBoxWithValidBox) {
    AABBf box1;
    AABBf box2(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));

    box1.Merge(box2);

    EXPECT_EQ(box1.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box1.Max(), Vector3f(4.0f, 5.0f, 6.0f));
}

TEST(AABB, Merge_IntegerType) {
    AABB<int32_t> box1(Vector3<int32_t>(0, 0, 0), Vector3<int32_t>(2, 2, 2));
    AABB<int32_t> box2(Vector3<int32_t>(1, 1, 1), Vector3<int32_t>(3, 3, 3));

    box1.Merge(box2);

    EXPECT_EQ(box1.Min(), Vector3<int32_t>(0, 0, 0));
    EXPECT_EQ(box1.Max(), Vector3<int32_t>(3, 3, 3));
}

TEST(AABB, MergeStatic_EmptySpan) {
    std::vector<AABBf> boxes;
    AABBf result = AABBf::Merge(boxes);
}

TEST(AABB, MergeStatic_SingleBox) {
    std::vector<AABBf> boxes = {
        AABBf(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f))};

    AABBf result = AABBf::Merge(boxes);

    EXPECT_EQ(result.Min(), Vector3f(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(result.Max(), Vector3f(4.0f, 5.0f, 6.0f));
}

TEST(AABB, MergeStatic_TwoBoxes) {
    std::vector<AABBf> boxes = {
        AABBf(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f)),
        AABBf(Vector3f(2.0f, 2.0f, 2.0f), Vector3f(3.0f, 3.0f, 3.0f)),
    };

    AABBf result = AABBf::Merge(boxes);

    EXPECT_EQ(result.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(result.Max(), Vector3f(3.0f, 3.0f, 3.0f));
}

TEST(AABB, MergeStatic_MultipleBoxes) {
    std::vector<AABBf> boxes = {
        AABBf(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f)),
        AABBf(Vector3f(2.0f, 2.0f, 2.0f), Vector3f(3.0f, 3.0f, 3.0f)),
        AABBf(Vector3f(-1.0f, -1.0f, -1.0f), Vector3f(0.5f, 0.5f, 0.5f)),
        AABBf(Vector3f(1.5f, 1.5f, 1.5f), Vector3f(2.5f, 2.5f, 2.5f)),
    };

    AABBf result = AABBf::Merge(boxes);

    EXPECT_EQ(result.Min(), Vector3f(-1.0f, -1.0f, -1.0f));
    EXPECT_EQ(result.Max(), Vector3f(3.0f, 3.0f, 3.0f));
}

TEST(AABB, MergeStatic_OverlappingBoxes) {
    std::vector<AABBf> boxes = {
        AABBf(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f)),
        AABBf(Vector3f(1.0f, 1.0f, 1.0f), Vector3f(3.0f, 3.0f, 3.0f)),
        AABBf(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(1.5f, 1.5f, 1.5f)),
    };

    AABBf result = AABBf::Merge(boxes);

    EXPECT_EQ(result.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(result.Max(), Vector3f(3.0f, 3.0f, 3.0f));
}

TEST(AABB, MergeStatic_WithPointBoxes) {
    std::vector<AABBf> boxes = {
        AABBf(Vector3f(1.0f, 1.0f, 1.0f), Vector3f(1.0f, 1.0f, 1.0f)),
        AABBf(Vector3f(2.0f, 2.0f, 2.0f), Vector3f(2.0f, 2.0f, 2.0f)),
        AABBf(Vector3f(3.0f, 3.0f, 3.0f), Vector3f(3.0f, 3.0f, 3.0f)),
    };

    AABBf result = AABBf::Merge(boxes);

    EXPECT_EQ(result.Min(), Vector3f(1.0f, 1.0f, 1.0f));
    EXPECT_EQ(result.Max(), Vector3f(3.0f, 3.0f, 3.0f));
}

TEST(AABB, MergeStatic_IntegerType) {
    std::vector<AABB<int32_t>> boxes = {
        AABB<int32_t>(Vector3<int32_t>(0, 0, 0), Vector3<int32_t>(2, 2, 2)),
        AABB<int32_t>(Vector3<int32_t>(1, 1, 1), Vector3<int32_t>(3, 3, 3)),
        AABB<int32_t>(Vector3<int32_t>(-1, -1, -1), Vector3<int32_t>(1, 1, 1)),
    };

    AABB<int32_t> result = AABB<int32_t>::Merge(boxes);

    EXPECT_EQ(result.Min(), Vector3<int32_t>(-1, -1, -1));
    EXPECT_EQ(result.Max(), Vector3<int32_t>(3, 3, 3));
}

// ---------------------- Enclose ----------------------

TEST(AABB, Enclose_SinglePoint) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f));
    Vector3f point(2.0f, 2.0f, 2.0f);

    box.Enclose(point);

    EXPECT_EQ(box.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box.Max(), Vector3f(2.0f, 2.0f, 2.0f));
}

TEST(AABB, Enclose_PointInsideBox) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    Vector3f point(2.0f, 3.0f, 4.0f);

    box.Enclose(point);

    EXPECT_EQ(box.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box.Max(), Vector3f(5.0f, 5.0f, 5.0f));
}

TEST(AABB, Enclose_PointExpandsMin) {
    AABBf box(Vector3f(1.0f, 1.0f, 1.0f), Vector3f(3.0f, 3.0f, 3.0f));
    Vector3f point(0.0f, 0.5f, 0.2f);

    box.Enclose(point);

    EXPECT_EQ(box.Min(), Vector3f(0.0f, 0.5f, 0.2f));
    EXPECT_EQ(box.Max(), Vector3f(3.0f, 3.0f, 3.0f));
}

TEST(AABB, Enclose_PointExpandsMax) {
    AABBf box(Vector3f(1.0f, 1.0f, 1.0f), Vector3f(3.0f, 3.0f, 3.0f));
    Vector3f point(4.0f, 3.5f, 5.0f);

    box.Enclose(point);

    EXPECT_EQ(box.Min(), Vector3f(1.0f, 1.0f, 1.0f));
    EXPECT_EQ(box.Max(), Vector3f(4.0f, 3.5f, 5.0f));
}

TEST(AABB, Enclose_NegativePoint) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Vector3f point(-1.0f, -2.0f, -3.0f);

    box.Enclose(point);

    EXPECT_EQ(box.Min(), Vector3f(-1.0f, -2.0f, -3.0f));
    EXPECT_EQ(box.Max(), Vector3f(2.0f, 2.0f, 2.0f));
}

TEST(AABB, Enclose_MultiplePoints) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f));

    box.Enclose(Vector3f(2.0f, 0.5f, 0.5f));
    box.Enclose(Vector3f(0.5f, 3.0f, 0.5f));
    box.Enclose(Vector3f(0.5f, 0.5f, 4.0f));

    EXPECT_EQ(box.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box.Max(), Vector3f(2.0f, 3.0f, 4.0f));
}

TEST(AABB, Enclose_IntegerType) {
    AABB<int32_t> box(Vector3<int32_t>(0, 0, 0), Vector3<int32_t>(2, 2, 2));
    Vector3<int32_t> point(3, 4, 5);

    box.Enclose(point);

    EXPECT_EQ(box.Min(), Vector3<int32_t>(0, 0, 0));
    EXPECT_EQ(box.Max(), Vector3<int32_t>(3, 4, 5));
}

TEST(AABB, EncloseStatic_EmptySpan) {
    std::vector<Vector3f> points;
    AABBf result = AABBf::Enclose(points);
}

TEST(AABB, EncloseStatic_SinglePoint) {
    std::vector<Vector3f> points = {
        Vector3f(2.0f, 3.0f, 4.0f),
    };

    AABBf result = AABBf::Enclose(points);

    EXPECT_EQ(result.Min(), Vector3f(2.0f, 3.0f, 4.0f));
    EXPECT_EQ(result.Max(), Vector3f(2.0f, 3.0f, 4.0f));
}

TEST(AABB, EncloseStatic_TwoPoints) {
    std::vector<Vector3f> points = {
        Vector3f(1.0f, 2.0f, 3.0f),
        Vector3f(4.0f, 5.0f, 6.0f),
    };

    AABBf result = AABBf::Enclose(points);

    EXPECT_EQ(result.Min(), Vector3f(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(result.Max(), Vector3f(4.0f, 5.0f, 6.0f));
}

TEST(AABB, EncloseStatic_MultiplePoints) {
    std::vector<Vector3f> points = {
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 2.0f, 3.0f),
        Vector3f(-1.0f, 4.0f, 2.0f),
        Vector3f(3.0f, -2.0f, 5.0f),
        Vector3f(2.0f, 3.0f, -1.0f),
    };

    AABBf result = AABBf::Enclose(points);

    EXPECT_EQ(result.Min(), Vector3f(-1.0f, -2.0f, -1.0f));
    EXPECT_EQ(result.Max(), Vector3f(3.0f, 4.0f, 5.0f));
}

TEST(AABB, EncloseStatic_ColinearPoints) {
    std::vector<Vector3f> points = {
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
        Vector3f(2.0f, 0.0f, 0.0f),
        Vector3f(3.0f, 0.0f, 0.0f),
    };

    AABBf result = AABBf::Enclose(points);

    EXPECT_EQ(result.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(result.Max(), Vector3f(3.0f, 0.0f, 0.0f));
}

TEST(AABB, EncloseStatic_CoplanarPoints) {
    std::vector<Vector3f> points = {
        Vector3f(0.0f, 0.0f, 5.0f),
        Vector3f(1.0f, 0.0f, 5.0f),
        Vector3f(0.0f, 1.0f, 5.0f),
        Vector3f(1.0f, 1.0f, 5.0f),
    };

    AABBf result = AABBf::Enclose(points);

    EXPECT_EQ(result.Min(), Vector3f(0.0f, 0.0f, 5.0f));
    EXPECT_EQ(result.Max(), Vector3f(1.0f, 1.0f, 5.0f));
}

TEST(AABB, EncloseStatic_AllNegativePoints) {
    std::vector<Vector3f> points = {
        Vector3f(-1.0f, -2.0f, -3.0f),
        Vector3f(-4.0f, -5.0f, -6.0f),
        Vector3f(-2.0f, -3.0f, -4.0f),
    };

    AABBf result = AABBf::Enclose(points);

    EXPECT_EQ(result.Min(), Vector3f(-4.0f, -5.0f, -6.0f));
    EXPECT_EQ(result.Max(), Vector3f(-1.0f, -2.0f, -3.0f));
}

TEST(AABB, EncloseStatic_DuplicatePoints) {
    std::vector<Vector3f> points = {
        Vector3f(1.0f, 2.0f, 3.0f),
        Vector3f(1.0f, 2.0f, 3.0f),
        Vector3f(1.0f, 2.0f, 3.0f),
    };

    AABBf result = AABBf::Enclose(points);

    EXPECT_EQ(result.Min(), Vector3f(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(result.Max(), Vector3f(1.0f, 2.0f, 3.0f));
}

TEST(AABB, EncloseStatic_IntegerType) {
    std::vector<Vector3<int32_t>> points = {
        Vector3<int32_t>(0, 0, 0),
        Vector3<int32_t>(5, 3, 2),
        Vector3<int32_t>(-2, 4, 1),
        Vector3<int32_t>(3, -1, 6),
    };

    AABB<int32_t> result = AABB<int32_t>::Enclose(points);

    EXPECT_EQ(result.Min(), Vector3<int32_t>(-2, -1, 0));
    EXPECT_EQ(result.Max(), Vector3<int32_t>(5, 4, 6));
}

// ---------------------- Corners ----------------------

TEST(AABB, Corners_ValidBox) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 3.0f, 4.0f));
    auto corners = box.Corners();

    EXPECT_EQ(corners[0], Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(corners[1], Vector3f(2.0f, 0.0f, 0.0f));
    EXPECT_EQ(corners[2], Vector3f(2.0f, 3.0f, 0.0f));
    EXPECT_EQ(corners[3], Vector3f(0.0f, 3.0f, 0.0f));
    EXPECT_EQ(corners[4], Vector3f(0.0f, 0.0f, 4.0f));
    EXPECT_EQ(corners[5], Vector3f(2.0f, 0.0f, 4.0f));
    EXPECT_EQ(corners[6], Vector3f(0.0f, 3.0f, 4.0f));
    EXPECT_EQ(corners[7], Vector3f(2.0f, 3.0f, 4.0f));
}

TEST(AABB, Corners_PointBox) {
    Vector3f point(1.0f, 2.0f, 3.0f);
    AABBf box(point, point);
    auto corners = box.Corners();

    for (int i = 0; i < 8; ++i) {
        EXPECT_EQ(corners[i], point);
    }
}

TEST(AABB, Corners_NegativeCoordinates) {
    AABBf box(Vector3f(-2.0f, -3.0f, -4.0f), Vector3f(0.0f, 0.0f, 0.0f));
    auto corners = box.Corners();

    EXPECT_EQ(corners[0], Vector3f(-2.0f, -3.0f, -4.0f));
    EXPECT_EQ(corners[1], Vector3f(0.0f, -3.0f, -4.0f));
    EXPECT_EQ(corners[2], Vector3f(0.0f, 0.0f, -4.0f));
    EXPECT_EQ(corners[3], Vector3f(-2.0f, 0.0f, -4.0f));
    EXPECT_EQ(corners[4], Vector3f(-2.0f, -3.0f, 0.0f));
    EXPECT_EQ(corners[5], Vector3f(0.0f, -3.0f, 0.0f));
    EXPECT_EQ(corners[6], Vector3f(-2.0f, 0.0f, 0.0f));
    EXPECT_EQ(corners[7], Vector3f(0.0f, 0.0f, 0.0f));
}

TEST(AABB, Corners_MixedCoordinates) {
    AABBf box(Vector3f(-1.0f, -1.0f, -1.0f), Vector3f(1.0f, 1.0f, 1.0f));
    auto corners = box.Corners();

    EXPECT_EQ(corners[0], Vector3f(-1.0f, -1.0f, -1.0f));
    EXPECT_EQ(corners[1], Vector3f(1.0f, -1.0f, -1.0f));
    EXPECT_EQ(corners[2], Vector3f(1.0f, 1.0f, -1.0f));
    EXPECT_EQ(corners[3], Vector3f(-1.0f, 1.0f, -1.0f));
    EXPECT_EQ(corners[4], Vector3f(-1.0f, -1.0f, 1.0f));
    EXPECT_EQ(corners[5], Vector3f(1.0f, -1.0f, 1.0f));
    EXPECT_EQ(corners[6], Vector3f(-1.0f, 1.0f, 1.0f));
    EXPECT_EQ(corners[7], Vector3f(1.0f, 1.0f, 1.0f));
}

TEST(AABB, Corners_FlatBoxXY) {
    AABBf box(Vector3f(0.0f, 0.0f, 5.0f), Vector3f(2.0f, 3.0f, 5.0f));
    auto corners = box.Corners();

    EXPECT_EQ(corners[0], Vector3f(0.0f, 0.0f, 5.0f));
    EXPECT_EQ(corners[1], Vector3f(2.0f, 0.0f, 5.0f));
    EXPECT_EQ(corners[2], Vector3f(2.0f, 3.0f, 5.0f));
    EXPECT_EQ(corners[3], Vector3f(0.0f, 3.0f, 5.0f));
    EXPECT_EQ(corners[4], Vector3f(0.0f, 0.0f, 5.0f));
    EXPECT_EQ(corners[5], Vector3f(2.0f, 0.0f, 5.0f));
    EXPECT_EQ(corners[6], Vector3f(0.0f, 3.0f, 5.0f));
    EXPECT_EQ(corners[7], Vector3f(2.0f, 3.0f, 5.0f));
}

TEST(AABB, Corners_IntegerType) {
    AABB<int32_t> box(Vector3<int32_t>(0, 0, 0), Vector3<int32_t>(2, 3, 4));
    auto corners = box.Corners();

    EXPECT_EQ(corners[0], Vector3<int32_t>(0, 0, 0));
    EXPECT_EQ(corners[1], Vector3<int32_t>(2, 0, 0));
    EXPECT_EQ(corners[2], Vector3<int32_t>(2, 3, 0));
    EXPECT_EQ(corners[3], Vector3<int32_t>(0, 3, 0));
    EXPECT_EQ(corners[4], Vector3<int32_t>(0, 0, 4));
    EXPECT_EQ(corners[5], Vector3<int32_t>(2, 0, 4));
    EXPECT_EQ(corners[6], Vector3<int32_t>(0, 3, 4));
    EXPECT_EQ(corners[7], Vector3<int32_t>(2, 3, 4));
}

TEST(AABB, Corners_VerifyAllCornersUnique) {
    AABBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    auto corners = box.Corners();

    for (int i = 0; i < 8; ++i) {
        for (int j = i + 1; j < 8; ++j) {
            EXPECT_NE(corners[i], corners[j]) << "Corners " << i << " and " << j << " should be different";
        }
    }
}

// ---------------------- Expand ----------------------

TEST(AABB, Expand_UniformOffset) {
    AABBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    Vector3f offset(1.0f, 1.0f, 1.0f);

    box.Expand(offset);

    EXPECT_EQ(box.Min(), Vector3f(0.0f, 1.0f, 2.0f));
    EXPECT_EQ(box.Max(), Vector3f(5.0f, 6.0f, 7.0f));
}

TEST(AABB, Expand_NonUniformOffset) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 3.0f, 4.0f));
    Vector3f offset(1.0f, 2.0f, 3.0f);

    box.Expand(offset);

    EXPECT_EQ(box.Min(), Vector3f(-1.0f, -2.0f, -3.0f));
    EXPECT_EQ(box.Max(), Vector3f(3.0f, 5.0f, 7.0f));
}

TEST(AABB, Expand_IntegerType) {
    AABB<int32_t> box(Vector3<int32_t>(0, 0, 0), Vector3<int32_t>(5, 5, 5));
    Vector3<int32_t> offset(2, 3, 1);

    box.Expand(offset);

    EXPECT_EQ(box.Min(), Vector3<int32_t>(-2, -3, -1));
    EXPECT_EQ(box.Max(), Vector3<int32_t>(7, 8, 6));
}

// ---------------------- Scale ----------------------

TEST(AABB, Scale_UniformFromCenter) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));

    box.Scale(2.0f);

    EXPECT_EQ(box.Min(), Vector3f(-1.0f, -1.0f, -1.0f));
    EXPECT_EQ(box.Max(), Vector3f(3.0f, 3.0f, 3.0f));
}

TEST(AABB, Scale_ShrinkFromCenter) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(4.0f, 4.0f, 4.0f));

    box.Scale(0.5f);

    EXPECT_NEAR(box.Min().x, 1.0f, ABS_TOLERANCE);
    EXPECT_NEAR(box.Min().y, 1.0f, ABS_TOLERANCE);
    EXPECT_NEAR(box.Min().z, 1.0f, ABS_TOLERANCE);
    EXPECT_NEAR(box.Max().x, 3.0f, ABS_TOLERANCE);
    EXPECT_NEAR(box.Max().y, 3.0f, ABS_TOLERANCE);
    EXPECT_NEAR(box.Max().z, 3.0f, ABS_TOLERANCE);
}

TEST(AABB, Scale_FromCustomCenter) {
    AABBf box(Vector3f(1.0f, 1.0f, 1.0f), Vector3f(3.0f, 3.0f, 3.0f));
    Vector3f center(0.0f, 0.0f, 0.0f);

    box.Scale(2.0f, center);

    EXPECT_EQ(box.Min(), Vector3f(2.0f, 2.0f, 2.0f));
    EXPECT_EQ(box.Max(), Vector3f(6.0f, 6.0f, 6.0f));
}

TEST(AABB, Scale_IntegerType) {
    AABB<int32_t> box(Vector3<int32_t>(0, 0, 0), Vector3<int32_t>(4, 4, 4));

    box.Scale(2);

    EXPECT_EQ(box.Min(), Vector3<int32_t>(-2, -2, -2));
    EXPECT_EQ(box.Max(), Vector3<int32_t>(6, 6, 6));
}

// ---------------------- ClosestPoint ----------------------

TEST(AABB, ClosestPoint_InsideBox) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    Vector3f point(5.0f, 5.0f, 5.0f);

    Vector3f closest = box.ClosestPoint(point);

    EXPECT_EQ(closest, point);
}

TEST(AABB, ClosestPoint_OutsideOnAllAxes) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    Vector3f point(15.0f, 15.0f, 15.0f);

    Vector3f closest = box.ClosestPoint(point);

    EXPECT_EQ(closest, Vector3f(10.0f, 10.0f, 10.0f));
}

TEST(AABB, ClosestPoint_OutsideNegative) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    Vector3f point(-5.0f, -5.0f, -5.0f);

    Vector3f closest = box.ClosestPoint(point);

    EXPECT_EQ(closest, Vector3f(0.0f, 0.0f, 0.0f));
}

TEST(AABB, ClosestPoint_OutsideOnOneAxis) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    Vector3f point(5.0f, 5.0f, 15.0f);

    Vector3f closest = box.ClosestPoint(point);

    EXPECT_EQ(closest, Vector3f(5.0f, 5.0f, 10.0f));
}

TEST(AABB, ClosestPoint_TypeConversion) {
    AABB<int32_t> box(Vector3<int32_t>(0, 0, 0), Vector3<int32_t>(10, 10, 10));
    Vector3f point(15.7f, 12.3f, -3.8f);

    Vector3f closest = box.ClosestPoint(point);

    EXPECT_NEAR(closest.x, 10.0f, ABS_TOLERANCE);
    EXPECT_NEAR(closest.y, 10.0f, ABS_TOLERANCE);
    EXPECT_NEAR(closest.z, 0.0f, ABS_TOLERANCE);
}

// ---------------------- DistanceSquared ----------------------

TEST(AABB, DistanceSquared_PointInside) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    Vector3f point(5.0f, 5.0f, 5.0f);

    EXPECT_FLOAT_EQ(box.DistanceSquared(point), 0.0f);
}

TEST(AABB, DistanceSquared_PointOutside) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    Vector3f point(12.0f, 13.0f, 14.0f);

    EXPECT_NEAR(box.DistanceSquared(point), 29.0f, ABS_TOLERANCE);
}

// ---------------------- Distance ----------------------

TEST(AABB, Distance_PointInside) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    Vector3f point(5.0f, 5.0f, 5.0f);

    EXPECT_FLOAT_EQ(box.Distance(point), 0.0f);
}

TEST(AABB, Distance_PointOutside) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    Vector3f point(13.0f, 14.0f, 15.0f);

    float expected = std::sqrt(3.0f * 3.0f + 4.0f * 4.0f + 5.0f * 5.0f);
    EXPECT_NEAR(box.Distance(point), expected, ABS_TOLERANCE);
}

// ---------------------- Operators ----------------------

TEST(AABB, OperatorPlus) {
    AABBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    Vector3f offset(10.0f, 20.0f, 30.0f);

    AABBf result = box + offset;

    EXPECT_EQ(result.Min(), Vector3f(11.0f, 22.0f, 33.0f));
    EXPECT_EQ(result.Max(), Vector3f(14.0f, 25.0f, 36.0f));
}

TEST(AABB, OperatorMinus) {
    AABBf box(Vector3f(10.0f, 20.0f, 30.0f), Vector3f(15.0f, 25.0f, 35.0f));
    Vector3f offset(5.0f, 10.0f, 15.0f);

    AABBf result = box - offset;

    EXPECT_EQ(result.Min(), Vector3f(5.0f, 10.0f, 15.0f));
    EXPECT_EQ(result.Max(), Vector3f(10.0f, 15.0f, 20.0f));
}

TEST(AABB, OperatorPlusEquals) {
    AABBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    Vector3f offset(10.0f, 20.0f, 30.0f);

    AABBf& result = (box += offset);

    EXPECT_EQ(box.Min(), Vector3f(11.0f, 22.0f, 33.0f));
    EXPECT_EQ(box.Max(), Vector3f(14.0f, 25.0f, 36.0f));
    EXPECT_EQ(&result, &box);
}

TEST(AABB, OperatorMinusEquals) {
    AABBf box(Vector3f(10.0f, 20.0f, 30.0f), Vector3f(15.0f, 25.0f, 35.0f));
    Vector3f offset(5.0f, 10.0f, 15.0f);

    AABBf& result = (box -= offset);

    EXPECT_EQ(box.Min(), Vector3f(5.0f, 10.0f, 15.0f));
    EXPECT_EQ(box.Max(), Vector3f(10.0f, 15.0f, 20.0f));
    EXPECT_EQ(&result, &box);
}

TEST(AABB, OperatorPipe) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    AABBf box2(Vector3f(10.0f, 10.0f, 10.0f), Vector3f(15.0f, 15.0f, 15.0f));

    AABBf result = box1 | box2;

    EXPECT_EQ(result.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(result.Max(), Vector3f(15.0f, 15.0f, 15.0f));
}

TEST(AABB, OperatorPipeEquals) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    AABBf box2(Vector3f(10.0f, 10.0f, 10.0f), Vector3f(15.0f, 15.0f, 15.0f));

    AABBf& result = (box1 |= box2);

    EXPECT_EQ(box1.Min(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box1.Max(), Vector3f(15.0f, 15.0f, 15.0f));
    EXPECT_EQ(&result, &box1);
}

TEST(AABB, OperatorEquals) {
    AABBf box1(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    AABBf box2(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    AABBf box3(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));

    EXPECT_TRUE(box1 == box2);
    EXPECT_FALSE(box1 == box3);
    EXPECT_TRUE(box1 == box1);
}

TEST(AABB, OperatorNotEquals) {
    AABBf box1(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    AABBf box2(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));

    EXPECT_TRUE(box1 != box2);
    EXPECT_FALSE(box1 != box1);
}

// ---------------------- Lerp ----------------------

TEST(AABB, Lerp_BasicInterpolation) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));

    EXPECT_EQ(box.Lerp(Vector3f(0.0f, 0.0f, 0.0f)), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box.Lerp(Vector3f(1.0f, 1.0f, 1.0f)), Vector3f(10.0f, 10.0f, 10.0f));
    EXPECT_EQ(box.Lerp(Vector3f(0.5f, 0.5f, 0.5f)), Vector3f(5.0f, 5.0f, 5.0f));
}

TEST(AABB, Lerp_NonUniformT) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 20.0f, 30.0f));
    Vector3f result = box.Lerp(Vector3f(0.25f, 0.5f, 0.75f));

    EXPECT_NEAR(result.x, 2.5f, ABS_TOLERANCE);
    EXPECT_NEAR(result.y, 10.0f, ABS_TOLERANCE);
    EXPECT_NEAR(result.z, 22.5f, ABS_TOLERANCE);
}

// ---------------------- Offset ----------------------

TEST(AABB, Offset_BasicCalculation) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));

    EXPECT_EQ(box.Offset(Vector3f(0.0f, 0.0f, 0.0f)), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box.Offset(Vector3f(10.0f, 10.0f, 10.0f)), Vector3f(1.0f, 1.0f, 1.0f));
    EXPECT_EQ(box.Offset(Vector3f(5.0f, 5.0f, 5.0f)), Vector3f(0.5f, 0.5f, 0.5f));
}

TEST(AABB, Offset_NonUniformBox) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 20.0f, 30.0f));
    Vector3f result = box.Offset(Vector3f(5.0f, 10.0f, 15.0f));

    EXPECT_NEAR(result.x, 0.5f, ABS_TOLERANCE);
    EXPECT_NEAR(result.y, 0.5f, ABS_TOLERANCE);
    EXPECT_NEAR(result.z, 0.5f, ABS_TOLERANCE);
}

TEST(AABB, LerpAndOffset_Inverse) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 20.0f, 30.0f));
    Vector3f t(0.25f, 0.5f, 0.75f);

    Vector3f lerpResult = box.Lerp(t);
    Vector3f offsetResult = box.Offset(lerpResult);

    EXPECT_NEAR(offsetResult.x, t.x, ABS_TOLERANCE);
    EXPECT_NEAR(offsetResult.y, t.y, ABS_TOLERANCE);
    EXPECT_NEAR(offsetResult.z, t.z, ABS_TOLERANCE);
}

// ---------------------- Cast Operator ----------------------

TEST(AABB, CastOperator_FloatToInt_Truncation) {
    AABBf boxf(Vector3f(1.7f, 2.3f, 3.9f), Vector3f(4.2f, 5.8f, 6.1f));
    AABB<int32_t> boxi = AABB<int32_t>(boxf);

    EXPECT_EQ(boxi.Min(), Vector3<int32_t>(1, 2, 3));
    EXPECT_EQ(boxi.Max(), Vector3<int32_t>(4, 5, 6));
}

TEST(AABB, CastOperator_IntToFloat) {
    AABB<int32_t> boxi(Vector3<int32_t>(1, 2, 3), Vector3<int32_t>(4, 5, 6));
    AABBf boxf = AABBf(boxi);

    EXPECT_EQ(boxf.Min(), Vector3f(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(boxf.Max(), Vector3f(4.0f, 5.0f, 6.0f));
}

TEST(AABB, CastOperator_DoubleToFloat) {
    AABBd boxd(Vector3<double>(1.5, 2.5, 3.5), Vector3<double>(4.5, 5.5, 6.5));
    AABBf boxf = AABBf(boxd);

    EXPECT_NEAR(boxf.Min().x, 1.5f, ABS_TOLERANCE);
    EXPECT_NEAR(boxf.Max().z, 6.5f, ABS_TOLERANCE);
}

// ---------------------- Contains(AABB) ----------------------

TEST(AABB, Contains_FullyContained) {
    AABBf outer(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    AABBf inner(Vector3f(2.0f, 2.0f, 2.0f), Vector3f(8.0f, 8.0f, 8.0f));

    EXPECT_TRUE(outer.Contains(inner));
    EXPECT_FALSE(inner.Contains(outer));
}

TEST(AABB, Contains_PartialOverlap) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    AABBf box2(Vector3f(3.0f, 3.0f, 3.0f), Vector3f(8.0f, 8.0f, 8.0f));

    EXPECT_FALSE(box1.Contains(box2));
    EXPECT_FALSE(box2.Contains(box1));
}

TEST(AABB, Contains_Separated) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    AABBf box2(Vector3f(10.0f, 10.0f, 10.0f), Vector3f(15.0f, 15.0f, 15.0f));

    EXPECT_FALSE(box1.Contains(box2));
    EXPECT_FALSE(box2.Contains(box1));
}

TEST(AABB, Contains_Itself) {
    AABBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));

    EXPECT_TRUE(box.Contains(box));
}

TEST(AABB, Contains_IdenticalBoxes) {
    AABBf box1(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    AABBf box2(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));

    EXPECT_TRUE(box1.Contains(box2));
    EXPECT_TRUE(box2.Contains(box1));
}

TEST(AABB, Contains_PointBoxInside) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    AABBf point(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(5.0f, 5.0f, 5.0f));

    EXPECT_TRUE(box.Contains(point));
    EXPECT_FALSE(point.Contains(box));
}

TEST(AABB, Contains_EdgeCase_SameMin) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    AABBf box2(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));

    EXPECT_TRUE(box1.Contains(box2));
    EXPECT_FALSE(box2.Contains(box1));
}

TEST(AABB, Contains_NegativeCoordinates) {
    AABBf outer(Vector3f(-10.0f, -10.0f, -10.0f), Vector3f(10.0f, 10.0f, 10.0f));
    AABBf inner(Vector3f(-5.0f, -5.0f, -5.0f), Vector3f(5.0f, 5.0f, 5.0f));

    EXPECT_TRUE(outer.Contains(inner));
    EXPECT_FALSE(inner.Contains(outer));
}

// ---------------------- Intersects(AABB) ----------------------

TEST(AABB, Intersects_Overlapping) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    AABBf box2(Vector3f(3.0f, 3.0f, 3.0f), Vector3f(8.0f, 8.0f, 8.0f));

    EXPECT_TRUE(box1.Intersects(box2));
    EXPECT_TRUE(box2.Intersects(box1));
}

TEST(AABB, Intersects_Separated) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    AABBf box2(Vector3f(10.0f, 10.0f, 10.0f), Vector3f(15.0f, 15.0f, 15.0f));

    EXPECT_FALSE(box1.Intersects(box2));
    EXPECT_FALSE(box2.Intersects(box1));
}

TEST(AABB, Intersects_FullyContained) {
    AABBf outer(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    AABBf inner(Vector3f(2.0f, 2.0f, 2.0f), Vector3f(8.0f, 8.0f, 8.0f));

    EXPECT_TRUE(outer.Intersects(inner));
    EXPECT_TRUE(inner.Intersects(outer));
}

TEST(AABB, Intersects_Identical) {
    AABBf box1(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    AABBf box2(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));

    EXPECT_TRUE(box1.Intersects(box2));
}

TEST(AABB, Intersects_EdgeTouching) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    AABBf box2(Vector3f(5.0f, 0.0f, 0.0f), Vector3f(10.0f, 5.0f, 5.0f));

    // Edge touching - boxes share a face but no volume overlap
    EXPECT_TRUE(box1.Intersects(box2));
}

TEST(AABB, Intersects_CornerTouching) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    AABBf box2(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(10.0f, 10.0f, 10.0f));

    // Corner touching - boxes share a single point
    EXPECT_TRUE(box1.Intersects(box2));
}

TEST(AABB, Intersects_SeparatedOnXAxis) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 10.0f, 10.0f));
    AABBf box2(Vector3f(6.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));

    EXPECT_FALSE(box1.Intersects(box2));
}

TEST(AABB, Intersects_NegativeCoordinates) {
    AABBf box1(Vector3f(-10.0f, -10.0f, -10.0f), Vector3f(-5.0f, -5.0f, -5.0f));
    AABBf box2(Vector3f(-7.0f, -7.0f, -7.0f), Vector3f(-2.0f, -2.0f, -2.0f));

    EXPECT_TRUE(box1.Intersects(box2));
}

// ---------------------- Intersects(AABB, AABB&) ----------------------

TEST(AABB, IntersectsWithOutput_Overlapping) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    AABBf box2(Vector3f(3.0f, 3.0f, 3.0f), Vector3f(8.0f, 8.0f, 8.0f));
    AABBf intersection;

    EXPECT_TRUE(box1.Intersects(box2, intersection));
    EXPECT_EQ(intersection.Min(), Vector3f(3.0f, 3.0f, 3.0f));
    EXPECT_EQ(intersection.Max(), Vector3f(5.0f, 5.0f, 5.0f));
}

TEST(AABB, IntersectsWithOutput_NoOverlap) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    AABBf box2(Vector3f(10.0f, 10.0f, 10.0f), Vector3f(15.0f, 15.0f, 15.0f));
    AABBf intersection;

    EXPECT_FALSE(box1.Intersects(box2, intersection));
}

TEST(AABB, IntersectsWithOutput_FullyContained) {
    AABBf outer(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    AABBf inner(Vector3f(2.0f, 2.0f, 2.0f), Vector3f(8.0f, 8.0f, 8.0f));
    AABBf intersection;

    EXPECT_TRUE(outer.Intersects(inner, intersection));
    EXPECT_EQ(intersection.Min(), inner.Min());
    EXPECT_EQ(intersection.Max(), inner.Max());
}

TEST(AABB, IntersectsWithOutput_PartialOverlap) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 10.0f, 10.0f));
    AABBf box2(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(15.0f, 15.0f, 15.0f));
    AABBf intersection;

    EXPECT_TRUE(box1.Intersects(box2, intersection));
    EXPECT_EQ(intersection.Min(), Vector3f(5.0f, 5.0f, 5.0f));
    EXPECT_EQ(intersection.Max(), Vector3f(10.0f, 10.0f, 10.0f));
}

TEST(AABB, IntersectsWithOutput_EdgeTouching) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    AABBf box2(Vector3f(5.0f, 0.0f, 0.0f), Vector3f(10.0f, 5.0f, 5.0f));
    AABBf intersection;

    bool result = box1.Intersects(box2, intersection);
    if (result) {
        // If edge touching counts as intersection, verify the intersection is a flat box
        EXPECT_EQ(intersection.Min().x, 5.0f);
        EXPECT_EQ(intersection.Max().x, 5.0f);
    }
}

// ---------------------- Translate ----------------------

TEST(AABB, Translate_PositiveOffset) {
    AABBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    box.Translate(Vector3f(10.0f, 20.0f, 30.0f));

    EXPECT_EQ(box.Min(), Vector3f(11.0f, 22.0f, 33.0f));
    EXPECT_EQ(box.Max(), Vector3f(14.0f, 25.0f, 36.0f));
}

TEST(AABB, Translate_NegativeOffset) {
    AABBf box(Vector3f(10.0f, 20.0f, 30.0f), Vector3f(15.0f, 25.0f, 35.0f));
    box.Translate(Vector3f(-5.0f, -10.0f, -15.0f));

    EXPECT_EQ(box.Min(), Vector3f(5.0f, 10.0f, 15.0f));
    EXPECT_EQ(box.Max(), Vector3f(10.0f, 15.0f, 20.0f));
}

TEST(AABB, Translate_ZeroOffset) {
    AABBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    Vector3f originalMin = box.Min();
    Vector3f originalMax = box.Max();

    box.Translate(Vector3f(0.0f, 0.0f, 0.0f));

    EXPECT_EQ(box.Min(), originalMin);
    EXPECT_EQ(box.Max(), originalMax);
}

TEST(AABB, Translate_PointBox) {
    Vector3f point(5.0f, 5.0f, 5.0f);
    AABBf box(point, point);
    box.Translate(Vector3f(2.0f, 3.0f, 4.0f));

    EXPECT_EQ(box.Min(), Vector3f(7.0f, 8.0f, 9.0f));
    EXPECT_EQ(box.Max(), Vector3f(7.0f, 8.0f, 9.0f));
}

TEST(AABB, Translate_IntegerType) {
    AABB<int32_t> box(Vector3<int32_t>(1, 2, 3), Vector3<int32_t>(4, 5, 6));
    box.Translate(Vector3<int32_t>(10, 20, 30));

    EXPECT_EQ(box.Min(), Vector3<int32_t>(11, 22, 33));
    EXPECT_EQ(box.Max(), Vector3<int32_t>(14, 25, 36));
}

// ---------------------- SetCenterExtents ----------------------

TEST(AABB, SetCenterExtents_Basic) {
    AABBf box;
    box.SetCenterExtents(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(2.0f, 3.0f, 4.0f));

    EXPECT_EQ(box.Min(), Vector3f(3.0f, 2.0f, 1.0f));
    EXPECT_EQ(box.Max(), Vector3f(7.0f, 8.0f, 9.0f));
}

TEST(AABB, SetCenterExtents_NegativeExtents) {
    AABBf box;
    box.SetCenterExtents(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(-2.0f, -3.0f, -4.0f));

    // Should use absolute value of extents
    EXPECT_EQ(box.Min(), Vector3f(-2.0f, -3.0f, -4.0f));
    EXPECT_EQ(box.Max(), Vector3f(2.0f, 3.0f, 4.0f));
}

TEST(AABB, SetCenterExtents_ZeroExtents) {
    AABBf box;
    box.SetCenterExtents(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(0.0f, 0.0f, 0.0f));

    EXPECT_EQ(box.Min(), Vector3f(5.0f, 5.0f, 5.0f));
    EXPECT_EQ(box.Max(), Vector3f(5.0f, 5.0f, 5.0f));
}

TEST(AABB, SetCenterExtents_NegativeCenter) {
    AABBf box;
    box.SetCenterExtents(Vector3f(-10.0f, -10.0f, -10.0f), Vector3f(5.0f, 5.0f, 5.0f));

    EXPECT_EQ(box.Min(), Vector3f(-15.0f, -15.0f, -15.0f));
    EXPECT_EQ(box.Max(), Vector3f(-5.0f, -5.0f, -5.0f));
}

// ---------------------- FromCenterExtents ----------------------

TEST(AABB, FromCenterExtents_Basic) {
    AABBf box = AABBf::FromCenterExtents(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(2.0f, 3.0f, 4.0f));

    EXPECT_EQ(box.Min(), Vector3f(3.0f, 2.0f, 1.0f));
    EXPECT_EQ(box.Max(), Vector3f(7.0f, 8.0f, 9.0f));
}

TEST(AABB, FromCenterExtents_NegativeExtents) {
    AABBf box = AABBf::FromCenterExtents(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(-2.0f, -3.0f, -4.0f));

    EXPECT_EQ(box.Min(), Vector3f(-2.0f, -3.0f, -4.0f));
    EXPECT_EQ(box.Max(), Vector3f(2.0f, 3.0f, 4.0f));
}

TEST(AABB, FromCenterExtents_ConsistencyWithSetter) {
    Vector3f center(10.0f, 20.0f, 30.0f);
    Vector3f extents(5.0f, 10.0f, 15.0f);

    AABBf box1 = AABBf::FromCenterExtents(center, extents);
    AABBf box2;
    box2.SetCenterExtents(center, extents);

    EXPECT_EQ(box1.Min(), box2.Min());
    EXPECT_EQ(box1.Max(), box2.Max());
}

// ---------------------- MaximumExtent ----------------------

TEST(AABB, MaximumExtent_XIsLargest) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 5.0f, 3.0f));

    EXPECT_EQ(box.MaximumExtent(), 0);  // X axis
}

TEST(AABB, MaximumExtent_YIsLargest) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(3.0f, 10.0f, 5.0f));

    EXPECT_EQ(box.MaximumExtent(), 1);  // Y axis
}

TEST(AABB, MaximumExtent_ZIsLargest) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(3.0f, 5.0f, 10.0f));

    EXPECT_EQ(box.MaximumExtent(), 2);  // Z axis
}

TEST(AABB, MaximumExtent_AllEqual) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    int result = box.MaximumExtent();

    // Any axis is valid when all are equal
    EXPECT_TRUE(result >= 0 && result <= 2);
}

TEST(AABB, MaximumExtent_NegativeCoordinates) {
    AABBf box(Vector3f(-10.0f, -5.0f, -3.0f), Vector3f(0.0f, 0.0f, 0.0f));

    // Should use size (10, 5, 3), not coordinates
    EXPECT_EQ(box.MaximumExtent(), 0);  // X has size 10
}

TEST(AABB, MaximumExtent_PointBox) {
    Vector3f point(5.0f, 5.0f, 5.0f);
    AABBf box(point, point);
    int result = box.MaximumExtent();

    // All extents are zero, any is valid
    EXPECT_TRUE(result >= 0 && result <= 2);
}

// ---------------------- Centroid ----------------------

TEST(AABB, Centroid_MatchesCenter) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 20.0f, 30.0f));

    EXPECT_EQ(box.Centroid(), box.Center());
}

TEST(AABB, Centroid_VariousBoxes) {
    AABBf box1(Vector3f(-5.0f, -5.0f, -5.0f), Vector3f(5.0f, 5.0f, 5.0f));
    EXPECT_EQ(box1.Centroid(), Vector3f(0.0f, 0.0f, 0.0f));

    AABBf box2(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(4.0f, 5.0f, 6.0f));
    EXPECT_EQ(box2.Centroid(), Vector3f(2.5f, 3.5f, 4.5f));
}
