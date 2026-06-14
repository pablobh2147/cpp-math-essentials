#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "BoundingBox.hpp"

#ifdef MATH_ESSENTIALS_NAMESPACE
#define NAMESPACE_NAME MATH_ESSENTIALS_NAMESPACE
#else
#define NAMESPACE_NAME
#endif

#ifdef MATH_ESSENTIALS_NAMESPACE
using namespace NAMESPACE_NAME;
#endif

constexpr float ABS_ERROR = std::numeric_limits<float>::epsilon() * 64.0f;

// ---------------------- Constructors ----------------------

TEST(OBB, Constructor_Default) {
    OBBf box;
    EXPECT_EQ(box.Center(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box.Extents(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box.Axes(), Matrix3f(1.0f));
}

TEST(OBB, Constructor_CenterExtents) {
    Vector3f center(1.0f, 2.0f, 3.0f);
    Vector3f extents(4.0f, 5.0f, 6.0f);
    OBBf box(center, extents);

    EXPECT_EQ(box.Center(), center);
    EXPECT_EQ(box.Extents(), extents);
    EXPECT_EQ(box.Axes(), Matrix3f(1.0f));
}

TEST(OBB, Constructor_NegativeExtents) {
    Vector3f center(0.0f, 0.0f, 0.0f);
    Vector3f extents(-2.0f, -3.0f, -4.0f);
    OBBf box(center, extents);

    EXPECT_EQ(box.Extents(), Vector3f(2.0f, 3.0f, 4.0f));
}

TEST(OBB, Constructor_CenterExtentsAxes) {
    Vector3f center(1.0f, 2.0f, 3.0f);
    Vector3f extents(4.0f, 5.0f, 6.0f);
    Matrix3f axes(1.0f);
    OBBf box(center, extents, axes);

    EXPECT_EQ(box.Center(), center);
    EXPECT_EQ(box.Extents(), extents);
    EXPECT_EQ(box.Axes(), axes);
}

TEST(OBB, Constructor_CenterExtentsEuler) {
    Vector3f center(1.0f, 2.0f, 3.0f);
    Vector3f extents(4.0f, 5.0f, 6.0f);
    Vector3f euler(0.0f, 0.0f, 0.0f);
    OBBf box(center, extents, euler);

    EXPECT_EQ(box.Center(), center);
    EXPECT_EQ(box.Extents(), extents);
    // With zero euler angles, axes should be identity
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(box.Axes()[i][j], Matrix3f(1.0f)[i][j], ABS_ERROR);
}

TEST(OBB, Constructor_DoubleType) {
    Vector3<double> center(1.0, 2.0, 3.0);
    Vector3<double> extents(4.0, 5.0, 6.0);
    OBBd box(center, extents);

    EXPECT_EQ(box.Center(), center);
    EXPECT_EQ(box.Extents(), extents);
}

// ---------------------- Getters ----------------------

TEST(OBB, Center_ReturnsCorrectValue) {
    OBBf box(Vector3f(5.0f, 10.0f, 15.0f), Vector3f(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(box.Center(), Vector3f(5.0f, 10.0f, 15.0f));
}

TEST(OBB, Centroid_MatchesCenter) {
    OBBf box(Vector3f(5.0f, 10.0f, 15.0f), Vector3f(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(box.Centroid(), box.Center());
}

TEST(OBB, Size_ReturnsDoubleExtents) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 3.0f, 4.0f));
    EXPECT_EQ(box.Size(), Vector3f(4.0f, 6.0f, 8.0f));
}

TEST(OBB, Extents_ReturnsCorrectValue) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 3.0f, 4.0f));
    EXPECT_EQ(box.Extents(), Vector3f(2.0f, 3.0f, 4.0f));
}

TEST(OBB, Axes_DefaultIsIdentity) {
    OBBf box;
    EXPECT_EQ(box.Axes(), Matrix3f(1.0f));
}

TEST(OBB, Volume_ValidBox) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 2.0f, 3.0f));
    // Size = (2, 4, 6), Volume = 2*4*6 = 48
    EXPECT_NEAR(box.Volume(), 48.0f, ABS_ERROR);
}

TEST(OBB, Volume_PointBox) {
    OBBf box(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_FLOAT_EQ(box.Volume(), 0.0f);
}

TEST(OBB, SurfaceArea_ValidBox) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 2.0f, 3.0f));
    // Size = (2, 4, 6), SA = 2*(2*4 + 4*6 + 6*2) = 2*(8+24+12) = 88
    EXPECT_NEAR(box.SurfaceArea(), 88.0f, ABS_ERROR);
}

TEST(OBB, SurfaceArea_PointBox) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_FLOAT_EQ(box.SurfaceArea(), 0.0f);
}

// ---------------------- Setters ----------------------

TEST(OBB, SetCenter_UpdatesCenter) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f));
    box.SetCenter(Vector3f(5.0f, 10.0f, 15.0f));

    EXPECT_EQ(box.Center(), Vector3f(5.0f, 10.0f, 15.0f));
    EXPECT_EQ(box.Extents(), Vector3f(1.0f, 1.0f, 1.0f));
}

TEST(OBB, SetExtents_UpdatesExtents) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f));
    box.SetExtents(Vector3f(5.0f, 10.0f, 15.0f));

    EXPECT_EQ(box.Center(), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(box.Extents(), Vector3f(5.0f, 10.0f, 15.0f));
}

TEST(OBB, SetExtents_NegativeExtents) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f));
    box.SetExtents(Vector3f(-5.0f, -10.0f, -15.0f));

    EXPECT_EQ(box.Extents(), Vector3f(5.0f, 10.0f, 15.0f));
}

TEST(OBB, SetAxes_Matrix) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f));
    Matrix3f axes(1.0f);
    box.SetAxes(axes);

    EXPECT_EQ(box.Axes(), axes);
}

TEST(OBB, SetAxes_EulerAngles_Zero) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f));
    box.SetAxes(Vector3f(0.0f, 0.0f, 0.0f));

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(box.Axes()[i][j], Matrix3f(1.0f)[i][j], ABS_ERROR);
}

TEST(OBB, SetAxes_EulerAngles_90DegYaw) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f));
    float yaw = glm::radians(90.0f);
    box.SetAxes(Vector3f(0.0f, yaw, 0.0f));

    // After 90 degree yaw rotation, X axis should point along Z
    Vector3f localX = Vector3f(box.Axes()[0]);
    EXPECT_NEAR(localX.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(localX.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(std::abs(localX.z), 1.0f, ABS_ERROR);
}

// ---------------------- Geometric Transformation ----------------------

TEST(OBB, Translate_PositiveOffset) {
    OBBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(1.0f, 1.0f, 1.0f));
    box.Translate(Vector3f(10.0f, 20.0f, 30.0f));

    EXPECT_EQ(box.Center(), Vector3f(11.0f, 22.0f, 33.0f));
    EXPECT_EQ(box.Extents(), Vector3f(1.0f, 1.0f, 1.0f));
}

TEST(OBB, Translate_NegativeOffset) {
    OBBf box(Vector3f(10.0f, 20.0f, 30.0f), Vector3f(1.0f, 1.0f, 1.0f));
    box.Translate(Vector3f(-5.0f, -10.0f, -15.0f));

    EXPECT_EQ(box.Center(), Vector3f(5.0f, 10.0f, 15.0f));
}

TEST(OBB, Translate_ZeroOffset) {
    OBBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(1.0f, 1.0f, 1.0f));
    box.Translate(Vector3f(0.0f, 0.0f, 0.0f));

    EXPECT_EQ(box.Center(), Vector3f(1.0f, 2.0f, 3.0f));
}

TEST(OBB, Rotate_ZeroAngles) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 2.0f, 3.0f));
    Matrix3f originalAxes = box.Axes();
    box.Rotate(Vector3f(0.0f, 0.0f, 0.0f));

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(box.Axes()[i][j], originalAxes[i][j], ABS_ERROR);
}

TEST(OBB, Rotate_90DegYaw) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 2.0f, 3.0f));
    float yaw = glm::radians(90.0f);
    box.Rotate(0.0f, yaw, 0.0f);

    // Extents should remain unchanged
    EXPECT_EQ(box.Extents(), Vector3f(1.0f, 2.0f, 3.0f));
    // Center should remain unchanged
    EXPECT_EQ(box.Center(), Vector3f(0.0f, 0.0f, 0.0f));
}

TEST(OBB, Rotate_EulerAnglesOverload) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 2.0f, 3.0f));
    float yaw = glm::radians(45.0f);
    box.Rotate(Vector3f(0.0f, yaw, 0.0f));

    EXPECT_EQ(box.Extents(), Vector3f(1.0f, 2.0f, 3.0f));
    EXPECT_EQ(box.Center(), Vector3f(0.0f, 0.0f, 0.0f));
}

TEST(OBB, Expand_PositiveOffset) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 2.0f, 3.0f));
    box.Expand(Vector3f(1.0f, 1.0f, 1.0f));

    EXPECT_EQ(box.Extents(), Vector3f(2.0f, 3.0f, 4.0f));
    EXPECT_EQ(box.Center(), Vector3f(0.0f, 0.0f, 0.0f));
}

TEST(OBB, Expand_NegativeOffset_Clamped) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 3.0f, 4.0f));
    box.Expand(Vector3f(-5.0f, -5.0f, -5.0f));

    // Since SetExtents uses abs, extents should be abs(2-5, 3-5, 4-5) = (3, 2, 1)
    EXPECT_EQ(box.Extents(), Vector3f(3.0f, 2.0f, 1.0f));
}

TEST(OBB, Scale_DoubleSize) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 2.0f, 3.0f));
    box.Scale(2.0f);

    EXPECT_EQ(box.Extents(), Vector3f(2.0f, 4.0f, 6.0f));
    EXPECT_EQ(box.Center(), Vector3f(0.0f, 0.0f, 0.0f));
}

TEST(OBB, Scale_HalfSize) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(4.0f, 6.0f, 8.0f));
    box.Scale(0.5f);

    EXPECT_NEAR(box.Extents().x, 2.0f, ABS_ERROR);
    EXPECT_NEAR(box.Extents().y, 3.0f, ABS_ERROR);
    EXPECT_NEAR(box.Extents().z, 4.0f, ABS_ERROR);
}

TEST(OBB, Scale_NegativeFactor) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 3.0f, 4.0f));
    box.Scale(-2.0f);

    // Scale uses std::abs, so result is same as scale 2
    EXPECT_EQ(box.Extents(), Vector3f(4.0f, 6.0f, 8.0f));
}

// ---------------------- Contains ----------------------

TEST(OBB, Contains_PointAtCenter) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    EXPECT_TRUE(box.Contains(Vector3f(0.0f, 0.0f, 0.0f)));
}

TEST(OBB, Contains_PointInside) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    EXPECT_TRUE(box.Contains(Vector3f(2.0f, 3.0f, 4.0f)));
}

TEST(OBB, Contains_PointOutside) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    EXPECT_FALSE(box.Contains(Vector3f(6.0f, 0.0f, 0.0f)));
}

TEST(OBB, Contains_PointOnBoundary) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    EXPECT_TRUE(box.Contains(Vector3f(5.0f, 0.0f, 0.0f)));
    EXPECT_TRUE(box.Contains(Vector3f(-5.0f, 0.0f, 0.0f)));
}

TEST(OBB, Contains_PointInsideRotated) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 1.0f, 1.0f));
    float yaw = glm::radians(90.0f);
    box.SetAxes(Vector3f(0.0f, yaw, 0.0f));

    // After 90 degree yaw, local X-axis points along world Z
    // So extents of 5 along local X means 5 along world Z
    EXPECT_TRUE(box.Contains(Vector3f(0.0f, 0.0f, 4.0f)));
    EXPECT_FALSE(box.Contains(Vector3f(4.0f, 0.0f, 0.0f)));
}

TEST(OBB, Contains_PointInsideTranslated) {
    OBBf box(Vector3f(10.0f, 10.0f, 10.0f), Vector3f(2.0f, 2.0f, 2.0f));
    EXPECT_TRUE(box.Contains(Vector3f(11.0f, 11.0f, 11.0f)));
    EXPECT_FALSE(box.Contains(Vector3f(0.0f, 0.0f, 0.0f)));
}

TEST(OBB, Contains_PointBox) {
    OBBf box(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(0.0f, 0.0f, 0.0f));
    EXPECT_TRUE(box.Contains(Vector3f(5.0f, 5.0f, 5.0f)));
    EXPECT_FALSE(box.Contains(Vector3f(5.1f, 5.0f, 5.0f)));
}

// ---------------------- Transform Helpers ----------------------

TEST(OBB, TransformPointToWorld_Identity) {
    OBBf box(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Vector3f local(1.0f, 1.0f, 1.0f);
    Vector3f world = box.TransformPointToWorld(local);

    EXPECT_NEAR(world.x, 6.0f, ABS_ERROR);
    EXPECT_NEAR(world.y, 6.0f, ABS_ERROR);
    EXPECT_NEAR(world.z, 6.0f, ABS_ERROR);
}

TEST(OBB, TransformPointToLocal_Identity) {
    OBBf box(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Vector3f world(6.0f, 6.0f, 6.0f);
    Vector3f local = box.TransformPointToLocal(world);

    EXPECT_NEAR(local.x, 1.0f, ABS_ERROR);
    EXPECT_NEAR(local.y, 1.0f, ABS_ERROR);
    EXPECT_NEAR(local.z, 1.0f, ABS_ERROR);
}

TEST(OBB, TransformPointToWorld_Roundtrip) {
    OBBf box(Vector3f(3.0f, 4.0f, 5.0f), Vector3f(2.0f, 2.0f, 2.0f));
    box.SetAxes(Vector3f(glm::radians(30.0f), glm::radians(45.0f), glm::radians(60.0f)));

    Vector3f local(1.0f, -0.5f, 0.7f);
    Vector3f world = box.TransformPointToWorld(local);
    Vector3f back = box.TransformPointToLocal(world);

    EXPECT_NEAR(back.x, local.x, ABS_ERROR);
    EXPECT_NEAR(back.y, local.y, ABS_ERROR);
    EXPECT_NEAR(back.z, local.z, ABS_ERROR);
}

TEST(OBB, TransformDirectionToWorld_Identity) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f));
    Vector3f dir(1.0f, 0.0f, 0.0f);
    Vector3f worldDir = box.TransformDirectionToWorld(dir);

    EXPECT_NEAR(worldDir.x, 1.0f, ABS_ERROR);
    EXPECT_NEAR(worldDir.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(worldDir.z, 0.0f, ABS_ERROR);
}

TEST(OBB, TransformDirectionToLocal_Identity) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f));
    Vector3f dir(1.0f, 0.0f, 0.0f);
    Vector3f localDir = box.TransformDirectionToLocal(dir);

    EXPECT_NEAR(localDir.x, 1.0f, ABS_ERROR);
    EXPECT_NEAR(localDir.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(localDir.z, 0.0f, ABS_ERROR);
}

TEST(OBB, TransformDirectionToWorld_Roundtrip) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f));
    box.SetAxes(Vector3f(glm::radians(30.0f), glm::radians(45.0f), glm::radians(60.0f)));

    Vector3f dir(1.0f, 0.0f, 0.0f);
    Vector3f worldDir = box.TransformDirectionToWorld(dir);
    Vector3f back = box.TransformDirectionToLocal(worldDir);

    EXPECT_NEAR(back.x, dir.x, ABS_ERROR);
    EXPECT_NEAR(back.y, dir.y, ABS_ERROR);
    EXPECT_NEAR(back.z, dir.z, ABS_ERROR);
}

// ---------------------- Corners ----------------------

TEST(OBB, Corners_AxisAligned) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 2.0f, 3.0f));
    auto corners = box.Corners();

    EXPECT_NEAR(corners[0].x, -1.0f, ABS_ERROR);
    EXPECT_NEAR(corners[0].y, -2.0f, ABS_ERROR);
    EXPECT_NEAR(corners[0].z, -3.0f, ABS_ERROR);

    EXPECT_NEAR(corners[7].x, 1.0f, ABS_ERROR);
    EXPECT_NEAR(corners[7].y, 2.0f, ABS_ERROR);
    EXPECT_NEAR(corners[7].z, 3.0f, ABS_ERROR);
}

TEST(OBB, Corners_Translated) {
    OBBf box(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(1.0f, 1.0f, 1.0f));
    auto corners = box.Corners();

    // Corner 0 = center + axes * (-extents) = (5,5,5) + (-1,-1,-1) = (4,4,4)
    EXPECT_NEAR(corners[0].x, 4.0f, ABS_ERROR);
    EXPECT_NEAR(corners[0].y, 4.0f, ABS_ERROR);
    EXPECT_NEAR(corners[0].z, 4.0f, ABS_ERROR);

    // Corner 7 = center + axes * (extents) = (5,5,5) + (1,1,1) = (6,6,6)
    EXPECT_NEAR(corners[7].x, 6.0f, ABS_ERROR);
    EXPECT_NEAR(corners[7].y, 6.0f, ABS_ERROR);
    EXPECT_NEAR(corners[7].z, 6.0f, ABS_ERROR);
}

TEST(OBB, Corners_PointBox) {
    OBBf box(Vector3f(3.0f, 4.0f, 5.0f), Vector3f(0.0f, 0.0f, 0.0f));
    auto corners = box.Corners();

    for (int i = 0; i < 8; ++i) {
        EXPECT_NEAR(corners[i].x, 3.0f, ABS_ERROR);
        EXPECT_NEAR(corners[i].y, 4.0f, ABS_ERROR);
        EXPECT_NEAR(corners[i].z, 5.0f, ABS_ERROR);
    }
}

TEST(OBB, Corners_AllUnique) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 3.0f, 4.0f));
    box.SetAxes(Vector3f(glm::radians(30.0f), glm::radians(45.0f), 0.0f));
    auto corners = box.Corners();

    for (int i = 0; i < 8; ++i) {
        for (int j = i + 1; j < 8; ++j) {
            float dist = glm::length(corners[i] - corners[j]);
            EXPECT_GT(dist, ABS_ERROR) << "Corners " << i << " and " << j << " should be different";
        }
    }
}

TEST(OBB, Corners_GetCornersMatchesCorners) {
    OBBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(2.0f, 3.0f, 4.0f));
    box.SetAxes(Vector3f(glm::radians(15.0f), glm::radians(30.0f), glm::radians(45.0f)));

    auto corners1 = box.Corners();
    std::array<Vector3f, 8> corners2;
    box.GetCorners(corners2);

    for (int i = 0; i < 8; ++i) {
        EXPECT_NEAR(corners1[i].x, corners2[i].x, ABS_ERROR);
        EXPECT_NEAR(corners1[i].y, corners2[i].y, ABS_ERROR);
        EXPECT_NEAR(corners1[i].z, corners2[i].z, ABS_ERROR);
    }
}

// ---------------------- Lerp ----------------------

TEST(OBB, Lerp_Center) {
    OBBf box(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(2.0f, 3.0f, 4.0f));
    Vector3f result = box.Lerp(Vector3f(0.5f, 0.5f, 0.5f));

    EXPECT_NEAR(result.x, 5.0f, ABS_ERROR);
    EXPECT_NEAR(result.y, 5.0f, ABS_ERROR);
    EXPECT_NEAR(result.z, 5.0f, ABS_ERROR);
}

TEST(OBB, Lerp_MinCorner) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 3.0f, 4.0f));
    Vector3f result = box.Lerp(Vector3f(0.0f, 0.0f, 0.0f));

    EXPECT_NEAR(result.x, -2.0f, ABS_ERROR);
    EXPECT_NEAR(result.y, -3.0f, ABS_ERROR);
    EXPECT_NEAR(result.z, -4.0f, ABS_ERROR);
}

TEST(OBB, Lerp_MaxCorner) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 3.0f, 4.0f));
    Vector3f result = box.Lerp(Vector3f(1.0f, 1.0f, 1.0f));

    EXPECT_NEAR(result.x, 2.0f, ABS_ERROR);
    EXPECT_NEAR(result.y, 3.0f, ABS_ERROR);
    EXPECT_NEAR(result.z, 4.0f, ABS_ERROR);
}

TEST(OBB, Lerp_NonUniformT) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    Vector3f result = box.Lerp(Vector3f(0.25f, 0.5f, 0.75f));

    // local_point = extents * ((t * 2) - 1) = 5 * ((0.25*2)-1, (0.5*2)-1, (0.75*2)-1)
    //             = 5 * (-0.5, 0, 0.5) = (-2.5, 0, 2.5)
    EXPECT_NEAR(result.x, -2.5f, ABS_ERROR);
    EXPECT_NEAR(result.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.z, 2.5f, ABS_ERROR);
}

// ---------------------- ClosestPoint ----------------------

TEST(OBB, ClosestPoint_PointInside) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    Vector3f point(2.0f, 3.0f, 1.0f);
    Vector3f closest = box.ClosestPoint(point);

    EXPECT_NEAR(closest.x, point.x, ABS_ERROR);
    EXPECT_NEAR(closest.y, point.y, ABS_ERROR);
    EXPECT_NEAR(closest.z, point.z, ABS_ERROR);
}

TEST(OBB, ClosestPoint_PointOutsideAllAxes) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    Vector3f point(10.0f, 10.0f, 10.0f);
    Vector3f closest = box.ClosestPoint(point);

    EXPECT_NEAR(closest.x, 5.0f, ABS_ERROR);
    EXPECT_NEAR(closest.y, 5.0f, ABS_ERROR);
    EXPECT_NEAR(closest.z, 5.0f, ABS_ERROR);
}

TEST(OBB, ClosestPoint_PointOutsideNegative) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    Vector3f point(-10.0f, -10.0f, -10.0f);
    Vector3f closest = box.ClosestPoint(point);

    EXPECT_NEAR(closest.x, -5.0f, ABS_ERROR);
    EXPECT_NEAR(closest.y, -5.0f, ABS_ERROR);
    EXPECT_NEAR(closest.z, -5.0f, ABS_ERROR);
}

TEST(OBB, ClosestPoint_PointOutsideOneAxis) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    Vector3f point(0.0f, 0.0f, 10.0f);
    Vector3f closest = box.ClosestPoint(point);

    EXPECT_NEAR(closest.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(closest.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(closest.z, 5.0f, ABS_ERROR);
}

TEST(OBB, ClosestPoint_RotatedBox) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 1.0f, 1.0f));
    float yaw = glm::radians(90.0f);
    box.SetAxes(Vector3f(0.0f, yaw, 0.0f));

    // After 90 yaw, local X aligns with world Z
    // Point far along world Z should clamp to extent along that axis
    Vector3f point(0.0f, 0.0f, 10.0f);
    Vector3f closest = box.ClosestPoint(point);

    EXPECT_NEAR(closest.z, 5.0f, ABS_ERROR);
    EXPECT_NEAR(closest.y, 0.0f, ABS_ERROR);
}

// ---------------------- DistanceSquared ----------------------

TEST(OBB, DistanceSquared_PointInside) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    Vector3f point(2.0f, 3.0f, 1.0f);

    EXPECT_NEAR(box.DistanceSquared(point), 0.0f, ABS_ERROR);
}

TEST(OBB, DistanceSquared_PointOutside) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    // Point at (8, 0, 0) -> closest is (5, 0, 0) -> dist^2 = 9
    Vector3f point(8.0f, 0.0f, 0.0f);

    EXPECT_NEAR(box.DistanceSquared(point), 9.0f, ABS_ERROR);
}

// ---------------------- Distance ----------------------

TEST(OBB, Distance_PointInside) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    Vector3f point(2.0f, 3.0f, 1.0f);

    EXPECT_NEAR(box.Distance(point), 0.0f, ABS_ERROR);
}

TEST(OBB, Distance_PointOutside) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    // Point at (8, 0, 0) -> closest is (5, 0, 0) -> dist = 3
    Vector3f point(8.0f, 0.0f, 0.0f);

    EXPECT_NEAR(box.Distance(point), 3.0f, ABS_ERROR);
}

TEST(OBB, Distance_PointOutsideMultipleAxes) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    // Point at (8, 9, 5) -> closest is (5, 5, 5) -> dist = sqrt(9+16+0) = 5
    Vector3f point(8.0f, 9.0f, 5.0f);

    EXPECT_NEAR(box.Distance(point), 5.0f, ABS_ERROR);
}

// ---------------------- MaximumExtent ----------------------

TEST(OBB, MaximumExtent_XIsLargest) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(10.0f, 5.0f, 3.0f));
    EXPECT_EQ(box.MaximumExtent(), 0);
}

TEST(OBB, MaximumExtent_YIsLargest) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(3.0f, 10.0f, 5.0f));
    EXPECT_EQ(box.MaximumExtent(), 1);
}

TEST(OBB, MaximumExtent_ZIsLargest) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(3.0f, 5.0f, 10.0f));
    EXPECT_EQ(box.MaximumExtent(), 2);
}

TEST(OBB, MaximumExtent_AllEqual) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(5.0f, 5.0f, 5.0f));
    int result = box.MaximumExtent();
    EXPECT_TRUE(result >= 0 && result <= 2);
}

// ---------------------- ToAABB ----------------------

TEST(OBB, ToAABB_AxisAligned) {
    OBBf obb(Vector3f(5.0f, 5.0f, 5.0f), Vector3f(2.0f, 3.0f, 4.0f));
    AABBf aabb = obb.ToAABB();

    EXPECT_NEAR(aabb.Min().x, 3.0f, ABS_ERROR);
    EXPECT_NEAR(aabb.Min().y, 2.0f, ABS_ERROR);
    EXPECT_NEAR(aabb.Min().z, 1.0f, ABS_ERROR);
    EXPECT_NEAR(aabb.Max().x, 7.0f, ABS_ERROR);
    EXPECT_NEAR(aabb.Max().y, 8.0f, ABS_ERROR);
    EXPECT_NEAR(aabb.Max().z, 9.0f, ABS_ERROR);
}

TEST(OBB, ToAABB_Rotated_ContainsOBBCorners) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 1.0f, 1.0f));
    obb.SetAxes(Vector3f(0.0f, glm::radians(45.0f), 0.0f));

    AABBf aabb = obb.ToAABB();
    auto corners = obb.Corners();

    // All OBB corners must be inside the resulting AABB
    for (int i = 0; i < 8; ++i) {
        EXPECT_GE(corners[i].x, aabb.Min().x - ABS_ERROR);
        EXPECT_GE(corners[i].y, aabb.Min().y - ABS_ERROR);
        EXPECT_GE(corners[i].z, aabb.Min().z - ABS_ERROR);
        EXPECT_LE(corners[i].x, aabb.Max().x + ABS_ERROR);
        EXPECT_LE(corners[i].y, aabb.Max().y + ABS_ERROR);
        EXPECT_LE(corners[i].z, aabb.Max().z + ABS_ERROR);
    }
}

TEST(OBB, ToAABB_PointBox) {
    OBBf obb(Vector3f(3.0f, 4.0f, 5.0f), Vector3f(0.0f, 0.0f, 0.0f));
    AABBf aabb = obb.ToAABB();

    EXPECT_NEAR(aabb.Min().x, 3.0f, ABS_ERROR);
    EXPECT_NEAR(aabb.Min().y, 4.0f, ABS_ERROR);
    EXPECT_NEAR(aabb.Min().z, 5.0f, ABS_ERROR);
    EXPECT_NEAR(aabb.Max().x, 3.0f, ABS_ERROR);
    EXPECT_NEAR(aabb.Max().y, 4.0f, ABS_ERROR);
    EXPECT_NEAR(aabb.Max().z, 5.0f, ABS_ERROR);
}

// ---------------------- Operators ----------------------

TEST(OBB, OperatorPlus) {
    OBBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(1.0f, 1.0f, 1.0f));
    Vector3f offset(10.0f, 20.0f, 30.0f);

    OBBf result = box + offset;

    EXPECT_EQ(result.Center(), Vector3f(11.0f, 22.0f, 33.0f));
    EXPECT_EQ(result.Extents(), Vector3f(1.0f, 1.0f, 1.0f));
}

TEST(OBB, OperatorMinus) {
    OBBf box(Vector3f(10.0f, 20.0f, 30.0f), Vector3f(1.0f, 1.0f, 1.0f));
    Vector3f offset(5.0f, 10.0f, 15.0f);

    OBBf result = box - offset;

    EXPECT_EQ(result.Center(), Vector3f(5.0f, 10.0f, 15.0f));
    EXPECT_EQ(result.Extents(), Vector3f(1.0f, 1.0f, 1.0f));
}

TEST(OBB, OperatorPlusEquals) {
    OBBf box(Vector3f(1.0f, 2.0f, 3.0f), Vector3f(1.0f, 1.0f, 1.0f));
    Vector3f offset(10.0f, 20.0f, 30.0f);

    OBBf& result = (box += offset);

    EXPECT_EQ(box.Center(), Vector3f(11.0f, 22.0f, 33.0f));
    EXPECT_EQ(box.Extents(), Vector3f(1.0f, 1.0f, 1.0f));
    EXPECT_EQ(&result, &box);
}

TEST(OBB, OperatorMinusEquals) {
    OBBf box(Vector3f(10.0f, 20.0f, 30.0f), Vector3f(1.0f, 1.0f, 1.0f));
    Vector3f offset(5.0f, 10.0f, 15.0f);

    OBBf& result = (box -= offset);

    EXPECT_EQ(box.Center(), Vector3f(5.0f, 10.0f, 15.0f));
    EXPECT_EQ(box.Extents(), Vector3f(1.0f, 1.0f, 1.0f));
    EXPECT_EQ(&result, &box);
}

// ---------------------- Cast Operator ----------------------

TEST(OBB, CastOperator_FloatToDouble) {
    OBBf boxf(Vector3f(1.5f, 2.5f, 3.5f), Vector3f(4.0f, 5.0f, 6.0f));
    OBBd boxd = OBBd(boxf);

    EXPECT_NEAR(boxd.Center().x, 1.5, 1e-5);
    EXPECT_NEAR(boxd.Center().y, 2.5, 1e-5);
    EXPECT_NEAR(boxd.Center().z, 3.5, 1e-5);
    EXPECT_NEAR(boxd.Extents().x, 4.0, 1e-5);
    EXPECT_NEAR(boxd.Extents().y, 5.0, 1e-5);
    EXPECT_NEAR(boxd.Extents().z, 6.0, 1e-5);
}

TEST(OBB, CastOperator_DoubleToFloat) {
    OBBd boxd(Vector3<double>(1.5, 2.5, 3.5), Vector3<double>(4.0, 5.0, 6.0));
    OBBf boxf = OBBf(boxd);

    EXPECT_NEAR(boxf.Center().x, 1.5f, ABS_ERROR);
    EXPECT_NEAR(boxf.Center().y, 2.5f, ABS_ERROR);
    EXPECT_NEAR(boxf.Center().z, 3.5f, ABS_ERROR);
    EXPECT_NEAR(boxf.Extents().x, 4.0f, ABS_ERROR);
    EXPECT_NEAR(boxf.Extents().y, 5.0f, ABS_ERROR);
    EXPECT_NEAR(boxf.Extents().z, 6.0f, ABS_ERROR);
}

// ---------------------- Operators preserve axes ----------------------

TEST(OBB, OperatorPlus_PreservesAxes) {
    OBBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 2.0f, 3.0f));
    box.SetAxes(Vector3f(glm::radians(30.0f), glm::radians(45.0f), 0.0f));
    Matrix3f originalAxes = box.Axes();

    OBBf result = box + Vector3f(5.0f, 5.0f, 5.0f);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(result.Axes()[i][j], originalAxes[i][j], ABS_ERROR);
}

TEST(OBB, OperatorMinus_PreservesAxes) {
    OBBf box(Vector3f(10.0f, 10.0f, 10.0f), Vector3f(1.0f, 2.0f, 3.0f));
    box.SetAxes(Vector3f(glm::radians(30.0f), glm::radians(45.0f), 0.0f));
    Matrix3f originalAxes = box.Axes();

    OBBf result = box - Vector3f(5.0f, 5.0f, 5.0f);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            EXPECT_NEAR(result.Axes()[i][j], originalAxes[i][j], ABS_ERROR);
}
