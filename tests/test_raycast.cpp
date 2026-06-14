#include <gtest/gtest.h>

#include "BoundingBox.hpp"
#include "Ray.hpp"

#ifdef MATH_ESSENTIALS_NAMESPACE
using namespace MATH_ESSENTIALS_NAMESPACE;
#endif

constexpr float ABS_ERROR = std::numeric_limits<float>::epsilon() * 64.0f;

// ---------------------- Ray vs AABB Intersection ----------------------

TEST(Raycast, Cast_RayHitsBoxFront) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 1.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 5.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.x, -1.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.z, 0.0f, ABS_ERROR);
}

TEST(Raycast, Cast_RayHitsBoxBack) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(5.0f, 1.0f, 1.0f),
        Vector3f(-1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 3.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 2.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.x, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.z, 0.0f, ABS_ERROR);
}

TEST(Raycast, Cast_RayHitsBoxTop) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(1.0f, 5.0f, 1.0f),
        Vector3f(0.0f, -1.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 3.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 2.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.y, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.z, 0.0f, ABS_ERROR);
}

TEST(Raycast, Cast_RayHitsBoxBottom) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(1.0f, -5.0f, 1.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 5.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.y, -1.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.z, 0.0f, ABS_ERROR);
}

TEST(Raycast, Cast_RayHitsBoxLeft) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(1.0f, 1.0f, -5.0f),
        Vector3f(0.0f, 0.0f, 1.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 5.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.z, -1.0f, ABS_ERROR);
}

TEST(Raycast, Cast_RayHitsBoxRight) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(1.0f, 1.0f, 5.0f),
        Vector3f(0.0f, 0.0f, -1.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 3.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 2.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.z, 1.0f, ABS_ERROR);
}

TEST(Raycast, Cast_RayMissesBox) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 5.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_FALSE(result.hit);
}

TEST(Raycast, Cast_RayOriginInsideBox) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(1.0f, 1.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 2.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 1.0f, ABS_ERROR);
}

TEST(Raycast, Cast_RayPointsAwayFromBox) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 1.0f, 1.0f),
        Vector3f(-1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_FALSE(result.hit);
}

TEST(Raycast, Cast_RayAtAngle) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(-5.0f, -5.0f, 1.0f),
        Vector3f(1.0f, 1.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 5.0f * std::sqrt(2.0f), ABS_ERROR);
    EXPECT_NEAR(result.position.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 1.0f, ABS_ERROR);
}

TEST(Raycast, Cast_RayHitsCorner) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(-5.0f, -5.0f, -5.0f),
        Vector3f(1.0f, 1.0f, 1.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 5.0f * std::sqrt(3.0f), ABS_ERROR);
    EXPECT_NEAR(result.position.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 0.0f, ABS_ERROR);
}

TEST(Raycast, Cast_RayHitsEdge) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 5.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 0.0f, ABS_ERROR);
}

TEST(Raycast, Cast_NegativeBox) {
    AABBf box(Vector3f(-5.0f, -5.0f, -5.0f), Vector3f(-2.0f, -2.0f, -2.0f));
    Ray<float> ray {
        Vector3f(-10.0f, -3.5f, -3.5f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 5.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, -5.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, -3.5f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, -3.5f, ABS_ERROR);
}

TEST(Raycast, Cast_LargeBox) {
    AABBf box(Vector3f(-100.0f, -100.0f, -100.0f), Vector3f(100.0f, 100.0f, 100.0f));
    Ray<float> ray {
        Vector3f(-200.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 100.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, -100.0f, ABS_ERROR);
}

TEST(Raycast, Cast_SmallBox) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.1f, 0.1f, 0.1f));
    Ray<float> ray {
        Vector3f(-1.0f, 0.05f, 0.05f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 0.05f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 0.05f, ABS_ERROR);
}

TEST(Raycast, Cast_RayParallelToBoxFace) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 3.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_FALSE(result.hit);
}

TEST(Raycast, Cast_DoubleType) {
    AABBd box(Vector3<double>(0.0, 0.0, 0.0), Vector3<double>(2.0, 2.0, 2.0));
    Ray<double> ray {
        Vector3<double>(-5.0, 1.0, 1.0),
        Vector3<double>(1.0, 0.0, 0.0),
    };

    RayIntersection<double> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 5.0, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 0.0, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 1.0, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 1.0, ABS_ERROR);
}

TEST(Raycast, Cast_RayOriginOnFace) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(0.0f, 1.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 1.0f, ABS_ERROR);
    EXPECT_FALSE(result.inside);
}

TEST(Raycast, Cast_RayOriginOnCorner) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 1.0f, 1.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 0.0f, ABS_ERROR);
    EXPECT_FALSE(result.inside);
}

TEST(Raycast, Cast_RayNearParallelToAxis) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 1.0f, 1.0f),
        Vector3f(1.0f, 1e-7f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.position.x, 0.0f, ABS_ERROR);
}

TEST(Raycast, Cast_InsideFlagWhenOriginInside) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(1.0f, 1.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_TRUE(result.inside);
}

TEST(Raycast, Cast_InsideFlagWhenOriginOutside) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 1.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_FALSE(result.inside);
}

TEST(Raycast, Cast_VeryLongDistance) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(-10000.0f, 1.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 10000.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 0.0f, ABS_ERROR);
}

TEST(Raycast, Cast_BoxAtOriginVsOffsetBox) {
    AABBf box1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    AABBf box2(Vector3f(10.0f, 10.0f, 10.0f), Vector3f(12.0f, 12.0f, 12.0f));

    Ray<float> ray1 {
        Vector3f(-5.0f, 1.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    Ray<float> ray2 {
        Vector3f(5.0f, 11.0f, 11.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result1 = ray1.Intersect(box1);
    RayIntersection<float> result2 = ray2.Intersect(box2);

    EXPECT_TRUE(result1.hit);
    EXPECT_TRUE(result2.hit);
    EXPECT_NEAR(result1.distance, 5.0f, ABS_ERROR);
    EXPECT_NEAR(result2.distance, 5.0f, ABS_ERROR);
}

TEST(Raycast, Cast_AllNegativeDirectionComponents) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));
    Ray<float> ray {
        Vector3f(5.0f, 5.0f, 5.0f),
        Vector3f(-1.0f, -1.0f, -1.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, std::sqrt(27.0f), ABS_ERROR);
    EXPECT_NEAR(result.position.x, 2.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 2.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 2.0f, ABS_ERROR);
}

TEST(Raycast, Cast_NormalDirectionCorrect) {
    AABBf box(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f));

    Ray<float> ray_x {
        Vector3f(-5.0f, 1.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };
    RayIntersection<float> resultX = ray_x.Intersect(box);
    EXPECT_NEAR(glm::length(resultX.normal), 1.0f, ABS_ERROR);

    Ray<float> ray_y {
        Vector3f(1.0f, -5.0f, 1.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
    };
    RayIntersection<float> resultY = ray_y.Intersect(box);
    EXPECT_NEAR(glm::length(resultY.normal), 1.0f, ABS_ERROR);

    Ray<float> ray_z {
        Vector3f(1.0f, 1.0f, -5.0f),
        Vector3f(0.0f, 0.0f, 1.0f),
    };
    RayIntersection<float> resultZ = ray_z.Intersect(box);
    EXPECT_NEAR(glm::length(resultZ.normal), 1.0f, ABS_ERROR);
}

TEST(Raycast, Cast_ZeroSizeBox) {
    Vector3f point(1.0f, 1.0f, 1.0f);
    AABBf box(point, point);

    Ray<float> ray {
        Vector3f(-5.0f, 1.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(box);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 6.0f, ABS_ERROR);
}

// ---------------------- Ray vs OBB Intersection ----------------------

TEST(Raycast, OBB_UnrotatedBoxBehavesLikeAABB) {
    OBBf obb(Vector3f(1.0f, 1.0f, 1.0f), Vector3f(1.0f, 1.0f, 1.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 1.0f, 1.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 5.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 1.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.x, -1.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.normal.z, 0.0f, ABS_ERROR);
}

TEST(Raycast, OBB_RotatedAroundYAxis45Degrees) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    float expected_distance = 5.0f - std::sqrt(2.0f);
    EXPECT_NEAR(result.distance, expected_distance, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 0.0f, ABS_ERROR);
}

TEST(Raycast, OBB_RotatedAroundXAxis90Degrees) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(glm::radians(90.0f), 0.0f, 0.0f));
    Ray<float> ray {
        Vector3f(0.0f, -5.0f, 0.0f),
        Vector3f(0.0f, 1.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 4.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, -1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 0.0f, ABS_ERROR);
}

TEST(Raycast, OBB_RotatedAroundZAxis90Degrees) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, 0.0f, glm::radians(90.0f)));
    Ray<float> ray {
        Vector3f(-5.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 4.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.x, -1.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.y, 0.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, 0.0f, ABS_ERROR);
}

TEST(Raycast, OBB_RotatedAroundAllAxes) {
    OBBf obb(
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 1.0f, 1.0f),
        Vector3f(glm::radians(30.0f), glm::radians(45.0f), glm::radians(60.0f)));
    Ray<float> ray {
        Vector3f(-10.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_GT(result.distance, 0.0f);
    EXPECT_LT(result.distance, 11.0f);
}

TEST(Raycast, OBB_RayMissesRotatedBox) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 5.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_FALSE(result.hit);
}

TEST(Raycast, OBB_RayOriginInsideRotatedBox) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    Ray<float> ray {
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_TRUE(result.inside);
    EXPECT_GT(result.distance, 0.0f);
}

TEST(Raycast, OBB_RayNearFaceOfRotatedBox) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    Vector3f face_point = obb.TransformPointToWorld(Vector3f(-1.0f, 0.0f, 0.0f));
    Vector3f face_normal = obb.TransformDirectionToWorld(Vector3f(-1.0f, 0.0f, 0.0f));
    Vector3f ray_origin = face_point - face_normal * 5.0f;

    Ray<float> ray {
        ray_origin,
        face_normal,
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_GT(result.distance, 0.0f);
    EXPECT_LT(result.distance, 6.0f);
    EXPECT_FALSE(result.inside);
}

TEST(Raycast, OBB_TranslatedAndRotated) {
    OBBf obb(Vector3f(10.0f, 5.0f, -3.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(90.0f), 0.0f));
    Ray<float> ray {
        Vector3f(5.0f, 5.0f, -3.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.position.y, 5.0f, ABS_ERROR);
    EXPECT_NEAR(result.position.z, -3.0f, ABS_ERROR);
}

TEST(Raycast, OBB_NormalTransformationCorrect) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(90.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    float normal_length = glm::length(result.normal);
    EXPECT_NEAR(normal_length, 1.0f, ABS_ERROR);

    EXPECT_NEAR(std::abs(result.normal.x) + std::abs(result.normal.y) + std::abs(result.normal.z), 1.0f, ABS_ERROR);
}

TEST(Raycast, OBB_DiagonalRayThroughRotatedBox) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-5.0f, -5.0f, -5.0f),
        Vector3f(1.0f, 1.0f, 1.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_GT(result.distance, 0.0f);
}

TEST(Raycast, OBB_RayPointsAwayFromRotatedBox) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 0.0f, 0.0f),
        Vector3f(-1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_FALSE(result.hit);
}

TEST(Raycast, OBB_SmallRotatedBox) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(0.1f, 0.1f, 0.1f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-1.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_GT(result.distance, 0.0f);
    EXPECT_LT(result.distance, 1.0f);
}

TEST(Raycast, OBB_LargeRotatedBox) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(100.0f, 100.0f, 100.0f), Vector3f(0.0f, glm::radians(30.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-200.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_GT(result.distance, 0.0f);
}

TEST(Raycast, OBB_NonUniformExtentsRotated) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 1.0f, 0.5f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-10.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_GT(result.distance, 0.0f);
}

TEST(Raycast, OBB_RayParallelToRotatedFace) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 3.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_FALSE(result.hit);
}

TEST(Raycast, OBB_DoubleType) {
    OBBd obb(Vector3<double>(0.0, 0.0, 0.0), Vector3<double>(1.0, 1.0, 1.0), Vector3<double>(0.0, glm::radians(45.0), 0.0));
    Ray<double> ray {
        Vector3<double>(-5.0, 0.0, 0.0),
        Vector3<double>(1.0, 0.0, 0.0),
    };

    RayIntersection<double> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_GT(result.distance, 0.0);
}

TEST(Raycast, OBB_180DegreeRotation) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(180.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_NEAR(result.distance, 4.0f, ABS_ERROR);
}

TEST(Raycast, OBB_MultipleRotatedBoxesDistanceComparison) {
    OBBf obb1(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    OBBf obb2(Vector3f(10.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));

    Ray<float> ray {
        Vector3f(-10.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result1 = ray.Intersect(obb1);
    RayIntersection<float> result2 = ray.Intersect(obb2);

    EXPECT_TRUE(result1.hit);
    EXPECT_TRUE(result2.hit);
    EXPECT_LT(result1.distance, result2.distance);
}

TEST(Raycast, OBB_DiagonalRayFromMultipleAngles) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));

    Ray<float> ray1 {
        Vector3f(-10.0f, -10.0f, 0.0f),
        glm::normalize(Vector3f(1.0f, 1.0f, 0.0f)),
    };

    Ray<float> ray2 {
        Vector3f(10.0f, -10.0f, 0.0f),
        glm::normalize(Vector3f(-1.0f, 1.0f, 0.0f)),
    };

    RayIntersection<float> result1 = ray1.Intersect(obb);
    RayIntersection<float> result2 = ray2.Intersect(obb);

    EXPECT_TRUE(result1.hit);
    EXPECT_TRUE(result2.hit);
    EXPECT_GT(result1.distance, 0.0f);
    EXPECT_GT(result2.distance, 0.0f);
}

TEST(Raycast, OBB_NegativeCoordinatesRotated) {
    OBBf obb(Vector3f(-10.0f, -10.0f, -10.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-20.0f, -10.0f, -10.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_GT(result.distance, 0.0f);
}

TEST(Raycast, OBB_VeryLongDistanceRotated) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(30.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-10000.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_GT(result.distance, 9998.0f);
    EXPECT_LT(result.distance, 10001.0f);
}

TEST(Raycast, OBB_InsideFlagWhenOriginInside) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(2.0f, 2.0f, 2.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    Ray<float> ray {
        Vector3f(0.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_TRUE(result.inside);
}

TEST(Raycast, OBB_InsideFlagWhenOriginOutside) {
    OBBf obb(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    Ray<float> ray {
        Vector3f(-5.0f, 0.0f, 0.0f),
        Vector3f(1.0f, 0.0f, 0.0f),
    };

    RayIntersection<float> result = ray.Intersect(obb);

    EXPECT_TRUE(result.hit);
    EXPECT_FALSE(result.inside);
}
