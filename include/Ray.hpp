/*  ══════════════════════════════════════════════════════════════════════════
 *  C++ Math Essentials Library
 *  ──────────────────────────────────────────────────────────────────────────
 *  Copyright (c) 2026 Pablo Bermejo Hernández
 *  Licensed under the MIT License - See LICENSE file for details
 *  ══════════════════════════════════════════════════════════════════════════
 */

#pragma once

#include <concepts>
#include <limits>

#include "BoundingBox.hpp"
#include "Vector.hpp"

#ifdef MATH_ESSENTIALS_NAMESPACE
#define NAMESPACE_NAME MATH_ESSENTIALS_NAMESPACE
#else
#define NAMESPACE_NAME
#endif

#ifdef MATH_ESSENTIALS_NAMESPACE
namespace NAMESPACE_NAME {
#endif

template <typename T>
concept ray_type = std::floating_point<T>;

template <ray_type T>
struct RayIntersection {
    bool hit = false;
    bool inside = false;
    T distance = 0;
    Vector3<T> position = Vector3<T>(0);
    Vector3<T> normal = Vector3<T>(0);

    [[nodiscard]] static constexpr RayIntersection<T> Miss() noexcept {
        return RayIntersection<T> {false, false, 0, Vector3<T> {0}, Vector3<T> {0}};
    }

    [[nodiscard]] constexpr operator bool() const noexcept { return hit; }
};

template <ray_type T>
struct Ray {
   private:
    Vector3<T> m_origin = Vector3<T>(0);
    Vector3<T> m_direction = Vector3<T>(0);

   public:
    constexpr Ray() noexcept = default;
    constexpr Ray(Vector3<T> origin, Vector3<T> direction) noexcept : m_origin(origin), m_direction(Normalize(direction)) {}

    [[nodiscard]] constexpr const Vector3<T>& Origin() const noexcept { return m_origin; }
    [[nodiscard]] constexpr const Vector3<T>& Direction() const noexcept { return m_direction; }

    constexpr void SetOrigin(Vector3<T> origin) noexcept { m_origin = origin; }
    constexpr void SetDirection(Vector3<T> direction) noexcept { m_direction = Normalize(direction); }

    [[nodiscard]] constexpr Vector3<T> PointAt(T t) const noexcept { return m_origin + (m_direction * t); }

   private:
    [[nodiscard]] static constexpr Vector3<T> Normalize(Vector3<T> v) noexcept {
        if (glm::dot(v, v) > std::numeric_limits<T>::epsilon()) {
            return glm::normalize(v);
        } else {
            return Vector3<T>(0);
        }
    }

   private:
    [[nodiscard]] static constexpr RayIntersection<T> RayIntersectAABB(Vector3<T> origin, Vector3<T> direction, Vector3<T> min, Vector3<T> max) noexcept {
        if (glm::dot(direction, direction) == 0) {  // Check for invalid rays
            return RayIntersection<T>::Miss();
        }

        Vector3<T> inv_dir = T {1} / direction;

        Vector3<T> t0 = (min - origin) * inv_dir;
        Vector3<T> t1 = (max - origin) * inv_dir;

        Vector3<T> t_min = glm::min(t0, t1);
        Vector3<T> t_max = glm::max(t0, t1);

        T t_near = std::max({t_min.x, t_min.y, t_min.z});
        T t_far = std::min({t_max.x, t_max.y, t_max.z});

        if (t_near > t_far || t_far < T {0}) {
            return RayIntersection<T>::Miss();
        }

        bool inside = t_near < T {0};
        T t_hit = inside ? t_far : t_near;

        Vector3<T> normal = Vector3<T>(0);

        Vector3<T> normal_test = inside ? t_max : t_min;
        if (t_hit == normal_test.x) {
            normal.x = -glm::sign(direction.x);
        } else if (t_hit == normal_test.y) {
            normal.y = -glm::sign(direction.y);
        } else {
            normal.z = -glm::sign(direction.z);
        }

        RayIntersection<T> result = {
            .hit = true,
            .inside = inside,
            .distance = t_hit,
            .position = origin + (direction * t_hit),
            .normal = normal,
        };

        return result;
    }

   public:
    template <aabb_type U>
    [[nodiscard]] constexpr RayIntersection<T> Intersect(Vector3<U> min, Vector3<U> max) const noexcept {
        return RayIntersectAABB(m_origin, m_direction, static_cast<Vector3<T>>(min), static_cast<Vector3<T>>(max));
    }

    template <aabb_type U>
    [[nodiscard]] constexpr RayIntersection<T> Intersect(const AABB<U>& aabb) const noexcept {
        return RayIntersectAABB(m_origin, m_direction, static_cast<Vector3<T>>(aabb.Min()), static_cast<Vector3<T>>(aabb.Max()));
    }

    template <obb_type U>
    [[nodiscard]] constexpr RayIntersection<T> Intersect(const OBB<U>& obb) const noexcept {
        Vector3<T> local_origin = obb.TransformPointToLocal(m_origin);
        Vector3<T> local_direction = obb.TransformDirectionToLocal(m_direction);

        Vector3<T> aabb_min = -obb.Extents();
        Vector3<T> aabb_max = obb.Extents();

        RayIntersection<T> result = RayIntersectAABB(local_origin, local_direction, aabb_min, aabb_max);

        if (result.hit) {
            result.position = PointAt(result.distance);

            Vector3<T> world_normal = obb.TransformDirectionToWorld(result.normal);
            result.normal = glm::normalize(world_normal);
        }

        return result;
    }
};

using Rayf = Ray<float>;
using Rayd = Ray<double>;

using RayIntersectionf = RayIntersection<float>;
using RayIntersectiond = RayIntersection<double>;

#ifdef MATH_ESSENTIALS_NAMESPACE
}  // namespace NAMESPACE_NAME
#endif