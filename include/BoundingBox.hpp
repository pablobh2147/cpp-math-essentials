/*  ══════════════════════════════════════════════════════════════════════════
 *  C++ Math Essentials Library
 *  ──────────────────────────────────────────────────────────────────────────
 *  Copyright (c) 2026 Pablo Bermejo Hernández
 *  Licensed under the MIT License - See LICENSE file for details
 *  ══════════════════════════════════════════════════════════════════════════
 */

#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/euler_angles.hpp>

#include <array>
#include <cstdint>
#include <span>
#include <type_traits>

#include "Matrix.hpp"
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
concept aabb_type = std::is_arithmetic_v<T>;

template <typename T>
concept obb_type = std::floating_point<T>;

template <aabb_type T>
class AABB {
   private:
    Vector3<T> m_min;
    Vector3<T> m_max;

   public:
    constexpr AABB() noexcept : m_min(0), m_max(0) {}
    constexpr AABB(Vector3<T> min, Vector3<T> max) noexcept { SetBounds(min, max); }

    // ---------------------- Getters ----------------------

    [[nodiscard]] constexpr const Vector3<T>& Min() const noexcept { return m_min; }
    [[nodiscard]] constexpr const Vector3<T>& Max() const noexcept { return m_max; }
    [[nodiscard]] constexpr Vector3<T> Center() const noexcept { return (m_min + m_max) / T {2}; }
    [[nodiscard]] constexpr Vector3<T> Centroid() const noexcept { return Center(); }
    [[nodiscard]] constexpr Vector3<T> Size() const noexcept { return m_max - m_min; }
    [[nodiscard]] constexpr Vector3<T> Extents() const noexcept { return (m_max - m_min) / T {2}; }

    [[nodiscard]] constexpr T Volume() const noexcept {
        const Vector3<T> d = Size();
        return d.x * d.y * d.z;
    }

    [[nodiscard]] constexpr T SurfaceArea() const noexcept {
        const Vector3<T> d = Size();
        return 2 * (d.x * d.y + d.y * d.z + d.z * d.x);
    }

    // ---------------------- Setters ----------------------

    constexpr void SetMin(Vector3<T> min) noexcept { SetBounds(min, m_max); }
    constexpr void SetMax(Vector3<T> max) noexcept { SetBounds(m_min, max); }

    constexpr void SetBounds(Vector3<T> a, Vector3<T> b) noexcept {
        m_min = glm::min(a, b);
        m_max = glm::max(a, b);
    }

    constexpr void SetCenterExtents(Vector3<T> center, Vector3<T> extents) noexcept {
        extents = glm::abs(extents);
        m_min = center - extents;
        m_max = center + extents;
    }

    // ---------------------- Geometric Transformation ----------------------

    constexpr void Translate(Vector3<T> delta) noexcept {
        m_max += delta;
        m_min += delta;
    }

    constexpr void Expand(Vector3<T> offset) noexcept {
        SetBounds(m_min - offset, m_max + offset);
    }

    constexpr void Scale(T factor, Vector3<T> center) noexcept {
        const Vector3<T> min = center + ((m_min - center) * factor);
        const Vector3<T> max = center + ((m_max - center) * factor);
        SetBounds(min, max);
    }

    constexpr void Scale(T factor) noexcept {
        Scale(factor, Center());
    }

    // ---------------------- Static Factory Methods ----------------------

    [[nodiscard]] static constexpr AABB<T> FromMinMax(Vector3<T> min, Vector3<T> max) noexcept {
        return AABB<T>(min, max);
    }

    [[nodiscard]] static constexpr AABB<T> FromCenterExtents(Vector3<T> center, Vector3<T> extents) noexcept {
        extents = glm::abs(extents);
        return AABB<T>(center - extents, center + extents);
    }

    // ---------------------- Intersections ----------------------

    [[nodiscard]] constexpr bool Contains(Vector3<T> point) const noexcept {
        return (
            point.x >= m_min.x && point.x <= m_max.x &&
            point.y >= m_min.y && point.y <= m_max.y &&
            point.z >= m_min.z && point.z <= m_max.z  //
        );
    }

    [[nodiscard]] constexpr bool Contains(const AABB<T>& other) const noexcept {
        return (
            other.m_min.x >= m_min.x && other.m_max.x <= m_max.x &&
            other.m_min.y >= m_min.y && other.m_max.y <= m_max.y &&
            other.m_min.z >= m_min.z && other.m_max.z <= m_max.z  //
        );
    }

    [[nodiscard]] constexpr bool Intersects(const AABB<T>& other) const noexcept {
        return (
            other.m_min.x <= m_max.x && other.m_max.x >= m_min.x &&
            other.m_min.y <= m_max.y && other.m_max.y >= m_min.y &&
            other.m_min.z <= m_max.z && other.m_max.z >= m_min.z  //
        );
    }

    [[nodiscard]] constexpr bool Intersects(const AABB<T>& other, AABB<T>& intersection) const noexcept {
        bool intersects = Intersects(other);

        if (intersects) {
            intersection.m_max = glm::min(m_max, other.m_max);
            intersection.m_min = glm::max(m_min, other.m_min);
        }

        return intersects;
    }

    // ---------------------- Utilities ----------------------

    constexpr void Merge(const AABB<T>& other) noexcept {
        m_min = glm::min(m_min, other.m_min);
        m_max = glm::max(m_max, other.m_max);
    }

    [[nodiscard]] constexpr static AABB<T> Merge(std::span<const AABB<T>> boxes) noexcept {
        if (boxes.empty()) {
            return {};
        }

        Vector3<T> min = boxes[0].m_min;
        Vector3<T> max = boxes[0].m_max;

        for (auto it = boxes.begin() + 1; it < boxes.end(); ++it) {
            min = glm::min(min, it->m_min);
            max = glm::max(max, it->m_max);
        }

        return AABB<T>(min, max);
    }

    template <typename F>
        requires std::floating_point<F>
    [[nodiscard]] constexpr Vector3<F> Lerp(Vector3<F> t) const noexcept {
        const Vector3<F> fmin = static_cast<Vector3<F>>(m_min);
        const Vector3<F> fsize = static_cast<Vector3<F>>(Size());
        return fmin + (fsize * t);
    }

    template <typename F>
        requires std::floating_point<F>
    [[nodiscard]] constexpr Vector3<F> Offset(Vector3<F> point) const noexcept {
        const Vector3<F> fmin = static_cast<Vector3<F>>(m_min);
        const Vector3<F> fsize = static_cast<Vector3<F>>(Size());

        Vector3<F> result;
        result.x = (fsize.x > 0) ? (point.x - fmin.x) / fsize.x : F {0};
        result.y = (fsize.y > 0) ? (point.y - fmin.y) / fsize.y : F {0};
        result.z = (fsize.z > 0) ? (point.z - fmin.z) / fsize.z : F {0};
        return result;
    }

    constexpr void Enclose(Vector3<T> point) noexcept {
        m_min = glm::min(m_min, point);
        m_max = glm::max(m_max, point);
    }

    [[nodiscard]] constexpr static AABB<T> Enclose(std::span<const Vector3<T>> points) noexcept {
        if (points.empty()) {
            return {};
        }

        Vector3<T> min = points[0];
        Vector3<T> max = points[0];

        for (auto it = points.begin() + 1; it < points.end(); ++it) {
            min = glm::min(min, *it);
            max = glm::max(max, *it);
        }

        return AABB<T>(min, max);
    }

    constexpr void GetCorners(std::span<Vector3<T>, 8> corners) const noexcept {
        corners[0] = m_min;
        corners[1] = Vector3<T>(m_max.x, m_min.y, m_min.z);
        corners[2] = Vector3<T>(m_max.x, m_max.y, m_min.z);
        corners[3] = Vector3<T>(m_min.x, m_max.y, m_min.z);
        corners[4] = Vector3<T>(m_min.x, m_min.y, m_max.z);
        corners[5] = Vector3<T>(m_max.x, m_min.y, m_max.z);
        corners[6] = Vector3<T>(m_min.x, m_max.y, m_max.z);
        corners[7] = m_max;
    }

    [[nodiscard]] constexpr std::array<Vector3<T>, 8> Corners() const noexcept {
        std::array<Vector3<T>, 8> corners;
        GetCorners(corners);
        return corners;
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    [[nodiscard]] constexpr Vector3<U> ClosestPoint(Vector3<U> point) const noexcept {
        return glm::clamp(point, static_cast<Vector3<U>>(m_min), static_cast<Vector3<U>>(m_max));
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    [[nodiscard]] constexpr U DistanceSquared(Vector3<U> point) const noexcept {
        const Vector3<U> closest = ClosestPoint(point);
        const Vector3<U> diff = point - closest;
        return (diff.x * diff.x) + (diff.y * diff.y) + (diff.z * diff.z);
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    [[nodiscard]] constexpr U Distance(Vector3<U> point) const noexcept {
        return glm::sqrt(DistanceSquared(point));
    }

    [[nodiscard]] constexpr int MaximumExtent() const noexcept {
        const Vector3<T> extent = Size();
        if (extent.x > extent.y && extent.x > extent.z) {
            return 0;
        } else if (extent.y > extent.z) {
            return 1;
        } else {
            return 2;
        }
    }

    // ---------------------- Operators ----------------------

    [[nodiscard]] constexpr AABB<T> operator+(Vector3<T> offset) const noexcept {
        return AABB<T>(m_min + offset, m_max + offset);
    }

    [[nodiscard]] constexpr AABB<T> operator-(Vector3<T> offset) const noexcept {
        return AABB<T>(m_min - offset, m_max - offset);
    }

    constexpr AABB<T>& operator+=(Vector3<T> offset) noexcept {
        m_min += offset;
        m_max += offset;
        return *this;
    }

    constexpr AABB<T>& operator-=(Vector3<T> offset) noexcept {
        m_min -= offset;
        m_max -= offset;
        return *this;
    }

    constexpr AABB<T> operator|(const AABB<T>& other) const noexcept {
        return AABB<T>(glm::min(m_min, other.m_min), glm::max(m_max, other.m_max));
    }

    constexpr AABB<T>& operator|=(const AABB<T>& other) noexcept {
        Merge(other);
        return *this;
    }

    [[nodiscard]] constexpr bool operator==(const AABB<T>& other) const noexcept {
        return m_min == other.m_min && m_max == other.m_max;
    }

    [[nodiscard]] constexpr bool operator!=(const AABB<T>& other) const noexcept {
        return !(*this == other);
    }

    template <typename U>
        requires std::is_arithmetic_v<U>
    [[nodiscard]] constexpr explicit operator AABB<U>() const noexcept {
        return AABB<U>(static_cast<Vector3<U>>(m_min), static_cast<Vector3<U>>(m_max));
    }
};

template <obb_type T>
class OBB {
   private:
    Vector3<T> m_center;
    Vector3<T> m_extents;
    Matrix3<T> m_axes;

   public:
    constexpr OBB() noexcept : m_center(0), m_extents(0), m_axes(1) {}
    constexpr OBB(Vector3<T> center, Vector3<T> extents) noexcept : m_center(center), m_extents(glm::abs(extents)), m_axes(1) {}
    constexpr OBB(Vector3<T> center, Vector3<T> extents, const Matrix3<T>& axes) noexcept : m_center(center), m_extents(glm::abs(extents)), m_axes(axes) {}
    constexpr OBB(Vector3<T> center, Vector3<T> extents, Vector3<T> euler_angles) noexcept : OBB(center, extents) { SetAxes(euler_angles); }

    // ---------------------- Getters ----------------------

    [[nodiscard]] constexpr const Vector3<T>& Center() const noexcept { return m_center; }
    [[nodiscard]] constexpr const Vector3<T>& Centroid() const noexcept { return m_center; }
    [[nodiscard]] constexpr Vector3<T> Size() const noexcept { return m_extents * T {2}; }
    [[nodiscard]] constexpr const Vector3<T>& Extents() const noexcept { return m_extents; }
    [[nodiscard]] constexpr const Matrix3<T>& Axes() const noexcept { return m_axes; }

    [[nodiscard]] constexpr T Volume() const noexcept {
        const Vector3<T> d = Size();
        return d.x * d.y * d.z;
    }

    [[nodiscard]] constexpr T SurfaceArea() const noexcept {
        const Vector3<T> d = Size();
        return 2 * ((d.x * d.y) + (d.y * d.z) + (d.z * d.x));
    }

    // ---------------------- Setters ----------------------

    constexpr void SetCenter(Vector3<T> center) noexcept { m_center = center; }
    constexpr void SetExtents(Vector3<T> extents) noexcept { m_extents = glm::abs(extents); }

    constexpr void SetAxes(const Matrix3<T>& axes) noexcept {
        m_axes = axes;
    }

    constexpr void SetAxes(Vector3<T> euler_angles) noexcept {
        m_axes = Matrix3<T>(glm::eulerAngleYXZ(euler_angles.y, euler_angles.x, euler_angles.z));
    }

    // ---------------------- Geometric Transformation ----------------------

    constexpr void Translate(Vector3<T> delta) noexcept {
        m_center += delta;
    }

    constexpr void Rotate(T pitch, T yaw, T roll) noexcept {
        const Matrix3<T> rot = Matrix3<T>(glm::eulerAngleYXZ(yaw, pitch, roll));
        m_axes *= rot;
    }

    constexpr void Rotate(Vector3<T> euler_angles) noexcept {
        Rotate(euler_angles.x, euler_angles.y, euler_angles.z);
    }

    constexpr void Expand(Vector3<T> offset) noexcept {
        SetExtents(m_extents + offset);
    }

    constexpr void Scale(T scale) noexcept {
        m_extents *= std::abs(scale);
    }

    // ---------------------- Intersections ----------------------

    [[nodiscard]] constexpr bool Contains(Vector3<T> point) const noexcept {
        const Vector3<T> local_point = TransformPointToLocal(point);
        return (
            local_point.x >= -m_extents.x && local_point.x <= m_extents.x &&
            local_point.y >= -m_extents.y && local_point.y <= m_extents.y &&
            local_point.z >= -m_extents.z && local_point.z <= m_extents.z  //
        );
    }

    // ---------------------- Helpers ----------------------

    [[nodiscard]] constexpr Vector3<T> TransformPointToWorld(Vector3<T> local_point) const noexcept {
        return (m_axes * local_point) + m_center;
    }

    [[nodiscard]] constexpr Vector3<T> TransformPointToLocal(Vector3<T> world_point) const noexcept {
        return (world_point - m_center) * m_axes;
    }

    [[nodiscard]] constexpr Vector3<T> TransformDirectionToWorld(Vector3<T> local_dir) const noexcept {
        return m_axes * local_dir;
    }

    [[nodiscard]] constexpr Vector3<T> TransformDirectionToLocal(Vector3<T> world_dir) const noexcept {
        return world_dir * m_axes;
    }

    // ---------------------- Utilities ----------------------

    [[nodiscard]] constexpr Vector3<T> Lerp(Vector3<T> t) const noexcept {
        const Vector3<T> local_point = m_extents * ((t * T {2}) - T {1});
        return TransformPointToWorld(local_point);
    }

    constexpr void GetCorners(std::span<Vector3<T>, 8> corners) const noexcept {
        corners[0] = TransformPointToWorld(-m_extents);
        corners[1] = TransformPointToWorld(Vector3<T>(m_extents.x, -m_extents.y, -m_extents.z));
        corners[2] = TransformPointToWorld(Vector3<T>(m_extents.x, m_extents.y, -m_extents.z));
        corners[3] = TransformPointToWorld(Vector3<T>(-m_extents.x, m_extents.y, -m_extents.z));
        corners[4] = TransformPointToWorld(Vector3<T>(-m_extents.x, -m_extents.y, m_extents.z));
        corners[5] = TransformPointToWorld(Vector3<T>(m_extents.x, -m_extents.y, m_extents.z));
        corners[6] = TransformPointToWorld(Vector3<T>(-m_extents.x, m_extents.y, m_extents.z));
        corners[7] = TransformPointToWorld(m_extents);
    }

    [[nodiscard]] constexpr std::array<Vector3<T>, 8> Corners() const noexcept {
        std::array<Vector3<T>, 8> corners;
        GetCorners(corners);
        return corners;
    }

    [[nodiscard]] constexpr Vector3<T> ClosestPoint(Vector3<T> point) const noexcept {
        const Vector3<T> local_point = TransformPointToLocal(point);
        const Vector3<T> local_closest = glm::clamp(local_point, -m_extents, m_extents);
        return m_center + (m_axes * local_closest);
    }

    [[nodiscard]] constexpr T DistanceSquared(Vector3<T> point) const noexcept {
        const Vector3<T> closest = ClosestPoint(point);
        const Vector3<T> diff = point - closest;
        return (diff.x * diff.x) + (diff.y * diff.y) + (diff.z * diff.z);
    }

    [[nodiscard]] constexpr T Distance(Vector3<T> point) const noexcept {
        return glm::sqrt(DistanceSquared(point));
    }

    [[nodiscard]] constexpr int MaximumExtent() const noexcept {
        if (m_extents.x > m_extents.y && m_extents.x > m_extents.z) {
            return 0;
        } else if (m_extents.y > m_extents.z) {
            return 1;
        } else {
            return 2;
        }
    }

    [[nodiscard]] constexpr AABB<T> ToAABB() const noexcept {
        Matrix3<T> abs_axes;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                abs_axes[i][j] = std::abs(m_axes[i][j]);
            }
        }

        const Vector3<T> aabb_extents = abs_axes * m_extents;
        return AABB<T>(m_center - aabb_extents, m_center + aabb_extents);
    }

    // ---------------------- Operators ----------------------

    [[nodiscard]] constexpr OBB<T> operator+(Vector3<T> offset) const noexcept {
        return OBB<T>(m_center + offset, m_extents, m_axes);
    }

    [[nodiscard]] constexpr OBB<T> operator-(Vector3<T> offset) const noexcept {
        return OBB<T>(m_center - offset, m_extents, m_axes);
    }

    constexpr OBB<T>& operator+=(Vector3<T> offset) noexcept {
        m_center += offset;
        return *this;
    }

    constexpr OBB<T>& operator-=(Vector3<T> offset) noexcept {
        m_center -= offset;
        return *this;
    }

    template <obb_type U>
    [[nodiscard]] constexpr explicit operator OBB<U>() const noexcept {
        return OBB<U>(static_cast<Vector3<U>>(m_center), static_cast<Vector3<U>>(m_extents), static_cast<Matrix3<U>>(m_axes));
    }
};

using AABBf = AABB<float>;
using AABBd = AABB<double>;
using AABBi32 = AABB<int32_t>;
using AABBu32 = AABB<uint32_t>;
using AABBi64 = AABB<int64_t>;
using AABBu64 = AABB<uint64_t>;

using OBBf = OBB<float>;
using OBBd = OBB<double>;

#ifdef MATH_ESSENTIALS_NAMESPACE
}  // namespace NAMESPACE_NAME
#endif

template <typename T>
    requires std::is_arithmetic_v<T>
struct std::formatter<NAMESPACE_NAME::AABB<T>> {
    constexpr auto parse(std::format_parse_context& ctx) {
        return ctx.begin();
    }

    auto format(const NAMESPACE_NAME::AABB<T>& box, std::format_context& ctx) const {
        return std::format_to(ctx.out(), "AABB(min: {}, max: {})", box.Min(), box.Max());
    }
};

template <typename T>
    requires std::is_arithmetic_v<T>
struct std::formatter<NAMESPACE_NAME::OBB<T>> {
    constexpr auto parse(std::format_parse_context& ctx) {
        return ctx.begin();
    }

    auto format(const NAMESPACE_NAME::OBB<T>& box, std::format_context& ctx) const {
        return std::format_to(ctx.out(), "OBB(center: {}, extents: {})", box.Center(), box.Extents());
    }
};