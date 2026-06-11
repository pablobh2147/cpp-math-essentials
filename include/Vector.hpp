#pragma once

#include <glm/glm.hpp>

#include <concepts>
#include <format>

#ifdef MATH_ESSENTIALS_NAMESPACE
#define NAMESPACE_NAME MATH_ESSENTIALS_NAMESPACE
#else
#define NAMESPACE_NAME
#endif

#ifdef MATH_ESSENTIALS_NAMESPACE
namespace NAMESPACE_NAME {
#endif

template <int lenght>
concept vector_lenght = (lenght >= 2);

template <typename T>
concept vector_type = std::floating_point<T> || std::integral<T>;

template <int lenght, typename T>
    requires vector_lenght<lenght> && vector_type<T>
using Vector = glm::vec<lenght, T>;

template <typename T>
using Vector2 = Vector<2, T>;
using Vector2i = Vector2<int>;
using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;

template <typename T>
using Vector3 = Vector<3, T>;
using Vector3i = Vector3<int>;
using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;

template <typename T>
using Vector4 = Vector<4, T>;
using Vector4i = Vector4<int>;
using Vector4f = Vector4<float>;
using Vector4d = Vector4<double>;

#ifdef MATH_ESSENTIALS_NAMESPACE
}  // namespace NAMESPACE_NAME
#endif

template <int length, typename T>
    requires NAMESPACE_NAME::vector_lenght<length> && NAMESPACE_NAME::vector_type<T>
struct std::formatter<NAMESPACE_NAME::Vector<length, T>> {
    constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

    auto format(const NAMESPACE_NAME::Vector<length, T>& vec,
                std::format_context& ctx) const {
        auto out = std::format_to(ctx.out(), "(");
        for (int i = 0; i < length; ++i) {
            if (i > 0) {
                out = std::format_to(out, ", ");
            }
            out = std::format_to(out, "{}", vec[i]);
        }
        return std::format_to(out, ")");
    }
};
