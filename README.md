# cpp-math-essentials

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)

A lightweight, header-only C++20 math library built on top of [GLM](https://github.com/g-truc/glm). It provides expressive type aliases, bounding-box primitives (AABB & OBB), and ray-intersection utilities — ideal for game engines, physics simulations, and spatial queries.

## Features

- **Vectors** — `Vector2/3/4` type aliases (`int`, `float`, `double`) with concept-constrained templates and `std::format` support.
- **Matrices** — `Matrix2/3/4` type aliases with the same concept constraints and formatting.
- **AABB** — Axis-Aligned Bounding Box with containment, intersection, merge, enclose, corners, lerp, closest-point, distance, scale/expand, operators, and type casting.
- **OBB** — Oriented Bounding Box with Euler-angle rotation, local/world transforms, corners, closest-point, distance, and AABB conversion.
- **Ray** — Ray casting against AABB and OBB with hit/miss, distance, position, surface normal, and inside-flag.
- **Configurable namespace** — Wrap everything in a custom namespace (default: `mth`) via a CMake option.
- **`std::format` integration** — All vector, matrix, and bounding-box types are directly formattable.

## Integration

### CMake FetchContent (recommended)

```cmake
include(FetchContent)

FetchContent_Declare(
    cpp-math-essentials
    GIT_REPOSITORY https://github.com/pablobh2147/cpp-math-essentials.git
    GIT_TAG main
)
FetchContent_MakeAvailable(cpp-math-essentials)

target_link_libraries(your_target PRIVATE math_essentials)
```

### As a subdirectory

```cmake
add_subdirectory(path/to/cpp-math-essentials)
target_link_libraries(your_target PRIVATE math_essentials)
```

> **Note:** GLM is required as a dependency of your project when using this library. You can provide it yourself or let your build system fetch it.

### Namespace configuration

By default the library wraps all symbols in the `mth` namespace. Override it with:

```cmake
set(MATH_ESSENTIALS_NAMESPACE my_namespace CACHE STRING "" FORCE)
```

## Usage

```cpp
#include "Vector.hpp"
#include "BoundingBox.hpp"
#include "Ray.hpp"

#include <format>
#include <iostream>

int main() {
    using namespace mth;

    // Vectors
    Vector3f position(1.0f, 2.0f, 3.0f);
    std::cout << std::format("Position: {}\n", position);

    // AABB
    AABBf box(Vector3f(0.0f), Vector3f(10.0f));
    bool inside = box.Contains(position);

    // Ray intersection
    Rayf ray(Vector3f(-5.0f, 1.0f, 1.0f), Vector3f(1.0f, 0.0f, 0.0f));
    RayIntersectionf hit = ray.Intersect(box);
    if (hit) {
        std::cout << std::format("Hit at {} (distance: {})\n", hit.position, hit.distance);
    }

    // OBB
    OBBf obb(Vector3f(0.0f), Vector3f(1.0f), Vector3f(0.0f, glm::radians(45.0f), 0.0f));
    RayIntersectionf obb_hit = ray.Intersect(obb);

    std::cout << std::format("Hit info: {}\n", obb_hit);

    return 0;
}
```

## Building & Testing

```bash
cmake -B build -DBUILD_TESTS=ON
cmake --build build
ctest --test-dir build
```

Tests use [Google Test](https://github.com/google/googletest) (v1.14.0) and GLM (v1.0.1), both fetched automatically via CMake FetchContent.

## Requirements

- C++20-compatible compiler (GCC ≥ 12, Clang ≥ 15, MSVC ≥ 19.29)
- CMake ≥ 3.14
- GLM (fetched automatically for tests, must be provided by the consumer project otherwise)

## License

This project is licensed under the [MIT License](LICENSE).

Copyright (c) 2026 Pablo Bermejo Hernández
