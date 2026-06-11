cmake_minimum_required(VERSION 3.14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(cpp-math-essentials)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_TESTS "Build tests" OFF)
option(MATH_ESSENTIALS_NAMESPACE "Namespace for math_essentials" "mth")

add_library(math_essentials INTERFACE)

target_include_directories(math_essentials INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(math_essentials INTERFACE MATH_ESSENTIALS_NAMESPACE=${MATH_ESSENTIALS_NAMESPACE})

if(BUILD_TESTS)
    message(STATUS "Building tests")

    include(FetchContent)

    # Fetch GLM just for testing
    FetchContent_Declare(
        glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG        1.0.1
    )
    FetchContent_MakeAvailable(glm)

    # Link math_essentials with GLM
    target_link_libraries(math_essentials INTERFACE glm::glm)

    # Enable testing
    enable_testing()
    add_subdirectory(tests)
else()
    message(STATUS "Not building tests")
endif()