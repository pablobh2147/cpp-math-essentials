# ----------- Fetch Google Test -----------

include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

# ----------- Include GoogleTest Module -----------

include(GoogleTest)

# ----------- Automatic Test Discovery -----------

# Find all subdirectories containing test files
file(GLOB TEST_DIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */)

# Find all test_*.cpp files in this directory
file(GLOB TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)

if(TEST_SOURCES)
    set(TEST_TARGET tests)

    # Create test executable
    add_executable(${TEST_TARGET} ${TEST_SOURCES})

    target_link_libraries(${TEST_TARGET} PRIVATE
        GTest::gtest_main
        math_essentials
    )

    gtest_discover_tests(${TEST_TARGET})

    message(STATUS "Discovered tests: (${TEST_SOURCES})")
endif()
