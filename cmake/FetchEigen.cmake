include_guard(GLOBAL)
include(FetchContent)

set(_EIGEN_VERSION 3.4.0)

set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
set(EIGEN_TEST_NOQT ON CACHE BOOL "" FORCE)

FetchContent_Declare(
    Eigen3
    URL https://gitlab.com/libeigen/eigen/-/archive/${_EIGEN_VERSION}/eigen-${_EIGEN_VERSION}.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    # URL_HASH SHA256=<add hash for reproducible builds>
)
FetchContent_MakeAvailable(Eigen3)

# Eigen 3.4.0's in-tree CMake creates an INTERFACE target named `eigen`.
# Alias it as `Eigen3::Eigen` for parity with find_package(Eigen3).
if(NOT TARGET Eigen3::Eigen AND TARGET eigen)
    add_library(Eigen3::Eigen ALIAS eigen)
endif()
