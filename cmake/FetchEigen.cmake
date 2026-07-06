if (NOT TARGET Eigen3::Eigen)
    message(STATUS "Fetching Eigen...")

    FetchContent_Declare(
        Eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE # git clone --depth=1
        GIT_PROGRESS TRUE
    )

    # Optionally disable parts of Eigen build if not needed
    set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_BLAS OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_LAPACK OFF CACHE BOOL "" FORCE)
    set(EIGEN_USE_LAPACKE OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_BTL OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(Eigen)
endif()