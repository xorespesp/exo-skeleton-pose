if (NOT TARGET apriltag::apriltag)
    message(STATUS "Fetching AprilTag...")

    FetchContent_Declare(
        apriltag
        GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git
        GIT_TAG v3.4.5
        GIT_SHALLOW TRUE # git clone --depth=1
        GIT_PROGRESS TRUE
        EXCLUDE_FROM_ALL # Added in CMake 3.28
    )

    # Ensure options declared via option() honor our cache values (avoids CMP0077 warnings)
    set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)   # static lib, no runtime DLL to bundle
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_PYTHON_WRAPPER OFF CACHE BOOL "" FORCE)
    set(ASAN OFF CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(apriltag)

    # Create an alias target for apriltag if it doesn't already exist
    if (TARGET apriltag AND NOT TARGET apriltag::apriltag)
        add_library(apriltag::apriltag ALIAS apriltag)
    endif()

endif()
