if (NOT TARGET CLI11::CLI11)
    message(STATUS "Fetching CLI11...")

    FetchContent_Declare(
        CLI11
        GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
        GIT_TAG v2.6.2
        GIT_SHALLOW TRUE # git clone --depth=1
        GIT_PROGRESS TRUE
        EXCLUDE_FROM_ALL # Added in CMake 3.28
    )

    # Ensure FetchContent_MakeAvailable uses the modern behavior (avoids some warnings)
    set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

    set(CLI11_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(CLI11_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(CLI11_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(CLI11_INSTALL OFF CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(CLI11)
endif()
