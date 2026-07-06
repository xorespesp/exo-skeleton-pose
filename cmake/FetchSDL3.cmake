include_guard(GLOBAL)

if (NOT TARGET SDL3::SDL3-static)
    message(STATUS "Fetching SDL3...")

    FetchContent_Declare(
        SDL3
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG        release-3.4.2
        GIT_SHALLOW    TRUE # git clone --depth=1
        GIT_PROGRESS   TRUE
        OVERRIDE_FIND_PACKAGE
        EXCLUDE_FROM_ALL # Added in CMake 3.28
    )

    # Disable Shared Libs
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    set(SDL_SHARED OFF CACHE BOOL "" FORCE)
    set(SDL_STATIC ON CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(SDL3)
endif()
