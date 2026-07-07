include_guard(GLOBAL)

# uWebSockets ships no CMake and is header-only C++, but its C dependency uSockets
# must be compiled. On Windows the only working event-loop backend is libuv, so we
# fetch libuv, build uSockets against it (no SSL, no zlib -> ws:// only), and expose
# the whole thing as `uwebsockets::uwebsockets`.

set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# --- libuv (event loop backend) --------------------------------------------------
if (NOT TARGET uv_a)
    message(STATUS "Fetching libuv...")
    FetchContent_Declare(
        libuv
        GIT_REPOSITORY https://github.com/libuv/libuv.git
        GIT_TAG v1.52.1
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
        EXCLUDE_FROM_ALL
    )
    set(LIBUV_BUILD_SHARED OFF CACHE BOOL "" FORCE) # static uv_a only
    set(LIBUV_BUILD_TESTS  OFF CACHE BOOL "" FORCE)
    set(LIBUV_BUILD_BENCH  OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING      OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(libuv)
endif()

# --- uWebSockets + uSockets ------------------------------------------------------
if (NOT TARGET uWebSockets)
    message(STATUS "Fetching uWebSockets...")
    FetchContent_Declare(
        uwebsockets
        GIT_REPOSITORY https://github.com/uNetworking/uWebSockets.git
        GIT_TAG v20.78.0
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
        GIT_SUBMODULES "uSockets" # pull the pinned uSockets, nothing else
        GIT_SUBMODULES_RECURSE OFF
        EXCLUDE_FROM_ALL
    )
    # NOTE: No CMakeLists at the root -> MakeAvailable just populates the sources.
    FetchContent_MakeAvailable(uwebsockets)

    set(_usockets_dir "${uwebsockets_SOURCE_DIR}/uSockets/src")

    # Core C sources + the libuv eventing backend (skip epoll_kqueue/gcd/asio and
    # the SSL crypto/ sources, which we disable via LIBUS_NO_SSL).
    file(GLOB _usockets_src "${_usockets_dir}/*.c")
    list(APPEND _usockets_src "${_usockets_dir}/eventing/libuv.c")

    add_library(uSockets STATIC ${_usockets_src})
    target_include_directories(uSockets PUBLIC "${_usockets_dir}")
    target_compile_definitions(uSockets PUBLIC LIBUS_NO_SSL LIBUS_USE_LIBUV)
    target_link_libraries(uSockets PUBLIC uv_a)
    if (WIN32)
        target_link_libraries(uSockets PUBLIC ws2_32)
        target_compile_definitions(uSockets PRIVATE _CRT_SECURE_NO_WARNINGS)
    endif()

    # uWebSockets itself is header-only (src/ has the C++ headers).
    add_library(uWebSockets INTERFACE)
    target_include_directories(uWebSockets INTERFACE "${uwebsockets_SOURCE_DIR}/src")
    target_link_libraries(uWebSockets INTERFACE uSockets)
    target_compile_definitions(uWebSockets INTERFACE UWS_NO_ZLIB) # no permessage-deflate
    add_library(uwebsockets::uwebsockets ALIAS uWebSockets)
endif()
