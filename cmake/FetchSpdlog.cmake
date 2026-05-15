include_guard(GLOBAL)
include(FetchContent)

set(_SPDLOG_VERSION 1.15.3)

set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_BENCH OFF CACHE BOOL "" FORCE)
set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    spdlog
    URL https://github.com/gabime/spdlog/archive/refs/tags/v${_SPDLOG_VERSION}.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    # URL_HASH SHA256=<add hash for reproducible builds>
)
FetchContent_MakeAvailable(spdlog)
