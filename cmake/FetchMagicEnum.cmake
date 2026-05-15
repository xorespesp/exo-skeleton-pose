include_guard(GLOBAL)
include(FetchContent)

set(_MAGIC_ENUM_VERSION 0.9.7)

set(MAGIC_ENUM_OPT_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MAGIC_ENUM_OPT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MAGIC_ENUM_OPT_INSTALL OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    magic_enum
    URL https://github.com/Neargye/magic_enum/archive/refs/tags/v${_MAGIC_ENUM_VERSION}.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    # URL_HASH SHA256=<add hash for reproducible builds>
)
FetchContent_MakeAvailable(magic_enum)
