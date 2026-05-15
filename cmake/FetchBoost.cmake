include_guard(GLOBAL)
include(FetchContent)

set(_BOOST_VERSION 1.88.0)

set(BOOST_INCLUDE_LIBRARIES
    asio
    pool
    circular_buffer
    program_options
    CACHE STRING "" FORCE)
set(BOOST_ENABLE_CMAKE ON CACHE BOOL "" FORCE)
set(BOOST_SKIP_INSTALL_RULES ON CACHE BOOL "" FORCE)

FetchContent_Declare(
    Boost
    URL https://github.com/boostorg/boost/releases/download/boost-${_BOOST_VERSION}/boost-${_BOOST_VERSION}-cmake.tar.xz
    DOWNLOAD_EXTRACT_TIMESTAMP ON
    # URL_HASH SHA256=<add hash for reproducible builds>
)
FetchContent_MakeAvailable(Boost)
