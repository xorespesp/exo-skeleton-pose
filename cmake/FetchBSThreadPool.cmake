if (NOT TARGET BS::thread_pool)
    message(STATUS "Fetching BS::thread_pool...")

    # Single header, MIT. The repository carries no CMake project, so the target is declared
    # here: an interface library that only contributes the include path.
    FetchContent_Declare(
        bs_thread_pool
        GIT_REPOSITORY https://github.com/bshoshany/thread-pool.git
        GIT_TAG v5.1.0
        GIT_SHALLOW TRUE # git clone --depth=1
        GIT_PROGRESS TRUE
    )

    FetchContent_MakeAvailable(bs_thread_pool)

    add_library(bs_thread_pool INTERFACE)
    add_library(BS::thread_pool ALIAS bs_thread_pool)
    target_include_directories(bs_thread_pool INTERFACE "${bs_thread_pool_SOURCE_DIR}/include")
endif()
