if (NOT TARGET spdlog::spdlog)
    message(STATUS "Fetching spdlog...")

    FetchContent_Declare(spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG        v1.15.2
        GIT_SHALLOW TRUE # git clone --depth=1
        GIT_PROGRESS TRUE
        EXCLUDE_FROM_ALL # Added in CMake 3.28
    )

    set(SPDLOG_BUILD_SHARED  OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
    set(SPDLOG_WCHAR_SUPPORT ON CACHE BOOL "" FORCE)

    # See: https://github.com/gabime/spdlog/blob/b18a234ed6af38638678a3338c0d7ed90210ae6c/CMakeLists.txt#L91
    #      https://github.com/gabime/spdlog/issues/355#issuecomment-273497846
    #set(SPDLOG_FMT_EXTERNAL_HO  ON CACHE BOOL "" FORCE)
    
    # https://github.com/gabime/spdlog/issues/3261#issuecomment-2483715077
    set(SPDLOG_USE_STD_FORMAT  ON CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(spdlog)
endif()