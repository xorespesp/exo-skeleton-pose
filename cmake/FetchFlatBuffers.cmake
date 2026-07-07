if (NOT TARGET flatbuffers::flatbuffers)
    message(STATUS "Fetching FlatBuffers...")

    FetchContent_Declare(
        flatbuffers
        GIT_REPOSITORY https://github.com/google/flatbuffers.git
        GIT_TAG v25.12.19
        GIT_SHALLOW TRUE # git clone --depth=1
        GIT_PROGRESS TRUE
        EXCLUDE_FROM_ALL # Added in CMake 3.28
    )

    # Ensure options declared via option() honor our cache values (avoids CMP0077 warnings)
    set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

    set(FLATBUFFERS_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
    set(FLATBUFFERS_INSTALL       OFF CACHE BOOL "" FORCE)
    set(FLATBUFFERS_BUILD_FLATLIB ON  CACHE BOOL "" FORCE) # flatbuffers::flatbuffers (runtime)
    set(FLATBUFFERS_BUILD_FLATC   ON  CACHE BOOL "" FORCE) # flatc (schema compiler)

    FetchContent_MakeAvailable(flatbuffers)

    # Upstream exports the runtime lib as the un-namespaced target `flatbuffers`.
    if (TARGET flatbuffers AND NOT TARGET flatbuffers::flatbuffers)
        add_library(flatbuffers::flatbuffers ALIAS flatbuffers)
    endif()
endif()
