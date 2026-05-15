cmake_minimum_required (VERSION 3.24)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

set(PROJECT_NAME exo-skeleton-pose-ebimu)
project(${PROJECT_NAME} VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FetchBoost)
include(FetchEigen)
include(FetchSpdlog)
include(FetchMagicEnum)

add_executable(${PROJECT_NAME}
  "src/main.cc"
  "src/core/ebrcv24g_device_session.hh"
  "src/core/ebrcv24g_proto.hh"
  "src/core/io_context_pool.hh"
  "src/core/math3d.hh"
  "src/core/pose_estimator.hh"
  "src/utils/bit_cast.hh"
  "src/utils/codecvt.hh"
  "src/utils/c_format.hh"
  "src/utils/debug_utils.hh"
  "src/utils/logger.hh"
  "src/utils/mutex.hh"
  "src/utils/time_utils.hh"
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Boost::asio
    Boost::pool
    Boost::circular_buffer
    Boost::program_options
    Eigen3::Eigen
    spdlog::spdlog_header_only
    magic_enum::magic_enum
)

########################################################################################
# Install
########################################################################################

include(GNUInstallDirs)

# Default install prefix → <source>/install when caller didn't set one explicitly
# (CMakePresets.json's installDir takes precedence over this fallback)
if(DEFINED CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX
        "${CMAKE_CURRENT_SOURCE_DIR}/install"
        CACHE PATH "Install prefix" FORCE
    )
endif()
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

# Install the executable
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install PDB next to the exe on MSVC for Debug / RelWithDebInfo builds
if(MSVC)
    install(FILES $<TARGET_PDB_FILE:${PROJECT_NAME}>
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        CONFIGURATIONS Debug RelWithDebInfo
        OPTIONAL
    )
endif()

# Bundle runtime DLLs that the exe depends on (no-op for fully-static deps).
# Requires CMake 3.21+; this project already needs 3.24.
install(TARGETS ${PROJECT_NAME}
    RUNTIME_DEPENDENCY_SET ${PROJECT_NAME}_runtime_deps
)
install(RUNTIME_DEPENDENCY_SET ${PROJECT_NAME}_runtime_deps
    PRE_EXCLUDE_REGEXES
        [[api-ms-]] [[ext-ms-]]            # Windows API set DLLs
    POST_EXCLUDE_REGEXES
        [[.*[\\/]system32[\\/].*\.dll]]    # Windows system DLLs
        [[^/(lib|usr/lib|usr/lib64)/.*]]   # Linux system libs
    DIRECTORIES $<TARGET_FILE_DIR:${PROJECT_NAME}>
)

# Project docs / metadata
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
    "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE.txt"
    DESTINATION ${CMAKE_INSTALL_DOCDIR}
)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/docs/"
    DESTINATION ${CMAKE_INSTALL_DOCDIR}
)
