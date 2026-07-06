cmake_minimum_required (VERSION 3.28 FATAL_ERROR)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

set(TARGET_NAME exo-skeleton-pose)
project(${TARGET_NAME} VERSION 0.0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(FetchContent)
include(GNUInstallDirs)

add_executable(${TARGET_NAME})
target_sources(${TARGET_NAME} PRIVATE
    "src/main.cc")
        
include(FetchCLI11)
include(FetchBoost)
include(FetchEigen)
include(FetchSpdlog)

target_link_libraries(${TARGET_NAME} PRIVATE
    Boost::asio
    Boost::pool
    Boost::circular_buffer
    Eigen3::Eigen
    spdlog::spdlog_header_only
)

########################################################################################
# Install
########################################################################################

if(DEFINED CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX
        "${CMAKE_CURRENT_SOURCE_DIR}/install"
        CACHE PATH "Install prefix" FORCE
    )
endif()
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

# Install the executable
install(TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install PDB next to the exe on MSVC for Debug / RelWithDebInfo builds
if(MSVC)
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}>
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        CONFIGURATIONS Debug RelWithDebInfo
        OPTIONAL
    )
endif()

# Bundle runtime DLLs that the exe depends on (no-op for fully-static deps).
# (Requires CMake 3.21+)
install(TARGETS ${TARGET_NAME}
    RUNTIME_DEPENDENCY_SET ${TARGET_NAME}_runtime_deps
)
install(RUNTIME_DEPENDENCY_SET ${TARGET_NAME}_runtime_deps
    PRE_EXCLUDE_REGEXES
        [[api-ms-]] [[ext-ms-]]            # Windows API set DLLs
    POST_EXCLUDE_REGEXES
        [[.*[\\/]system32[\\/].*\.dll]]    # Windows system DLLs
        [[^/(lib|usr/lib|usr/lib64)/.*]]   # Linux system libs
    DIRECTORIES $<TARGET_FILE_DIR:${TARGET_NAME}>
)

# Project docs / metadata
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
    DESTINATION ${CMAKE_INSTALL_DOCDIR}
)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/docs/"
    DESTINATION ${CMAKE_INSTALL_DOCDIR}
)
