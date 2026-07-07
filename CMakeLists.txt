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
    "src/main.cc"
    "src/cli_options.hh"
    "src/cli_options.cc"

    # camera sensor abstraction
    "src/hw/calibration.hh"
    "src/hw/sensor_frameset.hh"
    "src/hw/sensor_frame.hh"
    "src/hw/sensor_frame_source.hh"
    "src/hw/sensor_frame_observer.hh"
    "src/hw/sensor_frame_provider.hh"
    "src/hw/sensor_frame_provider.cc"

    # camera sensor backend implementation (Orbbec K4A Wrapper, temporary)
    "src/hw/k4a_frameset.hh"
    "src/hw/k4a_frameset.cc"
    "src/hw/k4a_frame_source.hh"
    "src/hw/k4a_frame_source.cc"

    # tag detection / pose
    "src/pose/tag_detector.hh"
    "src/pose/tag_detector.cc"
    "src/pose/pose_estimator.hh"
    "src/pose/pose_estimator.cc"

    # gui
    "src/gui/app_base.hh"
    "src/gui/app_renderer_sdl3.hh"
    "src/gui/app_renderer_sdl3.cc"
    "src/gui/frame_texture.hh"
    "src/gui/frame_texture.cc"
    "src/gui/debug_gui_app.hh"
    "src/gui/debug_gui_app.cc"

    # server (websocket + flatbuffers protocol)
    "src/net/pose_server.hh"
    "src/net/pose_server.cc"
)

target_include_directories(${TARGET_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src")

if (WIN32)
    target_compile_definitions(${TARGET_NAME} PRIVATE 
        NOMINMAX 
        WIN32_LEAN_AND_MEAN)
endif()

include(FetchCLI11)
include(FetchEigen)
include(FetchSpdlog)
include(FetchAprilTag)
include(FetchOrbbecK4A)
include(FetchSDL3)
include(FetchFlatBuffers)
include(FetchUWebSockets)

add_subdirectory(deps)

find_package(OpenCV REQUIRED)

# Generate protocol bindings from the FlatBuffers schema.
set(FBS_SCHEMA "${CMAKE_CURRENT_SOURCE_DIR}/proto/pose_protocol.fbs")

# C++ header -> build tree, compiled into the server.
set(FBS_GEN_DIR "${CMAKE_BINARY_DIR}/generated")
set(FBS_GEN_HEADER "${FBS_GEN_DIR}/pose_protocol_generated.h")
add_custom_command(
    OUTPUT ${FBS_GEN_HEADER}
    COMMAND flatc --cpp -o ${FBS_GEN_DIR} ${FBS_SCHEMA}
    DEPENDS ${FBS_SCHEMA} flatc
    COMMENT "Generating FlatBuffers C++ header from pose_protocol.fbs"
    VERBATIM
)
target_sources(${TARGET_NAME} PRIVATE ${FBS_GEN_HEADER})
target_include_directories(${TARGET_NAME} PRIVATE ${FBS_GEN_DIR})

# TS bindings -> webview build tree
set(FBS_TS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/webview/src/protocol")
set(FBS_TS_BARREL "${FBS_TS_DIR}/exo/proto.ts")
add_custom_command(
    OUTPUT ${FBS_TS_BARREL}
    COMMAND flatc --ts -o ${FBS_TS_DIR} ${FBS_SCHEMA}
    DEPENDS ${FBS_SCHEMA} flatc
    COMMENT "Generating FlatBuffers TS bindings for the webview"
    VERBATIM
)
add_custom_target(webview_protocol DEPENDS ${FBS_TS_BARREL})
add_dependencies(${TARGET_NAME} webview_protocol)

target_link_libraries(${TARGET_NAME} PRIVATE
    Eigen3::Eigen
    spdlog::spdlog_header_only
    apriltag::apriltag
    CLI11::CLI11
    k4a::k4a
    k4a::k4arecord
    ${OpenCV_LIBS}
    imgui::imgui
    implot::implot
    implot3d::implot3d
    imfilebrowser::imfilebrowser
    uwebsockets::uwebsockets
    flatbuffers::flatbuffers
)

setup_orbbec_k4a_binaries(${TARGET_NAME} INSTALL_PATH ${CMAKE_INSTALL_BINDIR})

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
