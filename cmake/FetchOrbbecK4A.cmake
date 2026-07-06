include_guard(GLOBAL)

if (NOT WIN32)
    message(FATAL_ERROR "FetchOrbbecK4A.cmake module currently supports Windows platform only.")
endif()

# Fetch the prebuilt Orbbec K4A Wrapper (Azure Kinect compatible) release zip.
# Ref: https://github.com/orbbec/OrbbecSDK-K4A-Wrapper/releases
FetchContent_Declare(
    orbbec_k4a_sdk_bin
    URL "https://github.com/orbbec/OrbbecSDK-K4A-Wrapper/releases/download/v2.0.12/OrbbecSDK_K4A_Wrapper_v2.0.12_windows_202605090937.zip"
)

# Download & extract
FetchContent_MakeAvailable(orbbec_k4a_sdk_bin)

FetchContent_GetProperties(orbbec_k4a_sdk_bin SOURCE_DIR orbbec_k4a_sdk_root_dir)
message(STATUS "orbbec_k4a_sdk_root_dir: ${orbbec_k4a_sdk_root_dir}")

# So find_package can locate the config under lib/cmake/{k4a,k4arecord}.
list(APPEND CMAKE_PREFIX_PATH "${orbbec_k4a_sdk_root_dir}")

# HOTFIX: k4a::k4arecord depends on OrbbecSDK::OrbbecSDK, but the package omits
# that target. Define it manually; OrbbecSDK.lib is missing too, so point the
# import lib at k4a.lib to satisfy the linker.
if (NOT TARGET OrbbecSDK::OrbbecSDK)
    message(WARNING "Applying hotfix for missing OrbbecSDK::OrbbecSDK target...")
    add_library(OrbbecSDK::OrbbecSDK SHARED IMPORTED)

    set_target_properties(OrbbecSDK::OrbbecSDK PROPERTIES
        IMPORTED_LOCATION "${orbbec_k4a_sdk_root_dir}/bin/OrbbecSDK.dll" # DLL file path
        IMPORTED_IMPLIB   "${orbbec_k4a_sdk_root_dir}/lib/k4a.lib" # Lib file path
    )
    message(WARNING "Applied hotfix: defined missing target OrbbecSDK::OrbbecSDK manually.")
endif()

# Creates the k4a::k4a and k4a::k4arecord targets.
find_package(k4a REQUIRED)
find_package(k4arecord REQUIRED)


# setup_orbbec_k4a_binaries(<target_name> [INSTALL_PATH <path>])
#
# Places the Orbbec K4A runtime binaries next to the target executable at build
# and install time.
#
# Arguments:
#   target_name  - CMake target to copy the binaries alongside (required).
#   INSTALL_PATH - install destination for the binaries (optional; no install
#                  rule is generated when omitted).
#
# NOTE:
#   - Must be called after the Orbbec K4A SDK is configured.
#   - Windows only.
#   - The extensions/ directory holds per-device adapter/filter plugins; without
#     it device detection may fail at runtime.
function(setup_orbbec_k4a_binaries TARGET_NAME)
    set(options "")
    set(oneValueArgs INSTALL_PATH)
    set(multiValueArgs "")
    cmake_parse_arguments(arg "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    file(GLOB orbbec_k4a_sdk_dll_files "${orbbec_k4a_sdk_root_dir}/bin/*.dll")
    if (NOT orbbec_k4a_sdk_dll_files)
        message(WARNING "No DLL files found in ${orbbec_k4a_sdk_root_dir}/bin/")
        return()
    endif()
    message(STATUS "Found Orbbec K4A DLL files: ${orbbec_k4a_sdk_dll_files}")

    # Copy runtime DLLs next to the executable.
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${orbbec_k4a_sdk_dll_files}
            "$<TARGET_FILE_DIR:${TARGET_NAME}>"
        COMMENT "Copying Orbbec K4A DLLs to build output directory..."
    )

    # Copy the extensions/ plugins too.
    if (EXISTS "${orbbec_k4a_sdk_root_dir}/bin/extensions")
        add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
                "${orbbec_k4a_sdk_root_dir}/bin/extensions"
                "$<TARGET_FILE_DIR:${TARGET_NAME}>/extensions"
            COMMENT "Copying Orbbec K4A extensions directory to build output directory..."
        )
    endif()

    # Install rules for the binaries (only if INSTALL_PATH is specified).
    if (arg_INSTALL_PATH)
        install(FILES
            ${orbbec_k4a_sdk_dll_files}
            DESTINATION "${arg_INSTALL_PATH}"
            COMPONENT Runtime
        )

        install(DIRECTORY
            "${orbbec_k4a_sdk_root_dir}/bin/extensions"
            DESTINATION "${arg_INSTALL_PATH}"
            COMPONENT Runtime
            OPTIONAL
        )

        message(STATUS "Orbbec K4A binaries will be installed to: ${arg_INSTALL_PATH}")
    endif()
endfunction()
