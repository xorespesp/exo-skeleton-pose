add_library(imfilebrowser INTERFACE)

add_library(imfilebrowser::imfilebrowser ALIAS imfilebrowser)

target_include_directories(imfilebrowser
    INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(imfilebrowser INTERFACE imgui::imgui)
