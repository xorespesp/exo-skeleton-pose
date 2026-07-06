add_library(implot3d STATIC)

target_sources(implot3d
    PRIVATE
        "implot3d.cpp"
        "implot3d_items.cpp"
        "implot3d_meshes.cpp"
        "implot3d_demo.cpp"
)

add_library(implot3d::implot3d ALIAS implot3d)

target_include_directories(implot3d
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(implot3d PUBLIC imgui::imgui)
