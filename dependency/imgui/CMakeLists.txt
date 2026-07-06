add_library(imgui STATIC)

target_sources(imgui
    PRIVATE
        "imgui.cpp"
        "imgui_draw.cpp"
        "imgui_tables.cpp"
        "imgui_widgets.cpp"
        "imgui_demo.cpp"
        "imgui_impl_sdlrenderer3.cpp"
        "imgui_impl_sdl3.cpp"
)

add_library(imgui::imgui ALIAS imgui)

target_include_directories(imgui
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

# imgui_impl_sdlrenderer3.cpp uses SDL3 headers
if(TARGET SDL3::SDL3-static)
    target_link_libraries(imgui PUBLIC SDL3::SDL3-static)
elseif(TARGET SDL3::SDL3)
    target_link_libraries(imgui PUBLIC SDL3::SDL3)
endif()
