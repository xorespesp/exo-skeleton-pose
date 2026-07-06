add_library(implot STATIC)

target_sources(implot
    PRIVATE
        "implot.cpp"
        "implot_items.cpp"
        "implot_demo.cpp"
)

add_library(implot::implot ALIAS implot)

target_include_directories(implot
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(implot PUBLIC imgui::imgui)
