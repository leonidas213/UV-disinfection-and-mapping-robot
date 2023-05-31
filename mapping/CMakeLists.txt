add_library(mapping
mapping.cpp

)
target_link_libraries(mapping
pico_stdlib
math3d
        )
target_include_directories (mapping PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})