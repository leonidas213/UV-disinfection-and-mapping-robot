add_library(Stepper
        driver.cpp
        
)

target_link_libraries(Stepper
pico_stdlib

        )
target_include_directories (Stepper PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})