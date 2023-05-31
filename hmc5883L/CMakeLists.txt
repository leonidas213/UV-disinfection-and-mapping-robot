add_library(hmc5883l
hmc5883l.cpp

)

target_link_libraries(hmc5883l
pico_stdlib
hardware_i2c
i2csoft
        )
target_include_directories (hmc5883l PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})