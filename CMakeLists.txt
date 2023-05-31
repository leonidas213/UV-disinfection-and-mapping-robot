# cmake version
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
#set(PICO_BOARD pico_w)
# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(Roomba C CXX ASM)

# initialize the sdk
pico_sdk_init()

#######

add_executable(Roomba main2.cpp)
# name anything you want

# must match with executable name and source file names
pico_enable_stdio_uart(Roomba 1)
pico_enable_stdio_usb(Roomba 1)
add_subdirectory(StepperDriver)
add_subdirectory(MPU6050)
add_subdirectory(hmc5883L)
add_subdirectory(mapping)
#add_subdirectory(hc-04)
# must match with executable name
target_link_libraries(Roomba pico_stdlib 
hardware_pwm
hardware_i2c
hardware_pio
Stepper
mpu6050
pico_bootsel_via_double_reset
pico_multicore
pico_time
hmc5883l
pico_bootrom
mapping
)

# must match with executable name
pico_add_extra_outputs(Roomba)
