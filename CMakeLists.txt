cmake_minimum_required(VERSION 3.16)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/stm32-cmake/cmake/stm32_gcc.cmake)

project(stm32h7 CXX C ASM)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  
 
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

# set(CMSIS_INCLUDE_DIRS [ 
#     ../libs/stm32-cmsis-h7-src/Include,
#     ../libs/stm32-cmsis-src/Include,
#     ../libs/stm32-cmsis-src/Core/Include
# ])
# set(CMSIS_SOURCES ../libs/stm32-cmsis-h7-src/Source)

stm32_fetch_cmsis(H7)
stm32_fetch_hal(H7)  

find_package(CMSIS COMPONENTS STM32H7 STM32H743ZI REQUIRED)
find_package(HAL COMPONENTS STM32H7 RCC GPIO CORTEX TIM DMA PWR REQUIRED)

set(PROJECT_SOURCES
    src/main.cpp
    # config/stm32f4xx_hal_conf.h
)
 
add_executable(stm32h7 ${PROJECT_SOURCES})
 
target_include_directories(stm32h7 
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/config
)

target_link_libraries(stm32h7
    CMSIS::STM32::H7::M7
    CMSIS::STM32::H743xx::M7
    HAL::STM32::H7::M7::RCCEx 
    HAL::STM32::H7::M7::GPIO
    HAL::STM32::H7::M7::CORTEX
    HAL::STM32::H7::M7::TIMEx
    HAL::STM32::H7::M7::DMAEx
    HAL::STM32::H7::M7::PWREx
    STM32::NoSys 
)  

stm32_add_linker_script(stm32h7 PRIVATE H743.ld) 
stm32_print_size_of_target(stm32h7)
