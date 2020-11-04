#  Copyright (c) Microsoft Corporation. All rights reserved.
#  Licensed under the MIT License.

cmake_minimum_required (VERSION 3.8)

project (Camera-Rework C)

include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/printf ${CMAKE_SOURCE_DIR}/lib ${CMAKE_SOURCE_DIR}/lib/mt3620)

# macro
add_compile_definitions(AzureSphere_CM4)

# Create executable
ADD_EXECUTABLE(${PROJECT_NAME} main.c printf/printf.c lib/VectorTable.c lib/SPIMaster.c lib/I2CMaster.c lib/GPIO.c lib/UART.c arducam_driver/ArduCAM.c arducam_driver/ll.c arducam_driver/ov2640_regs.c "delay.c" "delay.h")
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC ${AZURE_SPHERE_API_SET_DIR}/usr/include/azureiot)
TARGET_COMPILE_DEFINITIONS(${PROJECT_NAME} PUBLIC AZURE_IOT_HUB_CONFIGURED)
target_link_libraries (${PROJECT_NAME} applibs curl pthread gcc_s c azureiot)
set_target_properties (${PROJECT_NAME} PROPERTIES LINK_DEPENDS ${CMAKE_SOURCE_DIR}/linker.ld)

azsphere_configure_tools(TOOLS_REVISION "20.04")
azsphere_configure_api(TARGET_API_SET "6")

#azsphere_target_add_image_package(${PROJECT_NAME})
include ("${AZURE_SPHERE_MAKE_IMAGE_FILE}")