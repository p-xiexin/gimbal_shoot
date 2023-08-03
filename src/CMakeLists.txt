cmake_minimum_required(VERSION 3.0)
project(Gimbal_Shoot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


set("${PROJECT_SOURCE_DIR}/include"
                        "${PROJECT_SOURCE_DIR}/src")
include_directories(${COMMON_INCLUDE_DIR})

# find_package(PkgConfig)
# pkg_search_module(GLIB REQUIRED glib-2.0)
# include_directories(${GLIB_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# CameraDisplay （相机查验）
set(CAMERA_DISPLAY_PROJECT_NAME "camera_display")
set(CAMERA_DISPLAY_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/camera_display.cpp)
add_executable(${CAMERA_DISPLAY_PROJECT_NAME} ${CAMERA_DISPLAY_PROJECT_SOURCES})
target_link_libraries(${CAMERA_DISPLAY_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# UartTest （通信查验）
set(UART_PROJECT_NAME "uart_test")
set(UART_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/uart_test.cpp)
add_executable(${UART_PROJECT_NAME} ${UART_PROJECT_SOURCES})
target_link_libraries(${UART_PROJECT_NAME} PRIVATE pthread )
target_link_libraries(${UART_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
target_link_libraries(${UART_PROJECT_NAME} PRIVATE serial)

# ColorDetect (色块识别)
set(COLOR_PROJECT_NAME "color_detect")
set(COLOR_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/color_detect.cpp)
add_executable(${COLOR_PROJECT_NAME} ${COLOR_PROJECT_SOURCES})
target_link_libraries(${COLOR_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# HSVDetect (色块识别)
set(HSV_PROJECT_NAME "hsv_detect")
set(HSV_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/hsv_detect.cpp)
add_executable(${HSV_PROJECT_NAME} ${HSV_PROJECT_SOURCES})
target_link_libraries(${HSV_PROJECT_NAME} PRIVATE pthread )
target_link_libraries(${HSV_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
target_link_libraries(${HSV_PROJECT_NAME} PRIVATE serial)


# RectDetect (矩形识别)
set(RECT_PROJECT_NAME "rect_detect")
set(RECT_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/rect_detect.cpp)
add_executable(${RECT_PROJECT_NAME} ${RECT_PROJECT_SOURCES})
target_link_libraries(${RECT_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# LaserDetect (激光点识别)
set(Laser_PROJECT_NAME "laser_detect")
set(Laser_PROJECT_SOURCES ${PROJECT_SOURCE_DIR}/tools/laser_detect.cpp)
add_executable(${Laser_PROJECT_NAME} ${Laser_PROJECT_SOURCES})
target_link_libraries(${Laser_PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# shoot (自瞄主程序)
set(PROJECT_NAME "shoot")
add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/shoot.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE serial)
