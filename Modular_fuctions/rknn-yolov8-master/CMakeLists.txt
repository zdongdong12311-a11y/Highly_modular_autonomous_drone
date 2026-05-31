# 设置最低版本号
cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
# 设置项目名称
project(rknn_yolov8_ros VERSION 0.0.1 LANGUAGES CXX)

message(STATUS "System: ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION}")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(LIB_ARCH "aarch64")

set(RKNN_API_PATH ${CMAKE_CURRENT_SOURCE_DIR}/librknn_api)
set(RKNN_API_INCLUDE_PATH ${RKNN_API_PATH}/include)
set(RKNN_API_LIB_PATH ${RKNN_API_PATH}/${LIB_ARCH}/librknnrt.so)
if(NOT EXISTS "${RKNN_API_LIB_PATH}")
    message(FATAL_ERROR
        "RKNN runtime not found: ${RKNN_API_LIB_PATH}\n"
        "On RK3588: bash scripts/fetch_rknn_runtime.sh\n"
        "Or see: librknn_api/aarch64/README.md")
endif()

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV: ${OpenCV_VERSION}")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${RKNN_API_INCLUDE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_library(nn_process SHARED
    src/process/preprocess.cpp
    src/process/postprocess.cpp
)
target_link_libraries(nn_process ${OpenCV_LIBS})

add_library(rknn_engine SHARED src/engine/rknn_engine.cpp)
target_link_libraries(rknn_engine ${RKNN_API_LIB_PATH})

add_library(yolov8_lib SHARED src/task/yolov8.cpp)
target_link_libraries(yolov8_lib rknn_engine nn_process)

add_library(draw_lib SHARED src/draw/cv_draw.cpp)
target_link_libraries(draw_lib ${OpenCV_LIBS})

add_executable(yolov8_img src/yolov8_img.cpp)
target_link_libraries(yolov8_img draw_lib yolov8_lib)

add_executable(yolov8_video src/yolov8_video.cpp)
target_link_libraries(yolov8_video draw_lib yolov8_lib)

find_package(catkin QUIET COMPONENTS roscpp sensor_msgs cv_bridge vision_msgs image_transport)
if(catkin_FOUND)
    catkin_package(
        CATKIN_DEPENDS roscpp sensor_msgs cv_bridge vision_msgs image_transport
    )

    find_package(Threads REQUIRED)

    add_executable(yolov8_ros_node src/yolov8_ros_node.cpp)
    target_include_directories(yolov8_ros_node PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(yolov8_ros_node
        draw_lib
        yolov8_lib
        ${catkin_LIBRARIES}
        Threads::Threads
    )

    install(TARGETS yolov8_ros_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    )
endif()
