cmake_minimum_required(VERSION 2.8)
project(dji)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")
link_directories(/home/jjt/Documents/Flight23/lib)
link_directories(/usr/local/lib)
include_directories(/home/jjt/Documents/Flight23)
include_directories(/usr/local/include/eigen3)
include_directories(/home/jjt/Downloads/Onboard-SDK-3.2/osdk-core/inc)
include_directories(/home/jjt/Downloads/Onboard-SDK-3.2/osdk-wrapper/inc)
include_directories(/home/jjt/Downloads/Onboard-SDK-3.2/platform/linux/inc)


find_package(OpenCV REQUIRED)


add_executable(${PROJECT_NAME} main2.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} pthread libaruco.so libusb-1.0.so libdjiosdk-core.a libdjiosdk-platform.a libdjiosdk-wrapper.a)
