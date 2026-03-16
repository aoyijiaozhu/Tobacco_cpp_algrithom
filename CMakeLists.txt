cmake_minimum_required(VERSION 3.10)

# 设置 vcpkg 路径（保留你本地的路径设置即可）
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND EXISTS "D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
endif()

project(TobaccoVision VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 【关键优化】：解决 MSVC 下由于中文字符导致的 C2001 和 C2143 报错！
if(MSVC)
    add_compile_options("/utf-8")
endif()

# --- 依赖查找 --- 

# 1. 查找 OpenCV
find_package(OpenCV REQUIRED)

# 2. ZeroMQ & nlohmann_json (你同事已经将它们内置在 include/ 目录中了，不再需要 find_package)

# 包含头文件目录 (包含系统 OpenCV 和本地 include 目录)
include_directories(${OpenCV_INCLUDE_DIRS} include)

# 【关键修改】：把同事新增的 ZMQ 和 BatchLoader 源文件加进来
add_executable(vision_test 
    main.cpp 
    src/vision_system.cpp
    src/locator.cpp
    src/mapping_system.cpp
    src/defect_detector.cpp
    src/batch_loader.cpp
    src/zmq_publisher.cpp
)

# 【关键修改】：链接 OpenCV 库，并链接系统中的 zmq 库
target_link_libraries(vision_test PRIVATE 
    ${OpenCV_LIBS}
    zmq 
)
