# TobaccoVision CMakeLists.txt
#
# 构建目标：
#   tobacco_api  -- 静态库，包含核心功能（拼接/检测/API），供外部项目链接
#   test_stream  -- 集成测试可执行文件
#
# 依赖：OpenCV >= 4.x（必须），OpenCV CUDA 模块（可选，自动检测）
# 构建：rm -rf build && mkdir build && cd build && cmake .. && make -j$(nproc)

cmake_minimum_required(VERSION 3.10)
project(TobaccoVision VERSION 2.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC 下需要显式指定 UTF-8 编码，避免中文注释编译报错
if(MSVC)
    add_compile_options("/utf-8")
endif()

find_package(OpenCV REQUIRED)
# 头文件搜索路径：OpenCV 系统路径 + 本项目 include 目录
include_directories(${OpenCV_INCLUDE_DIRS} include)

# tobacco_api 静态库：包含拼接、检测和 API 层的全部实现
# 外部项目链接此库即可使用 analyzeStream() 和 plyToArray()
add_library(tobacco_api STATIC
    src/mapping_system.cpp      # 线扫描拼接、PLY 导出、体积计算
    src/defect_detector.cpp     # 表面缺陷检测（凹坑/凸起）
    api/analyze_stream.cpp      # 核心分析流水线
    api/ply_array.cpp           # PLY 转一维深度数组
)
target_link_libraries(tobacco_api PUBLIC ${OpenCV_LIBS})

# test_stream：集成测试，验证单相机分析和 plyToArray 功能
add_executable(test_stream test/test_analyze_stream.cpp)
target_link_libraries(test_stream PRIVATE tobacco_api)
