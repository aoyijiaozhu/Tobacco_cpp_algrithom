cmake_minimum_required(VERSION 3.10)

# Vcpkg toolchain configuration (Windows only)
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND EXISTS "D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "D:/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
endif()

project(TobaccoVision
    VERSION 2.0.0
    DESCRIPTION "Tobacco Vision System with Line-Scan 3D Reconstruction"
    LANGUAGES CXX
)

# C++ standard configuration
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type configuration
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Compiler-specific options
if(MSVC)
    add_compile_options(/utf-8 /W3)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -O3 -march=native)
endif()

# Dependencies
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui cudaarithm cudaimgproc cudafilters cudawarping)
find_package(Threads REQUIRED)

# Optional CUDA support detection
if(OpenCV_CUDA_VERSION)
    message(STATUS "OpenCV built with CUDA ${OpenCV_CUDA_VERSION}")
    add_compile_definitions(HAVE_OPENCV_CUDAIMGPROC HAVE_OPENCV_CUDAFILTERS HAVE_OPENCV_CUDAARITHM)
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# Source files
set(SOURCES
    main.cpp
    src/vision_system.cpp
    src/locator.cpp
    src/mapping_system.cpp
    src/defect_detector.cpp
    src/batch_loader.cpp
    src/zmq_publisher.cpp
    src/ply_processor.cpp
)

# Executable target
add_executable(vision_test ${SOURCES})

# Link libraries
target_link_libraries(vision_test PRIVATE
    ${OpenCV_LIBS}
    Threads::Threads
    zmq
)

# Installation rules
install(TARGETS vision_test DESTINATION bin)
install(FILES config.json DESTINATION etc)

# Print configuration summary
message(STATUS "=== Build Configuration ===")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "OpenCV Libraries: ${OpenCV_LIBS}")
message(STATUS "CUDA Support: ${OpenCV_CUDA_VERSION}")
