#!/bin/bash
# install_opencv_cuda.sh: 在 Ubuntu 环境下编译安装带 CUDA 支持的 OpenCV

set -e

OPENCV_VERSION=4.8.0
INSTALL_DIR=/usr/local

echo ">>> Installing dependencies..."
apt-get update && apt-get install -y \
    build-essential cmake git pkg-config \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libgtk-3-dev libatlas-base-dev gfortran \
    python3-dev

echo ">>> Cloning OpenCV ${OPENCV_VERSION}..."
cd /tmp
git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv.git
git clone --depth 1 --branch ${OPENCV_VERSION} https://github.com/opencv/opencv_contrib.git

echo ">>> Building OpenCV with CUDA support (ARCH 7.5 for GTX 1660 Ti)..."
cd opencv
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
      -D INSTALL_PYTHON_EXAMPLES=OFF \
      -D INSTALL_C_EXAMPLES=OFF \
      -D OPENCV_ENABLE_NONFREE=ON \
      -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
      -D WITH_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D WITH_CUBLAS=ON \
      -D CUDA_ARCH_BIN=7.5 \
      -D BUILD_EXAMPLES=OFF ..

make -j$(nproc)
make install
ldconfig

echo ">>> OpenCV ${OPENCV_VERSION} with CUDA installed successfully."
