# Tobacco Vision System

## 项目概述

基于 C++17 的烟叶视觉检测系统，采用线扫描深度相机对传送带上的烟叶进行实时三维重建，支持体积计算、缺陷检测和结果实时推送。

**版本**: v2.0
**语言**: C++17
**依赖**: OpenCV 4.x (CUDA), ZeroMQ, nlohmann-json

---

## 坐标系定义

| 轴 | 方向 | 物理范围 |
|----|------|----------|
| X  | 皮带运动方向 | 0.0 m ~ 20.0 m |
| Y  | 储柜宽度方向 | 0.0 m ~ 2.0 m  |
| Z  | 烟叶堆叠高度 | 0.0 m ~ 2.7 m  |

相机安装于顶部，`world_z = h_install - local_depth`，深度越小表示堆叠越高。

---

## 核心功能

### 1. 流式帧输入（vision_system）

- 接口接受内存二进制流（`std::vector<uint8_t>`），无需落盘。
- 支持单相机 / 双相机场景自动适配，解码后直接进入处理流水线。
- 对外暴露 `processFrameMat()` / `generatePointCloud()` 两个入口，分别对应 Mat 帧和相机流数据。

### 2. 全局高程图拼接（mapping_system）

- **线扫描累积**：每帧深度图经旋转、缩放（mm → m）、阈值过滤后，按照 `dist_per_frame = belt_speed / fps` 计算物理偏移，写入全局 DEM 矩阵（`dem_matrix`）。
- **双模执行**：
  - CPU 路径：`cv::Mat` + 标准 OpenCV 函数。
  - GPU 路径：运行时检测 `cv::cuda` 是否可用，可用则切换至 `cv::cuda::GpuMat`，对 `convertTo / threshold / compare / resize` 等操作全部卸载到 GPU，显著降低每帧处理延迟。
- **ROI 写入**：使用 `cv::Mat::operator()` 进行 ROI 索引，避免额外内存拷贝。

### 3. 点云生成（mapping_system）

- `generate_full_ply()` 以可配置步长（`ply_stride`）遍历 DEM，将有效高度点转换为 PLY 格式输出至 `output_ply/`。
- 生成结果为标准 ASCII PLY，可直接在 CloudCompare / MeshLab 中查看。

### 4. 体积与质量计算（mapping_system）

- `calculate_global_volume()` 对 DEM 中超过最小高度阈值的像素，按体素面积（`voxel_area = pixel_size_x × pixel_size_y`）累加体积。
- 质量 = 体积 × 烟叶密度（可在 `config.json` 中配置）。
- 每次 `publish_snapshot` 触发时计算并通过 ZMQ 发布。

### 5. 缺陷检测（defect_detector）

算法流程：

```
DEM 图 → 均值滤波（大窗口）→ 背景图 bg
         ↓
  diff_pit  = bg - dem   （凹坑：低于背景）
  diff_bump = dem - bg   （凸起：高于背景）
         ↓
  阈值化 + 安全边界掩码 → pit_mask / bump_mask
         ↓
  形态学开运算（去噪）→ 轮廓提取
         ↓
  尺寸过滤（min_long_mm × min_short_mm）→ 有效缺陷
         ↓
  输出 JSON 事件列表（含类型、坐标、尺寸、最大偏差）
```

- **GPU 加速路径**：`detect_global_cuda()` 将背景差分、比较、形态学处理全部在 `cv::cuda::GpuMat` 上执行，CPU 仅负责轮廓提取和结果封装。
- **CPU 回退路径**：`detect_global_cpu()` 在无 CUDA 环境下自动启用。
- 检测结果写入 `result_images/` 并通过 ZMQ 发布。

### 6. 坐标映射（locator）

```
world_x = frame_id × dist_per_frame + local_y
world_y = camera_center_y + local_x   （camera_center_y = 1.0 m）
world_z = h_install - local_z
```

将图像像素坐标实时映射为储柜绝对物理坐标，缺陷位置直接以米为单位上报。

### 7. 异步批量数据加载（batch_loader）

- 主控线程（`worker_loop`）监控数据集目录，使用自然排序（`natural_sort`）保证帧序正确。
- 多个加载线程（`loader_worker`）并行解码图像，结果存入 `assembly_buffer`。
- 生产者-消费者队列（`batch_queue`）解耦 IO 与计算，避免 IO 阻塞处理流水线。
- 线程同步：`std::mutex` + `std::condition_variable` + `std::atomic`。

### 8. ZeroMQ 实时推送（zmq_publisher）

- PUB/SUB 模式，默认绑定 `tcp://*:5555`。
- 发布主题：
  - `DEFECTS`：JSON 格式缺陷事件列表，含坐标、类型、尺寸、偏差值。
  - `SUMMARY`：JSON 格式汇总信息，含拼接长度、体积、质量、PLY 路径、图片路径。
- 发布频率由 `publish_every_frames` 控制，与处理帧率解耦。

---

## 处理数据流

```
图像文件（磁盘）
    │
    ▼
BatchLoader（多线程异步解码）
    │
    ▼
vision_system::processFrameMat()
    │
    ▼
MappingSystem::process_frame()        ← GPU/CPU 自适应
    │  深度图 → 旋转/缩放/阈值 → 写入 DEM
    ▼
全局 DEM 矩阵（内存）
    │
    ├─── generate_full_ply()  → output_ply/*.ply
    │
    ├─── calculate_volume()   → 体积 / 质量
    │
    └─── DefectDetector::detect_global()  ← GPU/CPU 自适应
              │  背景差分 → 掩码 → 轮廓 → 缺陷事件
              ▼
         ZmqPublisher::publish()
              ├── DEFECTS  → 下游消费者
              └── SUMMARY  → 下游消费者
```

---

## 项目结构

```
Cpp_Backend_3.12_gpu/
├── CMakeLists.txt
├── config.json
├── main.cpp
├── include/
│   ├── vision_interface.hpp   # 系统顶层接口
│   ├── mapping_system.hpp     # 高程图拼接与点云生成
│   ├── defect_detector.hpp    # 缺陷检测
│   ├── locator.hpp            # 坐标映射
│   ├── batch_loader.hpp       # 异步图像加载
│   ├── zmq_publisher.hpp      # ZMQ 推送
│   ├── zmq.hpp
│   └── json.hpp
├── src/
│   ├── vision_system.cpp
│   ├── mapping_system.cpp
│   ├── defect_detector.cpp
│   ├── locator.cpp
│   ├── batch_loader.cpp
│   └── zmq_publisher.cpp
├── dataset/                   # 输入图像数据集
├── build/                     # 编译输出
├── output_ply/                # 点云文件
└── result_images/             # 检测结果图
```

---

## 环境安装（Ubuntu 22.04）

### 1. 系统更新与基础依赖

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y build-essential cmake git pkg-config libzmq3-dev
```

### 2. 安装 CUDA Toolkit

详细见https://developer.nvidia.com/cuda-12-6-0-download-archive

echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

nvcc --version   # 验证
```

### 3. 编译 OpenCV with CUDA

```bash
sudo apt install -y libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libtbb-dev libeigen3-dev

cd ~/Downloads
git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv.git
git clone --depth 1 --branch 4.10.0 https://github.com/opencv/opencv_contrib.git

cd opencv && mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN=8.6 \
      -D WITH_CUDNN=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D ENABLE_FAST_MATH=1 \
      -D CUDA_FAST_MATH=1 \
      -D WITH_CUBLAS=1 \
      -D WITH_TBB=ON \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_TESTS=OFF \
      ..

make -j$(nproc)
sudo make install
sudo ldconfig
```

> **`CUDA_ARCH_BIN` 对照**：RTX 30 系列 → `8.6`，RTX 40 系列 → `8.9`
> 查询命令：`nvidia-smi --query-gpu=compute_cap --format=csv`

### 4. 验证 OpenCV CUDA 支持

```bash
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -i cuda
```

---

## 编译

```bash
cd ~/Project/Cpp_Backend_3.12_gpu
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

---

## 配置说明

编辑 `config.json`：

```json
{
  "dataset_path": "dataset/depth",
  "system": {
    "install_height_mm": 2700.0,
    "belt_speed_m_s": 0.132,
    "fps": 5.0
  },
  "zmq": {
    "bind": "tcp://*:5555"
  }
}
```

| 字段 | 说明 |
|------|------|
| `install_height_mm` | 相机安装高度（mm），用于 Z 轴坐标换算 |
| `belt_speed_m_s` | 传送带线速度（m/s） |
| `fps` | 相机采集帧率，与 belt_speed 共同决定每帧行进距离 |
| `zmq.bind` | ZMQ 发布端地址 |

---

## 运行

```bash
cd build
./vision_test
```

---

## 输出说明

| 输出 | 位置 | 格式 |
|------|------|------|
| 点云文件 | `output_ply/*.ply` | ASCII PLY |
| 检测结果图 | `result_images/*.png` | PNG |
| 实时推送 | ZMQ `DEFECTS` 主题 | JSON |
| 汇总信息 | ZMQ `SUMMARY` 主题 | JSON |

**DEFECTS 消息示例**：
```json
{
  "type": "pit",
  "world_x": 3.42,
  "world_y": 0.87,
  "world_z": 2.31,
  "max_deviation_mm": 45.2,
  "size_long_mm": 120.0,
  "size_short_mm": 80.0
}
```

**SUMMARY 消息示例**：
```json
{
  "stitched_length_m": 5.28,
  "volume_m3": 1.74,
  "mass_kg": 870.0,
  "ply_path": "output_ply/frame_0100.ply",
  "result_image": "result_images/frame_0100.png"
}
```

---

## 常见问题

### CUDA 相关错误

```bash
nvcc --version
nvidia-smi
pkg-config --modversion opencv4
```

确认 OpenCV 编译时 `WITH_CUDA=ON` 且 `CUDA_ARCH_BIN` 与当前 GPU 一致。

### ZeroMQ 链接错误

```bash
sudo apt install -y libzmq3-dev
sudo ldconfig
```

### 权限问题

```bash
sudo chmod +x install_opencv_cuda.sh
```
