# 项目优化报告2026年2月4日

本文档详细记录了从初始版本到当前版本的代码演进过程，涵盖了架构重构、GPU深度优化以及工程化适配等关键变更。

## 1. 架构演进：从单线程到“生产者-消费者”模型

### 初始版本
- **逻辑**：串行执行。读取图片 -> 处理图片 -> 保存结果 -> 读取下一张。
- **瓶颈**：CPU读取图片（IO密集型）和GPU处理图片（计算密集型）串行进行，无法充分利用硬件资源。

### 当前版本
- **引入 `BatchLoader` 模块**：
  - 实现了一个后台线程，专门负责从磁盘读取图片。
  - 维护一个 `std::set` 用于文件去重，防止重复处理。
  - 将读取到的图片打包成 Batch（如 32张/组），放入线程安全的阻塞队列。
- **主线程重构**：
  - 变为消费者角色，循环从队列中获取 Batch 数据。
  - 调用 `detector.predictBatch` 进行批量推理。
  - 实现了 CPU 读取与 GPU 计算的并行流水线。
- **效果**：显著减少了 GPU 等待数据的时间，大幅提升了吞吐量。

## 2. GPU 深度优化

### 初始版本
- **逻辑**：主要依赖 CPU 版 OpenCV (`cv::Mat`) 进行图像处理（高斯滤波、阈值分割、形态学操作）。
- **问题**：CPU 计算慢，且涉及频繁的 CPU-GPU 内存拷贝（如果部分步骤用了 GPU）。

### 当前版本
- **全流程 GPU 化**：
  - 使用 `cv::cuda::GpuMat` 替代 `cv::Mat`。
  - 滤波 (`cuda::createGaussianFilter`)、形态学 (`cuda::createMorphologyFilter`)、算术运算 (`cuda::subtract`, `cuda::compare`) 全部在 GPU 上执行。
  - 只有在最后需要轮廓提取 (`findContours`) 时才将二值图下载回 CPU。
- **Lazy Visualization (懒加载可视化)**：
  - **优化前**：每帧都将处理后的深度图、缺陷掩膜下载回 CPU 用于显示/保存，消耗大量 PCIe 带宽。
  - **优化后**：仅下载必要的二值掩膜 (`mask_cpu`) 用于轮廓分析。仅在确实检测到缺陷需要可视化时，才下载完整的渲染图。
- **ROI-based Processing (基于感兴趣区域的处理)**：
  - **优化前**：对全图进行 `minMaxLoc` 计算高度极值。
  - **优化后**：仅在检测到缺陷的连通域（ROI）内计算 `minMaxLoc`，大幅减少计算量。
- **批量接口 (`predictBatch`)**：
  - 预留了批量处理接口，为未来使用 CUDA Stream 进一步并行化打下基础。

## 3. 工程化适配

- **构建系统**：
  - 移除 `vcpkg` 强依赖，改为优先查找系统安装的 OpenCV（适配 Docker 环境）。
  - 添加 `docker_build.sh` 和 `install_opencv_cuda.sh`，支持在 Linux/Docker 环境下的一键编译和部署。
  - 针对特定 GPU (GTX 1660 Ti) 设置 `CUDA_ARCH_BIN=7.5` 编译选项。
- **跨平台兼容**：
  - 将 Windows 特有的 `system("pause")` 替换为跨平台的 `std::cin.get()`。
  - 路径处理改为相对路径并增加父目录搜索逻辑，适应不同运行目录。

---

## 4. AI 修改提示词 (Prompts for Reconstruction)

如果您希望使用 AI 将初始版本代码重构为当前版本，可以按顺序使用以下提示词：

### 阶段一：工程化与环境适配
```text
请优化项目的构建系统以支持 Docker 和 Linux 环境：
1. 修改 CMakeLists.txt：移除对 vcpkg 的强制依赖，改为优先查找系统路径下的 OpenCV。
2. 创建 install_opencv_cuda.sh 脚本：用于在 Ubuntu/Docker 环境下编译安装带 CUDA 支持的 OpenCV (4.x版本)，并针对 GTX 1660 Ti 设置 CUDA_ARCH_BIN=7.5。
3. 创建 docker_build.sh：用于清理缓存并执行 cmake build。
4. 修改 main.cpp：将 system("pause") 替换为 std::cin.get() 以实现跨平台暂停。
```

### 阶段二：GPU 算法移植 (基础版)
```text
请将 DefectDetector 类的核心算法移植到 GPU：
1. 修改 defect_detector.hpp/cpp，引入 <opencv2/cudafilters.hpp> 等 CUDA 头文件。
2. 将 cv::Mat 替换为 cv::cuda::GpuMat 作为主要数据结构。
3. 使用 cv::cuda::createGaussianFilter 替代 cv::GaussianBlur。
4. 使用 cv::cuda::threshold, cv::cuda::subtract, cv::cuda::compare 等 CUDA 接口替代对应的 CPU 函数。
5. 注意：findContours 仍然需要在 CPU 上执行，请在最后一步将 mask 下载回 CPU。
```

### 阶段三：GPU 深度优化 (性能版)
```text
在 GPU 版本的基础上进行深度性能优化：
1. 实现 "Lazy Download"：不要每帧都下载用于可视化的 r_dep 图像。仅在检测到缺陷（contours不为空）时，才下载必要的图像数据进行绘制。
2. 实现 "ROI Optimization"：不要对全图进行 minMaxLoc。仅在 findContours 找到缺陷后，利用 boundingRect 确定 ROI 区域，仅在该区域内的 GpuMat 上计算最大高度值。
3. 显存优化：将高斯滤波器和形态学滤波器对象作为类成员变量预先分配，避免每帧重复创建。
```

### 阶段四：引入生产者-消费者模型 (架构重构)
```text
请重构 main.cpp 的处理流程，实现“生产者-消费者”模型以提升吞吐量：
1. 新建 BatchLoader 类 (batch_loader.hpp/cpp)：
   - 启动一个后台线程，持续扫描文件夹中的新图片。
   - 维护一个 std::set<string> 用于文件去重（根据文件名）。
   - 将读取到的图片打包成 BatchData（包含 vector<Mat> images, vector<string> filenames），放入线程安全的阻塞队列。
   - 暂时使用单线程顺序读取以保证稳定性。
2. 修改 DefectDetector：增加 predictBatch(vector<Mat>) 接口。
3. 修改 main.cpp：
   - 实例化 BatchLoader 并启动。
   - 主循环改为从 BatchLoader 的队列中获取 batch。
   - 遍历 batch 中的每一帧调用 detector 处理。
```
