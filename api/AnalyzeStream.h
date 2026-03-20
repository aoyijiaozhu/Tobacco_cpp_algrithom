/**
 * @file AnalyzeStream.h
 * @brief 核心分析接口 -- 对深度帧序列执行完整的拼接、建模、检测流水线
 */
#pragma once
#include "../include/common_types.hpp"
#include <string>

/**
 * @brief 对相机深度帧序列执行完整分析流水线
 *
 * 流水线依次执行：帧校验 -> DEM 拼接 -> PLY 点云导出 -> 体积/质量计算 -> 缺陷检测
 *
 * @param streams 相机 ID 到帧序列的映射，每帧为 16 位深度图的原始字节
 * @param batchId 批次号，用于输出文件命名（PLY 文件和检测图）
 * @return JSON 字符串，包含每个相机的 ply_path、volume_m3、mass_kg、defects 等字段
 */
std::string analyzeStream(const CameraStreamData& streams, int batchId);
