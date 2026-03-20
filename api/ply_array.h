/**
 * @file ply_array.h
 * @brief PLY 点云转一维深度数组接口
 */
#pragma once
#include <string>
#include <vector>

/**
 * @brief 将 PLY 点云文件转换为固定尺寸的一维深度数组
 *
 * 内部通过 popen 调用 Python 脚本（integrated_pipeline_optimized.py），
 * 执行点云裁剪、离群点去除和网格化，返回 20x450 = 9000 个 float。
 *
 * @param plyPath PLY 点云文件路径
 * @return 行优先的一维深度数组，大小为 9000（grid_y=20, grid_x=450）
 * @throws std::runtime_error config.json 读取失败、Python 脚本执行失败或返回错误
 */
std::vector<float> plyToArray(const std::string& plyPath);
