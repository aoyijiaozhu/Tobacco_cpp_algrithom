/**
 * @file common_types.hpp
 * @brief 公共类型定义，供 API 层和测试层使用
 */
#pragma once
#include <map>
#include <vector>

/**
 * 相机流数据类型
 * key:   相机 ID（如 101）
 * value: 帧序列，每帧为 16 位深度图的原始字节（行优先，大小 = width * height * 2）
 */
typedef std::map<int, std::vector<std::vector<unsigned char>>> CameraStreamData;
