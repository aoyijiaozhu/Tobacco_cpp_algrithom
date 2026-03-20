/**
 * @file ply_array.cpp
 * @brief plyToArray() 实现 -- 通过 Python 子进程将 PLY 点云转为一维深度数组
 */
#include "ply_array.h"
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <array>
#include "../include/json.hpp"

using json = nlohmann::json;

std::vector<float> plyToArray(const std::string& plyPath) {
    // 从 config.json 读取 Python 解释器路径和脚本路径
    std::ifstream ifs("config.json");
    if (!ifs.is_open())
        throw std::runtime_error("Cannot open config.json");
    json cfg = json::parse(ifs);
    std::string pythonBin  = cfg.at("python").at("python_bin").get<std::string>();
    std::string scriptPath = cfg.at("python").at("script_path").get<std::string>();

    // 构建命令行：python script.py "ply_path" 2>&1
    std::string cmd = pythonBin + " " + scriptPath + " "
                    + "\"" + plyPath + "\"" + " 2>&1";

    // 通过 popen 调用 Python 脚本，读取 stdout 输出
    std::array<char, 4096> buf;
    std::string output;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe)
        throw std::runtime_error("popen failed");

    while (fgets(buf.data(), buf.size(), pipe))
        output.append(buf.data());

    int ret = pclose(pipe);
    if (ret != 0)
        throw std::runtime_error("Python script failed: " + output);

    // 解析 Python 输出的 JSON 数组
    json j = json::parse(output);

    // 检查是否返回了错误对象
    if (j.is_object() && j.contains("error"))
        throw std::runtime_error("Python error: " + j["error"].get<std::string>());

    // 将 JSON 数组转为 vector<float>
    std::vector<float> result;
    result.reserve(j.size());
    for (auto& v : j)
        result.push_back(v.get<float>());

    return result;
}
