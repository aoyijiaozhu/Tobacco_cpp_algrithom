#pragma once
#include <string>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include "json.hpp"

using json = nlohmann::json;

/**
 * @brief Image frame data structure
 * @details Stores raw image data with dimensions
 */
struct ImageFrame {
    std::vector<unsigned char> data;  // Raw image buffer
    int width;                         // Image width in pixels
    int height;                        // Image height in pixels
};

/**
 * @brief Conveyor flow analysis result
 * @details Used for tobacco flow rate detection on conveyor belt
 */
struct FlowResult {
    bool hasTobacco;      // Whether tobacco is detected on belt
    double flowRate;      // Flow rate in kg/s or m³/s
    double timestamp;     // Timestamp in seconds since epoch
};

/**
 * @brief Carriage state enumeration
 * @details Represents the operational state of the material carriage
 */
enum class CarriageState {
    STOPPED,    // Carriage is stationary
    FILLING,    // Carriage is being filled with material
    RETURNING   // Carriage is returning to start position
};

/**
 * @brief Carriage detection result
 * @details Contains state and diagnostic information for carriage system
 */
struct CarriageResult {
    CarriageState state;  // Current carriage state
    std::string info;     // Additional diagnostic information
    double timestamp;     // Timestamp in seconds since epoch
};

/**
 * @brief Anomaly detection result
 * @details Describes detected defects or anomalies in material
 */
struct AnomalyResult {
    bool hasAnomaly;      // Whether any anomaly was detected
    std::string type;     // Anomaly type (e.g., "HOLE", "BUMP", "MULTIPLE")
    double severity;      // Severity measure (typically deviation in mm)
    std::string details;  // JSON string with detailed defect information
};

/**
 * @brief Camera stream data container
 * @details Maps camera ID to list of raw image buffers
 * @note Key: camera ID (e.g., 101), Value: vector of encoded image buffers
 */
typedef std::map<int, std::vector<std::vector<unsigned char>>> CameraStreamData;

/**
 * @brief 烟草视觉检测系统主接口
 * @details 提供线扫描3D重建、缺陷检测、体积计算等核心功能
 */
class TobaccoVisionSystem {
public:
    /**
     * @brief 初始化视觉系统
     * @param configPath 配置文件路径 (通常为 config.json)
     * @details 加载系统参数，初始化映射系统和缺陷检测器
     */
    static void init(const std::string& configPath);

    /**
     * @brief 重置会话状态
     * @details 清空累积的高程图和点云数据，用于开始新批次处理
     */
    static void resetSession();

    /**
     * @brief 分析传送带流速
     * @param frame 输入图像帧
     * @param elapsed 已过时间 (秒)
     * @return FlowResult 流速分析结果
     */
    static FlowResult analyzeConveyorFlow(const ImageFrame& frame, double elapsed);

    /**
     * @brief 检测行车状态
     * @param rgbFrame RGB图像帧
     * @param sideInfo 侧面信息字符串
     * @param elapsed 已过时间 (秒)
     * @return CarriageResult 行车状态检测结果
     */
    static CarriageResult detectCarriageState(const ImageFrame& rgbFrame, const std::string& sideInfo, double elapsed);

    /**
     * @brief 生成点云 (单帧处理)
     * @param streams 相机流数据 (camera_id -> 图像缓冲区列表)
     * @param frameId 帧编号
     * @return std::string 处理状态 ("processed", "cached", "error")
     * @details 解码深度图并拼接到全局高程图
     */
    static std::string generatePointCloud(const CameraStreamData& streams, int frameId);

    /**
     * @brief 处理单帧深度图 (Mat格式)
     * @param depth 深度图 (CV_16UC1, 单位: mm)
     * @param frameId 帧编号
     * @return std::string 处理状态
     */
    static std::string processFrameMat(const cv::Mat& depth, int frameId);

    /**
     * @brief 批量处理深度图帧
     * @param depth_batch 深度图批次
     * @param frame_ids 对应的帧编号列表
     * @details 批量拼接，提高处理效率
     */
    static void processFrameBatch(const std::vector<cv::Mat>& depth_batch, const std::vector<int>& frame_ids);

    /**
     * @brief 生成完整PLY点云文件
     * @param batchId 批次编号
     * @return std::string PLY文件路径
     * @details 从累积的高程图生成无断层点云
     */
    static std::string generateFullPly(int batchId);

    /**
     * @brief 计算体积和质量
     * @param plyPath PLY文件路径 (当前版本未使用，直接从内存计算)
     * @param outMass 输出质量 (kg)
     * @return double 体积 (m³)
     * @details 基于高程图积分计算总体积和质量
     */
    static double calculateVolume(const std::string& plyPath, double& outMass);

    /**
     * @brief 获取拼接长度
     * @return double 累积拼接的物理长度 (米)
     */
    static double getStitchedLength();

    /**
     * @brief 检测缺陷 (标准接口)
     * @param plyPath PLY文件路径
     * @return AnomalyResult 缺陷检测结果
     * @details 后端契约的标准接口
     */
    static AnomalyResult detectAnomalies(const std::string& plyPath);

    /**
     * @brief 检测缺陷 (ZMQ专用接口)
     * @return json 缺陷列表 (JSON数组)
     * @details 直接从内存中的高程图检测，返回JSON格式结果用于ZMQ发布
     */
    static json detectAnomaliesByMat();
};