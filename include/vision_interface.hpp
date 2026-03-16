#pragma once
#include <string>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include "json.hpp" 

using json = nlohmann::json;

struct ImageFrame {
    std::vector<unsigned char> data;
    int width;
    int height;
};

// 严格对齐后端的流速结构
struct FlowResult {
    bool hasTobacco;      
    double flowRate;      
    double timestamp;     
};

// 严格对齐后端的行车状态结构
enum class CarriageState {
    STOPPED,    
    FILLING,    
    RETURNING   
};

struct CarriageResult {
    CarriageState state;  
    std::string info;     
    double timestamp;     
};

struct AnomalyResult {
    bool hasAnomaly;
    std::string type;
    double severity;
    std::string details;
};

typedef std::map<int, std::vector<std::vector<unsigned char>>> CameraStreamData;

class TobaccoVisionSystem {
public:
    static void init(const std::string& configPath);
    static void resetSession();
    
    static FlowResult analyzeConveyorFlow(const ImageFrame& frame, double elapsed);
    static CarriageResult detectCarriageState(const ImageFrame& rgbFrame, const std::string& sideInfo, double elapsed);
    
    static std::string generatePointCloud(const CameraStreamData& streams, int frameId);
    static std::string processFrameMat(const cv::Mat& depth, int frameId);
    static std::string generateFullPly(int batchId);
    
    static double calculateVolume(const std::string& plyPath, double& outMass);
    static double getStitchedLength();

    // 后端契约的标准接口
    static AnomalyResult detectAnomalies(const std::string& plyPath);
    
    // 【你同事加入的 ZMQ 专用接口】
    static json detectAnomaliesByMat(); 
};