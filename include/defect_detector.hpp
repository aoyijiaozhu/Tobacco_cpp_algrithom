#ifndef DEFECT_DETECTOR_HPP
#define DEFECT_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include "json.hpp"

using json = nlohmann::json;
using namespace cv;

class DefectDetector {
public:
    DefectDetector(const std::string& config_path);
    json detect_global(const Mat& dem_map);

private:
    json detect_global_cpu(const Mat& dem_map);
#if defined(HAVE_OPENCV_CUDAIMGPROC) && defined(HAVE_OPENCV_CUDAFILTERS) && defined(HAVE_OPENCV_CUDAARITHM)
    json detect_global_cuda(const Mat& dem_map);
#endif
    bool cuda_enabled;
    double defect_thresh_mm;
    double min_long_mm;
    double min_short_mm;
    int margin_x;
    int margin_y;
    int bg_kernel_width;
    int morph_kernel_size;
    double max_valid_height_mm;
    double min_valid_height_mm;
    bool draw_filtered;
};

#endif
