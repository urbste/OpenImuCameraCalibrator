
#include <vector>
#include <opencv2/opencv.hpp>

namespace OpenICC {

struct ApriltagDetectorData;

class ApriltagDetector {
 public:
  ApriltagDetector();

  ~ApriltagDetector();

  void detectTags(const cv::Mat& img_raw,
                  std::vector<cv::Point2f> &corners,
                  std::vector<int>& ids,
                  std::vector<double>& radii,
                  std::vector<cv::Point2f> &corners_rejected,
                  std::vector<int>& ids_rejected,
                  std::vector<double>& radii_rejected);

 private:
  ApriltagDetectorData* data;
};

}  // namespace OpenICC
