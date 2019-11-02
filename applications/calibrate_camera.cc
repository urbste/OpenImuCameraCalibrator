#include <dirent.h>
#include <gflags/gflags.h>
#include <time.h>
#include <algorithm>
#include <chrono>  // NOLINT
#include <string>
#include <vector>

#include <theia/sfm/bundle_adjustment/bundle_adjustment.h>
#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/feature_correspondence_2d_3d.h>
#include <theia/sfm/reconstruction.h>
#include <theia/solvers/ransac.h>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>

#include "OpenCameraCalibrator/utils/utils.h"

using namespace cv;

DEFINE_string(input_video, "", "Path to save charuco board to.");
DEFINE_string(detector_params, "", "Path detector yaml.");


std::vector<std::string> load_images(const std::string& img_dir_path) {
  DIR* dir;
  if ((dir = opendir(img_dir_path.c_str())) == nullptr) {
  }
  std::vector<std::string> img_paths;
  dirent* dp;
  for (dp = readdir(dir); dp != nullptr; dp = readdir(dir)) {
    img_paths.push_back(img_dir_path + "/" + std::string(dp->d_name));
  }
  closedir(dir);

  std::sort(img_paths.begin(), img_paths.end());
  return img_paths;
}

using Vector2D = Eigen::Vector2d;
using image_points_t =
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

int main(int argc, char* argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  int squaresX = 10;
  int squaresY = 8;
  int squareLength = 2;
  int markerLength = 1;
  int dictionaryId = cv::aruco::DICT_ARUCO_ORIGINAL;
  int margins = squareLength - markerLength;
  int borderBits = 1;
  bool showImage = true;
  const int min_number_detected_corners = 30;

  theia::RansacParameters ransac_params;
  ransac_params.error_thresh = 1.0;
  ransac_params.failure_probability = 0.01;
  ransac_params.min_iterations = 10;

  theia::Reconstruction recon_calib_dataset;

  // load images from folder
  Ptr<aruco::DetectorParameters> detectorParams =
      aruco::DetectorParameters::create();

  if (!OpenCamCalib::ReadDetectorParameters(FLAGS_detector_params, detectorParams)) {
    std::cerr << "Invalid detector parameters file\n";
    return 0;
  }

  Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(
      aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
  // create charuco board object
  Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(
      squaresX, squaresY, squareLength, markerLength, dictionary);
  Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

  theia::RandomNumberGenerator gne;
  // fill reconstruction with charuco points
  std::vector<cv::Point3f> chessoard3d = charucoboard->chessboardCorners;
  std::map<int, theia::TrackId> charuco_id_to_theia_track_id;
  for (int i = 0; i < chessoard3d.size(); ++i) {
    theia::TrackId track_id = recon_calib_dataset.AddTrack();
    theia::Track* track = recon_calib_dataset.MutableTrack(track_id);
    track->SetEstimated(true);
    Eigen::Vector4d* point = track->MutablePoint();
    (*point)[0] = static_cast<double>(chessoard3d[i].x);
    (*point)[1] = static_cast<double>(chessoard3d[i].y);
    (*point)[2] = static_cast<double>(chessoard3d[i].z);
    (*point)[3] = 1.0;
    charuco_id_to_theia_track_id[i] = track_id;
  }

  VideoCapture inputVideo;
  inputVideo.open(FLAGS_input_video);
  bool validPose = false;
  bool showRejected = false;
  int cnt_wrong = 0;
  const int skip_frames = 1;
  int frame_cnt = 0;
  bool first_detection = false;  // at the first detection fill a theia
                                 // reconstruction with keypoints
  while (true) {
    Mat image, imageCopy;
    if (!inputVideo.read(image)) {
      cnt_wrong++;
      if (cnt_wrong > 200) break;
      continue;
    }
    ++frame_cnt;
    std::cout << frame_cnt << " % " << skip_frames << " = "
              << frame_cnt % skip_frames << std::endl;
    if (frame_cnt % skip_frames != 0) continue;

    std::vector<int> markerIds, charucoIds;
    std::vector<std::vector<Point2f>> markerCorners, rejectedMarkers;
    std::vector<Point2f> charucoCorners;

    // detect markers
    aruco::detectMarkers(image, dictionary, markerCorners, markerIds,
                         detectorParams, rejectedMarkers);

    // refind strategy to detect more markers
    aruco::refineDetectedMarkers(image, board, markerCorners, markerIds,
                                 rejectedMarkers);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if (markerIds.size() > 0)
      interpolatedCorners = aruco::interpolateCornersCharuco(
          markerCorners, markerIds, image, charucoboard, charucoCorners,
          charucoIds);

    if (charucoIds.size() < min_number_detected_corners) continue;
    // draw results
    image.copyTo(imageCopy);
    if (markerIds.size() > 0) {
      aruco::drawDetectedMarkers(imageCopy, markerCorners);
    }

    if (showRejected && rejectedMarkers.size() > 0)
      aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(),
                                 Scalar(100, 0, 255));

    if (interpolatedCorners > 0) {
      Scalar color;
      color = Scalar(255, 0, 0);
      aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds,
                                        color);
    }

    // Estimate pose
    double px = static_cast<double>(image.cols) / 2.0;
    double py = static_cast<double>(image.rows) / 2.0;
    std::vector<theia::FeatureCorrespondence2D3D> correspondences(
        charucoIds.size());
    for (int i = 0; i < charucoIds.size(); ++i) {
      theia::FeatureCorrespondence2D3D correspondence;
      correspondence.feature << (static_cast<double>(charucoCorners[i].x) - px),
          (static_cast<double>(charucoCorners[i].y) - py);
      theia::TrackId track_id =
          charuco_id_to_theia_track_id.find(charucoIds[i])->second;
      const Eigen::Vector4d track =
          recon_calib_dataset.Track(track_id)->Point();
      correspondence.world_point = track.hnormalized();
      correspondences[i] = correspondence;
    }
    theia::RadialDistUncalibratedAbsolutePose pose;
    theia::RansacSummary ransac_summary;
    theia::RadialDistUncalibratedAbsolutePoseMetaData meta_data;
    meta_data.max_focal_length = 1200;
    meta_data.min_focal_length = 500;
    theia::EstimateRadialDistUncalibratedAbsolutePose(
        ransac_params, theia::RansacType::RANSAC, correspondences, meta_data,
        &pose, &ransac_summary);

    if (ransac_summary.inliers.size() < charucoIds.size() * 0.5) continue;

    // fill charucoCorners to theia reconstruction
    theia::ViewId view_id =
        recon_calib_dataset.AddView(std::to_string(frame_cnt), 0);
    theia::View* view = recon_calib_dataset.MutableView(view_id);
    view->SetEstimated(true);

    std::cout << "Estimated focal length: " << pose.focal_length << std::endl;
    std::cout << "Estimated radial distortion: " << pose.radial_distortion
              << std::endl
              << std::endl;
    std::cout<< "Number of Ransac inliers: "<<ransac_summary.inliers.size()<<std::endl;

    theia::Camera* cam = view->MutableCamera();
    cam->SetImageSize(image.cols, image.rows);
    cam->SetPosition(-pose.rotation * pose.translation);
    cam->SetOrientationFromRotationMatrix(pose.rotation);
    cam->SetCameraIntrinsicsModelType(
        theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION);
    double* intrinsics = cam->mutable_intrinsics();
    intrinsics[theia::DivisionUndistortionCameraModel::InternalParametersIndex::
                   FOCAL_LENGTH] = pose.focal_length;
    intrinsics[theia::DivisionUndistortionCameraModel::InternalParametersIndex::
                   PRINCIPAL_POINT_X] = image.cols / 2.0;
    intrinsics[theia::DivisionUndistortionCameraModel::InternalParametersIndex::
                   PRINCIPAL_POINT_Y] = image.rows / 2.0;
    intrinsics[theia::DivisionUndistortionCameraModel::InternalParametersIndex::
                   RADIAL_DISTORTION_1] = pose.radial_distortion;

    for (int i = 0; i < charucoIds.size(); ++i) {
      theia::TrackId track_id =
          charuco_id_to_theia_track_id.find(charucoIds[i])->second;
      theia::Feature feature;
      feature << static_cast<double>(charucoCorners[i].x),
          static_cast<double>(charucoCorners[i].y);
      recon_calib_dataset.AddObservation(view_id, track_id, feature);
    }

    imshow("out", imageCopy);
    char key = (char)waitKey(1);
    if (key == 27) break;
  }

  theia::BundleAdjustmentOptions ba_options;
  ba_options.verbose = true;
  ba_options.loss_function_type = theia::LossFunctionType::HUBER;
  ba_options.robust_loss_width = 1.345;
  ba_options.intrinsics_to_optimize =
      theia::OptimizeIntrinsicsType::FOCAL_LENGTH;  // ||
  //            theia::OptimizeIntrinsicsType::RADIAL_DISTORTION ||
  //            theia::OptimizeIntrinsicsType::PRINCIPAL_POINTS ||
  //            theia::OptimizeIntrinsicsType::ASPECT_RATIO;
  theia::BundleAdjustmentSummary summary = theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset);

  std::cout<<"Optimized camera focal length: "<<recon_calib_dataset.MutableView(0)->Camera().FocalLength()<<std::endl;
  std::cout<<"Mean reprojection error: "<<summary.final_cost / recon_calib_dataset.NumViews() / charuco_id_to_theia_track_id.size()<<std::endl;

  return 0;
}
