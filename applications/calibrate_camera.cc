#include <algorithm>
#include <chrono> // NOLINT
#include <fstream>
#include <gflags/gflags.h>
#include <string>
#include <time.h>
#include <vector>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>
#include <theia/io/reconstruction_writer.h>
#include <theia/sfm/bundle_adjustment/bundle_adjustment.h>
#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>
#include <theia/sfm/estimators/estimate_radial_dist_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/estimate_uncalibrated_absolute_pose.h>
#include <theia/sfm/estimators/feature_correspondence_2d_3d.h>
#include <theia/sfm/reconstruction.h>
#include <theia/solvers/ransac.h>

#include "OpenCameraCalibrator/utils/intrinsic_initializer.h"
#include "OpenCameraCalibrator/utils/json.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"

using namespace cv;
using namespace OpenCamCalib;
using namespace OpenCamCalib::utils;
//DEFINE_string(input_video, "", "Path to save charuco board to.");
//DEFINE_string(detector_params, "", "Path detector yaml.");
//DEFINE_string(camera_model_to_calibrate, "DOUBLE_SPHERE",
//             "What camera model do you want to calibrate. Options:"
//             "LINEAR_PINHOLE,DIVISION_UNDISTORTION,DOUBLE_SPHERE");
//DEFINE_double(downsample_factor, 2, "Downsample factor for images.");
//DEFINE_string(save_path_calib_dataset, "",
//             "Where to save the recon dataset to.");
//DEFINE_double(checker_size_m, 0.023,
//             "Size of one square on the checkerbaord in [m]. Needed to only "
//             "take far away poses!");
//DEFINE_double(grid_size, 0.04,
//             "Only take images that are at least grid_size apart");
//DEFINE_bool(is_stabelized, false, "Indicate if image was stablelized");
//DEFINE_bool(use_also_aruco_corners, false,
//           "Indicate if also aruco corners should be added");
//DEFINE_bool(verbose, false, "If more stuff should be printed");
//--input_video=/media/steffen/0F78151A1CEDE4A2/Sparsenet/CameraCalibrationStudy/GoPro6/1080_30/dataset1/cam/GH016354.MP4 --detector_params=/home/steffen/Projects/OpenCameraCalibrator/resource/charuco_detector_params.yml  --save_path_calib_dataset=/media/steffen/0F78151A1CEDE4A2/Sparsenet/SparsnetTests2020/GoPro6Calib1080NoStable3_30/ --camera_model_to_calibrate=DOUBLE_SPHERE --verbose=1

DEFINE_string(input_corners, "", "Path to save charuco board to.");
DEFINE_string(camera_model_to_calibrate, "DOUBLE_SPHERE",
              "What camera model do you want to calibrate. Options:"
              "LINEAR_PINHOLE,DIVISION_UNDISTORTION,DOUBLE_SPHERE");
DEFINE_string(save_path_calib_dataset, "",
              "Where to save the recon dataset to.");
DEFINE_double(grid_size, 0.04,
              "Only take images that are at least grid_size apart");
DEFINE_bool(verbose, false, "If more stuff should be printed");
//--input_corners=/media/steffen/0F78151A1CEDE4A2/Sparsenet/CameraCalibrationStudy/GoPro6/1080_30/dataset1/cam/GH016354_corners.uson --save_path_calib_dataset=/media/steffen/0F78151A1CEDE4A2/Sparsenet/SparsnetTests2020/GoPro6Calib1080NoStable3_30/ --camera_model_to_calibrate=DOUBLE_SPHERE --verbose=1
using Vector2D = Eigen::Vector2d;
using image_points_t =
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

int markerIdToTheiaId(const int marker_id, const int corner,
                      const int offset = 1000) {
  return (offset * marker_id) + offset + corner;
}

void scene_json_to_calib_dataset(
    const nlohmann::json &json, theia::Reconstruction &reconstruction,
    std::map<int, theia::TrackId> &charuco_id_to_theia_track_id) {

  // fill reconstruction with charuco points
  const auto scene_pts_it = json["scene_pts"];
  for (auto &it : scene_pts_it.items()) {
    theia::TrackId track_id = reconstruction.AddTrack();
    theia::Track *track = reconstruction.MutableTrack(track_id);
    track->SetEstimated(true);
    Eigen::Vector4d *point = track->MutablePoint();
    (*point)[0] = static_cast<double>(it.value()[0]);
    (*point)[1] = static_cast<double>(it.value()[1]);
    (*point)[2] = static_cast<double>(it.value()[2]);
    (*point)[3] = 1.0;
    charuco_id_to_theia_track_id[std::stoi(it.key())] = track_id;
  }
}

bool read_scene_bson(const std::string &input_bson,
                     nlohmann::json &scene_json) {
  std::ifstream input_corner_json(input_bson, std::ios::binary);
  if (!input_corner_json.is_open()) {
    std::cerr << "Can not open " << input_bson << "\n";
    return false;
  }
  // read the contents of the file into the vector
  std::uint8_t cont;
  std::vector<std::uint8_t> uson_file_content;
  while (input_corner_json.read(reinterpret_cast<char *>(&cont), sizeof(cont)))
    uson_file_content.push_back(cont);
  scene_json = nlohmann::json::from_ubjson(uson_file_content);
  input_corner_json.close();
  return true;
}

int main(int argc, char *argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  nlohmann::json scene_json;
  CHECK(read_scene_bson(FLAGS_input_corners, scene_json))
      << "Failed to load " << FLAGS_input_corners;

  theia::Reconstruction recon_calib_dataset;
  std::map<int, theia::TrackId> charuco_id_to_theia_track_id;
//  scene_json_to_calib_dataset(scene_json, recon_calib_dataset,
//                              charuco_id_to_theia_track_id);
  // fill reconstruction with charuco points
  const auto scene_pts_it = scene_json["scene_pts"];
  for (auto &it : scene_pts_it.items()) {
    theia::TrackId track_id = recon_calib_dataset.AddTrack();
    theia::Track *track = recon_calib_dataset.MutableTrack(track_id);
    track->SetEstimated(true);
    Eigen::Vector4d *point = track->MutablePoint();
    (*point)[0] = static_cast<double>(it.value()[0]);
    (*point)[1] = static_cast<double>(it.value()[1]);
    (*point)[2] = static_cast<double>(it.value()[2]);
    (*point)[3] = 1.0;
    charuco_id_to_theia_track_id[std::stoi(it.key())] = track_id;
  }
  theia::RansacParameters ransac_params;
  ransac_params.error_thresh = 0.1;
  ransac_params.failure_probability = 0.001;
  ransac_params.min_iterations = 30;
  ransac_params.use_mle = true;

  const int image_width = scene_json["image_width"];
  const int image_height = scene_json["image_height"];
  const double px = static_cast<double>(image_width) / 2.0;
  const double py = static_cast<double>(image_height) / 2.0;

  OpenCamCalib::aligned_vector<Eigen::Vector3d> saved_poses;
  std::map<theia::ViewId, double> ids_to_remove_after_init;

  // iterate views and estimate poses
  const auto views = scene_json["views"];
  for (const auto &view : views.items()) {
    const double timstamp_us = std::stod(view.key()); // to seconds
    const double timstamp_s = timstamp_us * 1e-6;     // to seconds
    const auto image_points = view.value()["image_points"];
    std::vector<int> board_pt3_ids;
    aligned_vector<Eigen::Vector2d> corners;
    for (const auto &img_pts : image_points.items()) {
      board_pt3_ids.push_back(std::stoi(img_pts.key()));
      corners.push_back(
          Eigen::Vector2d(img_pts.value()[0], img_pts.value()[1]));
    }

    std::cout << "timestamp: " << timstamp_s << "\n";
    // initialize cam pose
    std::vector<theia::FeatureCorrespondence2D3D> correspondences(
        board_pt3_ids.size());
    for (int i = 0; i < board_pt3_ids.size(); ++i) {
      theia::FeatureCorrespondence2D3D correspondence;
      correspondence.feature[0] = corners[i][0] - px;
      correspondence.feature[1] = corners[i][1] - py;
      theia::TrackId track_id =
          charuco_id_to_theia_track_id.find(board_pt3_ids[i])->second;
      const Eigen::Vector4d track =
          recon_calib_dataset.Track(track_id)->Point();
      correspondence.world_point = track.hnormalized();
      correspondences[i] = correspondence;
    }

    theia::RansacSummary ransac_summary;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d position;
    bool success_init = false;
    double focal_length, radial_distortion;
    if (FLAGS_camera_model_to_calibrate == "LINEAR_PINHOLE") {
      success_init = initialize_pinhole_camera(
          correspondences, ransac_params, ransac_summary, rotation, position,
          focal_length, FLAGS_verbose);
    } else if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION" ||
               FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE") {
      success_init = initialize_radial_undistortion_camera(
          correspondences, ransac_params, ransac_summary, image_width, rotation,
          position, focal_length, radial_distortion, FLAGS_verbose);
      if (FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE") {
        focal_length *= 0.5;
      }
    } else {
      std::cout << "THIS CAMERA MODEL (" << FLAGS_camera_model_to_calibrate
                << ") DOES NOT EXIST!\n";
      std::cout << "CHOOSE BETWEEN: LINEAR_PINHOLE, DIVISION_UNDISTORTION or "
                   "DOUBLE_SPHERE\n";
      return 0;
    }

    // check if a very close by pose is already present
    bool take_image = true;
    for (int i = 0; i < saved_poses.size(); ++i) {
      if ((position - saved_poses[i]).norm() < FLAGS_grid_size) {
        take_image = false;
        break;
      }
    }

    if (!take_image || !success_init) {
      continue;
    }

    saved_poses.push_back(position);

    // fill charucoCorners to theia reconstruction
    theia::ViewId view_id = recon_calib_dataset.AddView(
        std::to_string((uint64_t)timstamp_us), 0, timstamp_us);
    theia::View *theia_view = recon_calib_dataset.MutableView(view_id);
    theia_view->SetEstimated(true);

    theia::Camera *cam = theia_view->MutableCamera();
    cam->SetImageSize(image_width, image_height);
    cam->SetPrincipalPoint(image_width / 2.0, image_height / 2.0);
    cam->SetPosition(position);
    cam->SetOrientationFromRotationMatrix(rotation);
    cam->SetFocalLength(focal_length);
    if (FLAGS_camera_model_to_calibrate == "LINEAR_PINHOLE") {
      cam->SetCameraIntrinsicsModelType(
          theia::CameraIntrinsicsModelType::PINHOLE);
    } else if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION") {
      cam->SetCameraIntrinsicsModelType(
          theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION);
      cam->mutable_intrinsics()[theia::DivisionUndistortionCameraModel::
              InternalParametersIndex::RADIAL_DISTORTION_1] = radial_distortion;
    } else if (FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE") {
      cam->SetCameraIntrinsicsModelType(
          theia::CameraIntrinsicsModelType::DOUBLE_SPHERE);
      cam->mutable_intrinsics()[theia::DoubleSphereCameraModel::InternalParametersIndex::XI] = 1.0;
      cam->mutable_intrinsics()[theia::DoubleSphereCameraModel::InternalParametersIndex::ALPHA] = 0.0;
    }

    for (int i = 0; i < board_pt3_ids.size(); ++i) {
      theia::TrackId track_id =
          charuco_id_to_theia_track_id.find(board_pt3_ids[i])->second;
      recon_calib_dataset.AddObservation(view_id, track_id, corners[i]);
    }

    const double init_reproj_error =
        OpenCamCalib::utils::GetReprojErrorOfView(recon_calib_dataset, view_id);
    if (FLAGS_verbose) {
      std::cout << "View init reprojection error: " << init_reproj_error
                << std::endl;
    }
    if (init_reproj_error > 25.0) {
      ids_to_remove_after_init[view_id] = init_reproj_error;
    }
  }

  if (recon_calib_dataset.NumViews() < 10) {
    std::cout << "Not enough views left for proper calibration!" << std::endl;
    return 0;
  }

  std::cout << "Using " << recon_calib_dataset.NumViews()
            << " in bundle adjustment\n";
  // bundle adjust everything
  theia::BundleAdjustmentOptions ba_options;
  ba_options.fix_tracks = true;
  ba_options.verbose = true;
  ba_options.loss_function_type = theia::LossFunctionType::HUBER;
  ba_options.robust_loss_width = 1.345;

  /////////////////////////////////////////////////
  /// 1. Optimize focal length and radial distortion, keep principal point fixed
  /////////////////////////////////////////////////
  ba_options.constant_camera_orientation = false;
  ba_options.constant_camera_position = false;
  ba_options.intrinsics_to_optimize =
      theia::OptimizeIntrinsicsType::FOCAL_LENGTH;
  if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION" ||
      FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE")
    ba_options.intrinsics_to_optimize |=
        theia::OptimizeIntrinsicsType::RADIAL_DISTORTION;
  std::cout << "Bundle adjusting focal length and radial distortion. Keeping "
               "cam position fixed.\n";
  theia::BundleAdjustmentSummary summary =
      theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset);

  // reproj error per view, remove some views which have a high error
  std::map<theia::ViewId, double> ids_to_remove;
  for (int i = 0; i < recon_calib_dataset.NumViews(); ++i) {
    const theia::ViewId v_id = recon_calib_dataset.ViewIds()[i];
    const double view_reproj_error =
        OpenCamCalib::utils::GetReprojErrorOfView(recon_calib_dataset, v_id);
    if (view_reproj_error > 5.0) {
      ids_to_remove[v_id] = view_reproj_error;
    }
  }
  for (auto v_id : ids_to_remove) {
    recon_calib_dataset.RemoveView(v_id.first);
    std::cout << "Removed view: " << v_id.first
              << " with RMSE reproj error: " << v_id.second << "\n";
  }

  /////////////////////////////////////////////////
  /// 2. Optimize principal point keeping everything else fixed
  /////////////////////////////////////////////////
  ba_options.constant_camera_orientation = true;
  ba_options.constant_camera_position = true;
  ba_options.intrinsics_to_optimize =
      theia::OptimizeIntrinsicsType::PRINCIPAL_POINTS;

  summary = theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset);

  if (recon_calib_dataset.NumViews() < 8) {
    std::cout << "Not enough views left for proper calibration!" << std::endl;
    return 0;
  }

  /////////////////////////////////////////////////
  /// 3. Full reconstruction
  /////////////////////////////////////////////////
  ba_options.constant_camera_orientation = false;
  ba_options.constant_camera_position = false;
  ba_options.intrinsics_to_optimize =
      theia::OptimizeIntrinsicsType::PRINCIPAL_POINTS |
      theia::OptimizeIntrinsicsType::FOCAL_LENGTH;
  if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION" ||
      FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE")
    ba_options.intrinsics_to_optimize |=
        theia::OptimizeIntrinsicsType::RADIAL_DISTORTION;
  summary = theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset);

  OpenCamCalib::utils::PrintResult(FLAGS_camera_model_to_calibrate,
                                   recon_calib_dataset);

  theia::WriteReconstruction(recon_calib_dataset,
                             FLAGS_save_path_calib_dataset + ".calibdata");
  nlohmann::json json;

  const theia::Camera cam =
      recon_calib_dataset.View(recon_calib_dataset.ViewIds()[0])->Camera();
  const double *intrinsics = cam.intrinsics();

  // final reprojection error
  double reproj_error = 0;
  for (int i = 0; i < recon_calib_dataset.NumViews(); ++i) {
    const double view_reproj_error = OpenCamCalib::utils::GetReprojErrorOfView(
        recon_calib_dataset, recon_calib_dataset.ViewIds()[i]);
    reproj_error += view_reproj_error;
    if (FLAGS_verbose) {
      std::cout << "View: " << recon_calib_dataset.ViewIds()[i]
                << " RMSE reprojection error: " << view_reproj_error << "\n";
    }
  }
  const double total_repro_error =
      reproj_error / recon_calib_dataset.NumViews();
  std::cout << "Final camera calibration reprojection error: "
            << total_repro_error << " from " << recon_calib_dataset.NumViews()
            << " view." << std::endl;
  json["stabelized"] = false;
  json["fps"] = scene_json["camera_fps"];
  json["nr_calib_images"] = recon_calib_dataset.NumViews();
  json["final_ba_cost"] = summary.final_cost;
  json["final_reproj_error"] = total_repro_error;
  json["intrinsics"]["focal_length"] = cam.FocalLength();
  json["intrinsics"]["principal_pt_x"] = cam.PrincipalPointX();
  json["intrinsics"]["principal_pt_y"] = cam.PrincipalPointY();
  json["image_width"] = cam.ImageWidth();
  json["image_height"] = cam.ImageHeight();
  json["intrinsics"]["skew"] = 0.0;

  if (FLAGS_camera_model_to_calibrate == "LINEAR_PINHOLE") {
    json["intrinsics"]["aspect_ratio"] = intrinsics
        [theia::PinholeCameraModel::InternalParametersIndex::ASPECT_RATIO];
    json["intrinsic_type"]["camera_type"] =
        OpenCamCalib::utils::CameraIDToString(
            (int)theia::CameraIntrinsicsModelType::PINHOLE);
  } else if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION") {
    json["intrinsics"]["aspect_ratio"] =
        intrinsics[theia::DivisionUndistortionCameraModel::
                       InternalParametersIndex::ASPECT_RATIO];
    json["intrinsic_type"]["camera_type"] =
        OpenCamCalib::utils::CameraIDToString(
            (int)theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION);
    json["intrinsic_type"]["div_undist_distortion"] =
        intrinsics[theia::DivisionUndistortionCameraModel::
                       InternalParametersIndex::RADIAL_DISTORTION_1];
  } else if (FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE") {
    json["intrinsics"]["aspect_ratio"] = intrinsics
        [theia::DoubleSphereCameraModel::InternalParametersIndex::ASPECT_RATIO];
    json["intrinsic_type"]["camera_type"] =
        OpenCamCalib::utils::CameraIDToString(
            (int)theia::CameraIntrinsicsModelType::DOUBLE_SPHERE);
    json["intrinsic_type"]["xi"] =
        intrinsics[theia::DoubleSphereCameraModel::InternalParametersIndex::XI];
    json["intrinsic_type"]["alpha"] = intrinsics
        [theia::DoubleSphereCameraModel::InternalParametersIndex::ALPHA];
  }
  std::ofstream calib_txt_output(FLAGS_save_path_calib_dataset + ".json");
  calib_txt_output << std::setw(4) << json << std::endl;

  return 0;
}

// int main(int argc, char *argv[]) {
//  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

//  int squaresX = 10;
//  int squaresY = 8;
//  float squareLength = FLAGS_checker_size_m;
//  float markerLength = FLAGS_checker_size_m / 2.0;
//  int dictionaryId = cv::aruco::DICT_ARUCO_ORIGINAL;
//  const int min_number_detected_corners = 30;

//  theia::RansacParameters ransac_params;
//  ransac_params.error_thresh = 0.1;
//  ransac_params.failure_probability = 0.001;
//  ransac_params.min_iterations = 30;
//  ransac_params.use_mle = true;

//  theia::Reconstruction recon_calib_dataset;

//  // load images from folder
//  Ptr<aruco::DetectorParameters> detectorParams =
//      aruco::DetectorParameters::create();

//  if (!OpenCamCalib::utils::ReadDetectorParameters(FLAGS_detector_params,
//                                                   detectorParams)) {
//    std::cerr << "Invalid detector parameters file\n";
//    return 0;
//  }

//  Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(
//      aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
//  // create charuco board object
//  Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(
//      squaresX, squaresY, squareLength, markerLength, dictionary);
//  Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

//  // fill reconstruction with charuco points
//  std::vector<cv::Point3f> chessoard3d = charucoboard->chessboardCorners;
//  std::map<int, theia::TrackId> charuco_id_to_theia_track_id;
//  // fill charuco markers
//  for (size_t i = 0; i < chessoard3d.size(); ++i) {
//    theia::TrackId track_id = recon_calib_dataset.AddTrack();
//    theia::Track *track = recon_calib_dataset.MutableTrack(track_id);
//    track->SetEstimated(true);
//    Eigen::Vector4d *point = track->MutablePoint();
//    (*point)[0] = static_cast<double>(chessoard3d[i].x);
//    (*point)[1] = static_cast<double>(chessoard3d[i].y);
//    (*point)[2] = static_cast<double>(chessoard3d[i].z);
//    (*point)[3] = 1.0;
//    charuco_id_to_theia_track_id[i] = track_id;
//  }

//  if (FLAGS_use_also_aruco_corners) {
//    std::vector<std::vector<cv::Point3f>> arucoBoard3d =
//        charucoboard->objPoints;
//    // fill aruco markers
//    for (size_t i = 0; i < arucoBoard3d.size(); ++i) {
//      for (int j = 0; j < 4; ++j) {
//        theia::TrackId track_id = recon_calib_dataset.AddTrack();
//        theia::Track *track = recon_calib_dataset.MutableTrack(track_id);
//        track->SetEstimated(true);
//        Eigen::Vector4d *point = track->MutablePoint();
//        (*point)[0] = static_cast<double>(arucoBoard3d[i][j].x);
//        (*point)[1] = static_cast<double>(arucoBoard3d[i][j].y);
//        (*point)[2] = static_cast<double>(arucoBoard3d[i][j].z);
//        (*point)[3] = 1.0;
//        charuco_id_to_theia_track_id[markerIdToTheiaId(charucoboard->ids[i],
//                                                       j)] = track_id;
//      }
//    }
//  }

//  VideoCapture inputVideo;
//  inputVideo.open(FLAGS_input_video);
//  bool showRejected = false;
//  int cnt_wrong = 0;
//  double fps = inputVideo.get(cv::CAP_PROP_FPS);

//  OpenCamCalib::aligned_vector<Eigen::Vector3d> saved_poses;

//  int frame_cnt = 0;
//  double minMarkerLengthRatioOriginalImg = 0.0;
//  std::map<theia::ViewId, double> ids_to_remove_after_init;
//  while (true) {
//    Mat image, imageCopy;
//    if (!inputVideo.read(image)) {
//      cnt_wrong++;
//      if (cnt_wrong > 500)
//        break;
//      continue;
//    }
//    std::string timestamp =
//        std::to_string(inputVideo.get(cv::CAP_PROP_POS_MSEC) / 1000.0);
//    ++frame_cnt;

//    cv::resize(image, image, cv::Size(), 1. / FLAGS_downsample_factor,
//               1. / FLAGS_downsample_factor);

//    std::vector<int> markerIds, charucoIds;
//    std::vector<std::vector<Point2f>> markerCorners, rejectedMarkers;
//    std::vector<Point2f> charucoCorners;

//    // detect markers
//    detectorParams->minMarkerLengthRatioOriginalImg =
//        minMarkerLengthRatioOriginalImg;
//    //    if (FLAGS_verbose) {
//    //        std::cout<<"minMarkerLengthRatioOriginalImg:
//    //        "<<minMarkerLengthRatioOriginalImg<<std::endl;
//    //    }
//    minMarkerLengthRatioOriginalImg =
//        aruco::detectMarkers(image, dictionary, markerCorners, markerIds,
//                             detectorParams, rejectedMarkers);

//    // refind strategy to detect more markers
//    aruco::refineDetectedMarkers(image, board, markerCorners, markerIds,
//                                 rejectedMarkers);

//    // interpolate charuco corners
//    int interpolatedCorners = 0;
//    if (markerIds.size() > 0)
//      interpolatedCorners = aruco::interpolateCornersCharuco(
//          markerCorners, markerIds, image, charucoboard, charucoCorners,
//          charucoIds);

//    if (FLAGS_use_also_aruco_corners) {
//      if (markerIds.size() < min_number_detected_corners)
//        continue;
//    } else if (charucoIds.size() + 4 * markerIds.size() <
//               min_number_detected_corners) {
//      continue;
//    }

//    // draw results
//    if (FLAGS_verbose) {
//      image.copyTo(imageCopy);
//      if (markerIds.size() > 0) {
//        aruco::drawDetectedMarkers(imageCopy, markerCorners);
//      }

//      if (showRejected && rejectedMarkers.size() > 0)
//        aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(),
//                                   Scalar(100, 0, 255));

//      if (interpolatedCorners > 0) {
//        Scalar color;
//        color = Scalar(255, 0, 0);
//        aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners,
//        charucoIds,
//                                          color);
//      }
//    }
//    // Estimate pose
//    double px = static_cast<double>(image.cols) / 2.0;
//    double py = static_cast<double>(image.rows) / 2.0;
//    std::vector<theia::FeatureCorrespondence2D3D> correspondences(
//        charucoIds.size());
//    for (int i = 0; i < charucoIds.size(); ++i) {
//      theia::FeatureCorrespondence2D3D correspondence;
//      correspondence.feature << (static_cast<double>(charucoCorners[i].x) -
//      px),
//          (static_cast<double>(charucoCorners[i].y) - py);
//      theia::TrackId track_id =
//          charuco_id_to_theia_track_id.find(charucoIds[i])->second;
//      const Eigen::Vector4d track =
//          recon_calib_dataset.Track(track_id)->Point();
//      correspondence.world_point = track.hnormalized();
//      correspondences[i] = correspondence;
//    }

//    if (FLAGS_use_also_aruco_corners) {
//      for (int i = 0; i < markerIds.size(); ++i) {
//        for (int j = 0; j < 4; ++j) {
//          theia::FeatureCorrespondence2D3D correspondence;
//          correspondence.feature
//              << (static_cast<double>(markerCorners[i][j].x) - px),
//              (static_cast<double>(markerCorners[i][j].y) - py);
//          const int marker_id_converted = markerIdToTheiaId(markerIds[i], j);
//          theia::TrackId track_id =
//              charuco_id_to_theia_track_id.find(marker_id_converted)->second;
//          const Eigen::Vector4d track =
//              recon_calib_dataset.Track(track_id)->Point();
//          correspondence.world_point = track.hnormalized();
//          correspondences[i] = correspondence;
//        }
//      }
//    }

//    theia::RansacSummary ransac_summary;
//    Eigen::Matrix3d rotation;
//    Eigen::Vector3d position;
//    bool success_init = false;
//    double focal_length, radial_distortion;
//    if (FLAGS_camera_model_to_calibrate == "LINEAR_PINHOLE") {
//      success_init = OpenCamCalib::initialize_pinhole_camera(
//          correspondences, ransac_params, ransac_summary, rotation, position,
//          focal_length, FLAGS_verbose);
//    } else if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION" ||
//               FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE") {
//      success_init = OpenCamCalib::initialize_radial_undistortion_camera(
//          correspondences, ransac_params, ransac_summary, image.cols,
//          rotation, position, focal_length, radial_distortion, FLAGS_verbose);
//      if (FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE") {
//        focal_length *= 0.5;
//      }
//    } else {
//      std::cout << "THIS CAMERA MODEL (" << FLAGS_camera_model_to_calibrate
//                << ") DOES NOT EXIST!\n";
//      std::cout << "CHOOSE BETWEEN: LINEAR_PINHOLE, DIVISION_UNDISTORTION or "
//                   "DOUBLE_SPHERE\n";
//      return 0;
//    }

//    // check if a very close by pose is already present
//    bool take_image = true;
//    for (int i = 0; i < saved_poses.size(); ++i) {
//      if ((position - saved_poses[i]).norm() < FLAGS_grid_size) {
//        take_image = false;
//        break;
//      }
//    }

//    if (!take_image || !success_init) {
//      continue;
//    }

//    if (FLAGS_use_also_aruco_corners) {
//      if (ransac_summary.inliers.size() <
//          (charucoIds.size() + 4 * markerIds.size()) * 0.6) {
//        continue;
//      }
//    } else if (ransac_summary.inliers.size() < charucoIds.size() * 0.6) {
//      continue;
//    }

//    saved_poses.push_back(position);

//    // fill charucoCorners to theia reconstruction
//    theia::ViewId view_id =
//        recon_calib_dataset.AddView(timestamp, 0, std::stod(timestamp));
//    theia::View *view = recon_calib_dataset.MutableView(view_id);
//    view->SetEstimated(true);
////    std::cout<<"view_id: "<<view_id<<std::endl;
//    theia::Camera *cam = view->MutableCamera();
//    cam->SetImageSize(image.cols, image.rows);
//    cam->SetPrincipalPoint(image.cols / 2.0, image.rows / 2.0);
//    cam->SetPosition(position);
//    cam->SetOrientationFromRotationMatrix(rotation);
//    if (FLAGS_camera_model_to_calibrate == "LINEAR_PINHOLE") {
//      cam->SetCameraIntrinsicsModelType(
//          theia::CameraIntrinsicsModelType::PINHOLE);
//      cam->SetFocalLength(focal_length);
//    } else if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION") {
//      cam->SetCameraIntrinsicsModelType(
//          theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION);
//      cam->SetFocalLength(0.5*focal_length);
//      cam->mutable_intrinsics()[theia::DivisionUndistortionCameraModel::
//              InternalParametersIndex::RADIAL_DISTORTION_1] = radial_distortion;
//    } else if (FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE") {
//      cam->SetCameraIntrinsicsModelType(
//          theia::CameraIntrinsicsModelType::DOUBLE_SPHERE);
//      cam->SetFocalLength(0.5*focal_length);
//      cam->mutable_intrinsics()[theia::DoubleSphereCameraModel::InternalParametersIndex::XI] = 1.0;
//      cam->mutable_intrinsics()[theia::DoubleSphereCameraModel::InternalParametersIndex::ALPHA] = 0.0;
//    }

//    for (int i = 0; i < charucoIds.size(); ++i) {
//      theia::TrackId track_id =
//          charuco_id_to_theia_track_id.find(charucoIds[i])->second;
//      theia::Feature feature;
//      feature << static_cast<double>(charucoCorners[i].x),
//          static_cast<double>(charucoCorners[i].y);
//      recon_calib_dataset.AddObservation(view_id, track_id, feature);
//    }
////    if (FLAGS_use_also_aruco_corners) {
////      for (int i = 0; i < markerIds.size(); ++i) {
////        for (int j = 0; j < 4; ++j) {
////          const int marker_id_converted = markerIdToTheiaId(markerIds[i], j);
////          theia::TrackId track_id =
////              charuco_id_to_theia_track_id.find(marker_id_converted)->second;
////          theia::Feature feature;
////          feature << static_cast<double>(markerCorners[i][j].x),
////              static_cast<double>(markerCorners[i][j].y);
////          recon_calib_dataset.AddObservation(view_id, track_id, feature);
////        }
////      }
////    }

////    const double init_reproj_error =
////        OpenCamCalib::utils::GetReprojErrorOfView(recon_calib_dataset,
////        view_id);
////    if (FLAGS_verbose) {
////      std::cout << "View init reprojection error: " << init_reproj_error
////                << std::endl;
////    }
////    if (init_reproj_error > 25.0) {
////      ids_to_remove_after_init[view_id] = init_reproj_error;
////    }
////    if (FLAGS_verbose) {
////      imshow("out", imageCopy);
////      char key = (char)waitKey(1);
////      if (key == 27)
////        break;
////    }
//  }

//  if (recon_calib_dataset.NumViews() < 10) {
//    std::cout << "Not enough views left for proper calibration!" << std::endl;
//    return 0;
//  }

//  std::cout << "Using " << recon_calib_dataset.NumViews()
//            << " in bundle adjustment\n";
//  // bundle adjust everything
//  theia::BundleAdjustmentOptions ba_options;
//  ba_options.fix_tracks = true;
//  ba_options.verbose = true;
//  ba_options.loss_function_type = theia::LossFunctionType::HUBER;
//  ba_options.robust_loss_width = 1.345;

//  /////////////////////////////////////////////////
//  /// 1. Optimize focal length and radial distortion, keep principal point
//  /////////////////////////////////////////////////
//  ba_options.constant_camera_orientation = false;
//  ba_options.constant_camera_position = false;
//  ba_options.intrinsics_to_optimize =
//      theia::OptimizeIntrinsicsType::FOCAL_LENGTH;
//  if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION" ||
//      FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE")
//    ba_options.intrinsics_to_optimize |=
//        theia::OptimizeIntrinsicsType::RADIAL_DISTORTION;
//  std::cout << "Bundle adjusting focal length and radial distortion. Keeping "
//               "cam position fixed.\n";
//  theia::BundleAdjustmentSummary summary =
//      theia::BundleAdjustReconstruction(ba_options, &recon_calib_dataset);

//  // reproj error per view, remove some views which have a high error
//  std::map<theia::ViewId, double> ids_to_remove;
//  for (int i = 0; i < recon_calib_dataset.NumViews(); ++i) {
//    const theia::ViewId v_id = recon_calib_dataset.ViewIds()[i];
//    const double view_reproj_error =
//        OpenCamCalib::utils::GetReprojErrorOfView(recon_calib_dataset, v_id);
//    if (view_reproj_error > 5.0) {
//      ids_to_remove[v_id] = view_reproj_error;
//    }
//  }
//  for (auto v_id : ids_to_remove) {
//    recon_calib_dataset.RemoveView(v_id.first);
//    std::cout << "Removed view: " << v_id.first
//              << " with RMSE reproj error: " << v_id.second << "\n";
//  }

//  /////////////////////////////////////////////////
//  /// 2. Optimize principal point keeping everything else fixed
//  /////////////////////////////////////////////////
//  ba_options.constant_camera_orientation = true;
//  ba_options.constant_camera_position = true;
//  ba_options.intrinsics_to_optimize =
//      theia::OptimizeIntrinsicsType::PRINCIPAL_POINTS;

//  summary = theia::BundleAdjustReconstruction(ba_options,
//  &recon_calib_dataset);

//  if (recon_calib_dataset.NumViews() < 8) {
//    std::cout << "Not enough views left for proper calibration!" << std::endl;
//    return 0;
//  }

//  /////////////////////////////////////////////////
//  /// 3. Full reconstruction
//  /////////////////////////////////////////////////
//  ba_options.constant_camera_orientation = false;
//  ba_options.constant_camera_position = false;
//  ba_options.intrinsics_to_optimize =
//      theia::OptimizeIntrinsicsType::PRINCIPAL_POINTS |
//      theia::OptimizeIntrinsicsType::FOCAL_LENGTH;
//  if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION" ||
//      FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE")
//    ba_options.intrinsics_to_optimize |=
//        theia::OptimizeIntrinsicsType::RADIAL_DISTORTION;
//  summary = theia::BundleAdjustReconstruction(ba_options,
//  &recon_calib_dataset);

//  OpenCamCalib::utils::PrintResult(FLAGS_camera_model_to_calibrate,
//                                   recon_calib_dataset);

//  theia::WriteReconstruction(recon_calib_dataset,
//                             FLAGS_save_path_calib_dataset + ".calibdata");
//  nlohmann::json json;

//  const theia::Camera cam =
//      recon_calib_dataset.View(recon_calib_dataset.ViewIds()[0])->Camera();
//  const double *intrinsics = cam.intrinsics();

//  // final reprojection error
//  double reproj_error = 0;
//  for (int i = 0; i < recon_calib_dataset.NumViews(); ++i) {
//    const double view_reproj_error =
//    OpenCamCalib::utils::GetReprojErrorOfView(
//        recon_calib_dataset, recon_calib_dataset.ViewIds()[i]);
//    reproj_error += view_reproj_error;
//    if (FLAGS_verbose) {
//      std::cout << "View: " << recon_calib_dataset.ViewIds()[i]
//                << " RMSE reprojection error: " << view_reproj_error << "\n";
//    }
//  }
//  const double total_repro_error =
//      reproj_error / recon_calib_dataset.NumViews();
//  std::cout << "Final camera calibration reprojection error: "
//            << total_repro_error << " from " << recon_calib_dataset.NumViews()
//            << " view." << std::endl;
//  json["stabelized"] = FLAGS_is_stabelized;
//  json["fps"] = fps;
//  json["nr_images_used"] = recon_calib_dataset.NumViews();
//  json["final_ba_cost"] = summary.final_cost;
//  json["final_reproj_error"] = total_repro_error;
//  json["intrinsics"]["focal_length"] = cam.FocalLength();
//  json["intrinsics"]["principal_pt_x"] = cam.PrincipalPointX();
//  json["intrinsics"]["principal_pt_y"] = cam.PrincipalPointY();
//  json["image_width"] = cam.ImageWidth();
//  json["image_height"] = cam.ImageHeight();
//  json["intrinsics"]["skew"] = 0.0;

//  if (FLAGS_camera_model_to_calibrate == "LINEAR_PINHOLE") {
//    json["intrinsics"]["aspect_ratio"] = intrinsics
//        [theia::PinholeCameraModel::InternalParametersIndex::ASPECT_RATIO];
//    json["intrinsic_type"]["camera_type"] =
//        OpenCamCalib::utils::CameraIDToString(
//            (int)theia::CameraIntrinsicsModelType::PINHOLE);
//  } else if (FLAGS_camera_model_to_calibrate == "DIVISION_UNDISTORTION") {
//    json["intrinsics"]["aspect_ratio"] =
//        intrinsics[theia::DivisionUndistortionCameraModel::
//                       InternalParametersIndex::ASPECT_RATIO];
//    json["intrinsic_type"]["camera_type"] =
//        OpenCamCalib::utils::CameraIDToString(
//            (int)theia::CameraIntrinsicsModelType::DIVISION_UNDISTORTION);
//    json["intrinsic_type"]["div_undist_distortion"] =
//        intrinsics[theia::DivisionUndistortionCameraModel::
//                       InternalParametersIndex::RADIAL_DISTORTION_1];
//  } else if (FLAGS_camera_model_to_calibrate == "DOUBLE_SPHERE") {
//    json["intrinsics"]["aspect_ratio"] = intrinsics
//        [theia::DoubleSphereCameraModel::InternalParametersIndex::ASPECT_RATIO];
//    json["intrinsic_type"]["camera_type"] =
//        OpenCamCalib::utils::CameraIDToString(
//            (int)theia::CameraIntrinsicsModelType::DOUBLE_SPHERE);
//    json["intrinsic_type"]["xi"] =
//        intrinsics[theia::DoubleSphereCameraModel::InternalParametersIndex::XI];
//    json["intrinsic_type"]["alpha"] = intrinsics
//        [theia::DoubleSphereCameraModel::InternalParametersIndex::ALPHA];
//  }
//  std::ofstream calib_txt_output(FLAGS_save_path_calib_dataset + ".json");
//  calib_txt_output << std::setw(4) << json << std::endl;

//  return 0;
//}
