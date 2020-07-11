#include <dirent.h>
#include <gflags/gflags.h>
#include <time.h>
#include <algorithm>
#include <chrono>  // NOLINT
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>

#include "OpenCameraCalibrator/imu/read_gopro_imu_json.h"
#include "OpenCameraCalibrator/utils/types.h"
#include "OpenCameraCalibrator/utils/utils.h"

#include "theia/io/reconstruction_reader.h"
#include "theia/io/reconstruction_writer.h"
#include "theia/sfm/reconstruction.h"
#include <theia/sfm/camera/division_undistortion_camera_model.h>
#include <theia/sfm/camera/pinhole_camera_model.h>

// Input/output files.
DEFINE_string(
    gopro_telemetry_json, "",
    "Path to gopro telemetry json extracted with Sparsnet extractor.");
DEFINE_string(input_video, "", "Path to corresponding video file.");
DEFINE_string(detector_params, "", "Path detector yaml.");
DEFINE_string(input_calibration_dataset, "",
              "Path to input calibration dataset.");
DEFINE_string(output_calibration_dataset, "",
              "Path to write the pose calibration dataset to.");
DEFINE_double(checker_size_m, 0.039,
              "Size of one square on the checkerbaord in [m].");
DEFINE_string(output_path_txt_files_for_testing, "",
              "Output files for scale estimator.");

void UndistortImagePoints(const theia::Camera& theia_camera,
                          const std::vector<cv::Point2f>& points,
                          std::vector<cv::Point2f>& undistorted_points) {
    undistorted_points.resize(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        Eigen::Vector2d pt;
        pt << points[i].x, points[i].y;
        Eigen::Vector3d undist_pt = theia_camera.PixelToNormalizedCoordinates(pt);
        undist_pt /= undist_pt[2];
        undistorted_points[i].x = undist_pt[0];
        undistorted_points[i].y = undist_pt[1];
    }
}

int main(int argc, char* argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  // Load camera calibration reconstuction.
  theia::Reconstruction cam_calib_recon;
  CHECK(theia::ReadReconstruction(FLAGS_input_calibration_dataset,
                                  &cam_calib_recon))
      << "Could not read calibration reconstruction file.";
  // get one view for calibration info
  const theia::Camera camera = cam_calib_recon.View(0)->Camera();
  cv::Mat K_cv = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);

  int squaresX = 10;
  int squaresY = 8;
  float squareLength = FLAGS_checker_size_m;
  float markerLength = FLAGS_checker_size_m / 2.0;
  int dictionaryId = cv::aruco::DICT_ARUCO_ORIGINAL;
  const int min_number_detected_corners = 2;

  // set charuco detector parameters
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams =
      cv::aruco::DetectorParameters::create();
  if (!OpenCamCalib::ReadDetectorParameters(FLAGS_detector_params,
                                            detectorParams)) {
    std::cerr << "Invalid detector parameters file\n";
    return 0;
  }

  // initialize an aruco board
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(
      cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
  // create charuco board object
  cv::Ptr<cv::aruco::CharucoBoard> charucoboard = cv::aruco::CharucoBoard::create(
      squaresX, squaresY, squareLength, markerLength, dictionary);
  cv::Ptr<cv::aruco::Board> board = charucoboard.staticCast<cv::aruco::Board>();

  theia::Reconstruction pose_dataset;

  // add the 3D points to the reconstruction as fixed points
  std::vector<cv::Point3f> chessoard3d = charucoboard->chessboardCorners;
  std::map<int, theia::TrackId> charuco_id_to_theia_track_id;
  for (int i = 0; i < chessoard3d.size(); ++i) {
    theia::TrackId track_id = pose_dataset.AddTrack();
    theia::Track* track = pose_dataset.MutableTrack(track_id);
    track->SetEstimated(true);
    Eigen::Vector4d* point = track->MutablePoint();
    (*point)[0] = static_cast<double>(chessoard3d[i].x);
    (*point)[1] = static_cast<double>(chessoard3d[i].y);
    (*point)[2] = static_cast<double>(chessoard3d[i].z);
    (*point)[3] = 1.0;
    charuco_id_to_theia_track_id[i] = track_id;
  }

  // init the video stream
  cv::VideoCapture inputVideo;
  inputVideo.open(FLAGS_input_video);
  bool show_rejected = false;
  int cnt_wrong = 0;
  int frame_cnt = 0;

  // start processing the video stream and estimating poses
  while (true) {
    cv::Mat image, imageCopy;
    if (!inputVideo.read(image)) {
      cnt_wrong++;
      if (cnt_wrong > 200) break;
      continue;
    }
    const double timestamp_ = inputVideo.get(cv::CAP_PROP_POS_MSEC) / 1000.0;
    std::string timestamp_s = std::to_string(timestamp_);
    ++frame_cnt;

    cv::resize(image, image,
               cv::Size(camera.ImageWidth(), camera.ImageHeight()));

    std::vector<int> marker_ids, charuco_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_markers;
    std::vector<cv::Point2f> charuco_corners, undistorted_charuco_corners;

    // detect markers
    cv::aruco::detectMarkers(
                image, dictionary, marker_corners,
                marker_ids, detectorParams, rejected_markers);

    // refind strategy to detect more markers
    cv::aruco::refineDetectedMarkers(
                image, board, marker_corners, marker_ids, rejected_markers);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if (marker_ids.size() > 0) {
      interpolatedCorners = cv::aruco::interpolateCornersCharuco(
          marker_corners, marker_ids, image,
          charucoboard, charuco_corners, charuco_ids);
    }

    if (charuco_ids.size() < min_number_detected_corners) {
        continue;
    }

    // draw results
    image.copyTo(imageCopy);
    if (marker_ids.size() > 0) {
      cv::aruco::drawDetectedMarkers(imageCopy, marker_corners);
    }

    if (show_rejected && rejected_markers.size() > 0)
      cv::aruco::drawDetectedMarkers(imageCopy, rejected_markers, cv::noArray(),
                                 cv::Scalar(100, 0, 255));
    if (interpolatedCorners < 10)
        continue;


    cv::Scalar color = cv::Scalar(255, 0, 0);
    cv::aruco::drawDetectedCornersCharuco(imageCopy, charuco_corners, charuco_ids, color);

    // estimate camera poses, therefore the detected corners need to be undistorted first
    UndistortImagePoints(camera, charuco_corners, undistorted_charuco_corners);
    cv::Mat rvec, tvec;
    if(!cv::aruco::estimatePoseCharucoBoard(
                undistorted_charuco_corners, charuco_ids, charucoboard,
                K_cv, cv::Mat(), rvec, tvec)) {
        std::cout<<"Could not estimate pose for timestamp: "<<timestamp_<<"\n";
        continue;
    }
    Eigen::Vector3d axis;
    axis << rvec.at<double>(0),rvec.at<double>(1),rvec.at<double>(2);
    axis.normalize();
    Eigen::Vector3d pos;
    pos << tvec.at<double>(0),tvec.at<double>(1),tvec.at<double>(2);
    Eigen::AngleAxisd angle_axis(cv::norm(rvec), axis);
    //Eigen::Quaterniond EigenQuat(angle_axis.toRotationMatrix());
    pos = -angle_axis.toRotationMatrix().transpose()*pos;

    theia::ViewId view_id = pose_dataset.AddView(timestamp_s, 0);
    theia::View* view = pose_dataset.MutableView(view_id);
    view->SetEstimated(true);
    theia::Camera* cam = view->MutableCamera();
    cam->SetImageSize(camera.ImageWidth(), camera.ImageHeight());
    cam->DeepCopy(camera); // this also copies extrinsics so we need to set them afterwards
    cam->SetOrientationFromRotationMatrix(angle_axis.toRotationMatrix());
    cam->SetPosition(pos);


    for (int i = 0; i < charuco_ids.size(); ++i) {
      theia::TrackId track_id =
          charuco_id_to_theia_track_id.find(charuco_ids[i])->second;
      theia::Feature feature;
      feature << static_cast<double>(charuco_corners[i].x),
                 static_cast<double>(charuco_corners[i].y);
      pose_dataset.AddObservation(view_id, track_id, feature);
    }

    // test back projection
    cv::Mat back_proj;
    image.copyTo(back_proj);
    double reproj_error = 0;
    for (int i = 0; i < charuco_ids.size(); ++i) {
      theia::TrackId track_id =
          charuco_id_to_theia_track_id.find(charuco_ids[i])->second;
      const theia::Track* track = pose_dataset.Track(track_id);
      Eigen::Vector2d pt;
      cam->ProjectPoint(track->Point(), &pt);

      cv::drawMarker(back_proj, cv::Point(pt[0],pt[1]), color);
      theia::Feature feature;
      feature << static_cast<double>(charuco_corners[i].x),
                 static_cast<double>(charuco_corners[i].y);
      reproj_error += (pt-feature).norm();
    }
    const std::string plt_text = "Reprojection error: "+
            std::to_string(reproj_error / charuco_ids.size())+" pixels.";
    cv::putText(back_proj,plt_text, cv::Point(50,50),
                cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255,255,255), 2);

    cv::imshow("bbackprojected", back_proj);
    cv::waitKey(1);
  }

  theia::WriteReconstruction(pose_dataset, FLAGS_output_calibration_dataset);

  return 0;
}
