/* Copyright (C) 2021 Steffen Urban
 * All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include <gflags/gflags.h>
#include <string>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>

// Input/output files.
DEFINE_string(save_charuco_image, "", "Path to save charuco board to.");
DEFINE_int32(squaresX, 9, "squares in X.");
DEFINE_int32(squaresY, 7, "squares in Y.");
DEFINE_int32(squareLength, 300, "squareLength.");
DEFINE_int32(markerLength, 150, "markerLength.");

int main(int argc, char* argv[]) {
  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  int squaresX = FLAGS_squaresX;
  int squaresY = FLAGS_squaresY;
  int squareLength = FLAGS_squareLength;
  int markerLength = FLAGS_markerLength;
  int dictionaryId = cv::aruco::DICT_4X4_1000;
  int margins = squareLength - markerLength;
  int borderBits = 1;
  bool showImage = false;

  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(
          cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

  cv::Size imageSize;
  imageSize.width = squaresX * squareLength + 2 * margins;
  imageSize.height = squaresY * squareLength + 2 * margins;

  cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(
      squaresX, squaresY, (float)squareLength, (float)markerLength, dictionary);

  // show created board
  cv::Mat boardImage;
  board->draw(imageSize, boardImage, margins, borderBits);

  if (showImage) {
    cv::imshow("board", boardImage);
    cv::waitKey(0);
  }
  const std::string save_string = "board_" + std::to_string(squaresX) + "x" +
                                  std::to_string(squaresY) + ".jpg";
  std::cout << FLAGS_save_charuco_image + save_string << std::endl; 
  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  compression_params.push_back(100);
  cv::imwrite(FLAGS_save_charuco_image + "/" + save_string, boardImage, compression_params);

  return 0;
}
