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

int main(int argc, char *argv[]) {

  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  int squaresX = 9;
  int squaresY = 7;
  int squareLength = 300;
  int markerLength = 100;
  int dictionaryId = cv::aruco::DICT_ARUCO_ORIGINAL;
  int margins = squareLength - markerLength;
  int borderBits = 1;
  bool showImage = true;

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
                                  std::to_string(squaresY) + ".png";
  std::cout << FLAGS_save_charuco_image + save_string << std::endl;
  cv::imwrite(FLAGS_save_charuco_image + "/" + save_string, boardImage);

  return 0;
}
