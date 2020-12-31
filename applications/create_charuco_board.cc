#include <dirent.h>
#include <gflags/gflags.h>
#include <time.h>
#include <algorithm>
#include <chrono>  // NOLINT
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>



// Input/output files.
DEFINE_string(save_charuco_image, "", "Path to save charuco board to.");

using namespace cv;
int main(int argc, char* argv[]) {

  GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  int squaresX = 9;
  int squaresY = 7;
  int squareLength = 300;
  int markerLength = 100;
  int dictionaryId = cv::aruco::DICT_ARUCO_ORIGINAL;
  int margins = squareLength - markerLength;
  int borderBits = 1;
  bool showImage = true;

  Ptr<aruco::Dictionary> dictionary =
      aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

  Size imageSize;
  imageSize.width = squaresX * squareLength + 2 * margins;
  imageSize.height = squaresY * squareLength + 2 * margins;

  Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(squaresX, squaresY, (float)squareLength,
                                                          (float)markerLength, dictionary);

  // show created board
  Mat boardImage;
  board->draw(imageSize, boardImage, margins, borderBits);

  if(showImage) {
      imshow("board", boardImage);
      waitKey(0);
  }

  std::cout<<FLAGS_save_charuco_image+"board_9x7.png"<<std::endl;
  imwrite(FLAGS_save_charuco_image+"/board_9x7.png", boardImage);

  return 0;
}
