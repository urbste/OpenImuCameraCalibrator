// created by Steffen Urban November 2019
#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <theia/sfm/view.h>
#include <theia/sfm/reconstruction.h>

#include <vector>
#include <algorithm>
#include <dirent.h>

namespace OpenCamCalib {
namespace utils {

bool ReadDetectorParameters(std::string filename,
                            cv::Ptr<cv::aruco::DetectorParameters>& params);

double MedianOfDoubleVec(std::vector<double>& double_vec);

void PrintResult(const std::string cam_type,
                 const theia::Reconstruction &recon_calib_dataset);

std::string CameraIDToString(const int theia_enum);

double GetReprojErrorOfView(const theia::Reconstruction& recon_dataset,
                            const theia::ViewId v_id);

std::vector<std::string> load_images(const std::string &img_dir_path);


// from https://rosettacode.org/wiki/Averages/Simple_moving_average#C.2B.2B
class SimpleMovingAverage {
public:
    SimpleMovingAverage(unsigned int period) :
        period(period), window(new double[period]), head(NULL), tail(NULL),
                total(0) {
        assert(period >= 1);
    }
    ~SimpleMovingAverage() {
        delete[] window;
    }

    // Adds a value to the average, pushing one out if nescessary
    void add(double val) {
        // Special case: Initialization
        if (head == NULL) {
            head = window;
            *head = val;
            tail = head;
            inc(tail);
            total = val;
            return;
        }

        // Were we already full?
        if (head == tail) {
            // Fix total-cache
            total -= *head;
            // Make room
            inc(head);
        }

        // Write the value in the next spot.
        *tail = val;
        inc(tail);

        // Update our total-cache
        total += val;
    }

    // Returns the average of the last P elements added to this SMA.
    // If no elements have been added yet, returns 0.0
    double avg() const {
        ptrdiff_t size = this->size();
        if (size == 0) {
            return 0; // No entries => 0 average
        }
        return total / (double) size; // Cast to double for floating point arithmetic
    }

private:
    unsigned int period;
    double * window; // Holds the values to calculate the average of.

    // Logically, head is before tail
    double * head; // Points at the oldest element we've stored.
    double * tail; // Points at the newest element we've stored.

    double total; // Cache the total so we don't sum everything each time.

    // Bumps the given pointer up by one.
    // Wraps to the start of the array if needed.
    void inc(double * & p) {
        if (++p >= window + period) {
            p = window;
        }
    }

    // Returns how many numbers we have stored.
    ptrdiff_t size() const {
        if (head == NULL)
            return 0;
        if (head == tail)
            return period;
        return (period + tail - head) % period;
    }
};

}
}
