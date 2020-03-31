#include <iostream>
#include "utils.h"
#include "RandomizedHough.h"
#include <ctime>
#include <Eigen/Dense>
#include <random>
#include <ctime>
#include <chrono>

using namespace std;
using namespace cv;

int main() {
    Mat image, mask;
    image = imread("/home/ptl/Downloads/phase.png", 0);
    mask = imread("/home/ptl/Downloads/mask.png", 0);
//    displayImage(image);

    // test
    RandomizedHough r(false);

    auto start = std::chrono::system_clock::now();
    r.run(image, mask);
    auto end = std::chrono::system_clock::now();

    // time
    std::chrono::duration<double> elapsed_seconds = end-start;
    cout << "time consume: " << elapsed_seconds.count() << endl;

    return 0;
}
