#include <iostream>
#include "utils.h"
#include "RandomizedHough.h"
#include <ctime>
#include <Eigen/Dense>
#include <random>

using namespace std;
using namespace cv;

int main() {
    Mat image, mask;
    image = imread("/home/ptl/Downloads/phase.png");
    mask = imread("/home/ptl/Downloads/mask.png", 0);
    displayImage(image);

    RandomizedHough r(false);
    r.run(image, mask);

    return 0;
}
