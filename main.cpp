#include <iostream>
#include "utils.h"
#include "RandomizedHough.h"
#include <ctime>

using namespace std;
using namespace cv;

int main() {
    Mat image, mask;
    image = imread("/home/ptl/Downloads/phase.png");
    mask = imread("/home/ptl/Downloads/mask.png");
    displayImage(image);

    RandomizedHough r;
    r.run(image, mask);


    return 0;
}
