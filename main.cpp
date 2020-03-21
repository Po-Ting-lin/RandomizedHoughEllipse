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
//    displayImage(image);

    RandomizedHough r;
    r.run(image, mask);


//    Eigen::Matrix3f A;
//    Eigen::Vector3f b;

//    A << 1,2,3,  4,5,6,  7,8,10;
//    b << 3, 3, 4;
//    cout << "Here is the matrix A:\n" << A << endl;
//    cout << "Here is the vector b:\n" << b << endl;
//    Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);
//    cout << "The solution is:\n" << x << endl;
//
//    Point p(2.0, 1.0);
//    cout << (p.x + p.y) / 2;

    return 0;
}
