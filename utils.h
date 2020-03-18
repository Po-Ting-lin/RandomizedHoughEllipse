//
// Created by ptl on 2020-03-10.
//

#ifndef RANDOMIZEDHOUGHELLIPSE_UTILS_H
#define RANDOMIZEDHOUGHELLIPSE_UTILS_H

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#define REAL double

using namespace std;

void displayImage(const cv::Mat& image);

void printCV8UImage(const cv::Mat& image);

void linearReg(int n, vector<double> x, vector<double> y, REAL* b, REAL* m, REAL* r);



#endif //RANDOMIZEDHOUGHELLIPSE_UTILS_H
