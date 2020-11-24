#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#define REAL double

//class Pt {
//public:
//    double x, y;
//    Pt(double x, double y){
//        this->x = x;
//        this->y = x;
//    }
//    bool operator==(Pt &a){
//        return a.x == this->x && a.y == this->y;
//    };
//};

namespace patch {
    template < typename T >
    std::string to_string(T n) {
        std::ostringstream stm;
        stm << n;
        return stm.str();
    }
}


class Line {
public:
    double m, c;
    Line(double m, double c) {
        this->m = m;
        this->c = c;
    }
};


void displayImage(const cv::Mat& image, bool mag = false);

void printCV8UImage(const cv::Mat& image);

bool fitPoints(std::vector<cv::Point>& pts, Line& line);

std::string type2str(int type);

void drawFullImageLine(cv::Mat& img, double slope, double intercept, cv::Scalar color);

