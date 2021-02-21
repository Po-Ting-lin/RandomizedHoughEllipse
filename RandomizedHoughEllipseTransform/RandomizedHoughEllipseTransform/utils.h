#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>


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
bool findfittingPoints(std::vector<cv::Point>& pts, Line& line);
std::string type2str(int type);
void drawFullImageLine(cv::Mat& img, double slope, double intercept, cv::Scalar color);
void drawPoint(cv::Mat& img, int x, int y, cv::Scalar color);
void drawText(cv::Mat& img, int x, int y, std::string text);

