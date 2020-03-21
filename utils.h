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

namespace patch{
    template < typename T >
    string to_string(T n){
        ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}


class Line{
public:
    double m, c;
    Line(double m, double c){
        this->m = m;
        this->c = c;
    }
};


void displayImage(const cv::Mat& image, bool mag=false);

void printCV8UImage(const cv::Mat& image);

bool fitPoints(vector<cv::Point> &pts, Line &line);

string type2str(int type);

void drawFullImageLine(cv::Mat &img, double slope, double intercept, cv::Scalar color);



#endif //RANDOMIZEDHOUGHELLIPSE_UTILS_H
