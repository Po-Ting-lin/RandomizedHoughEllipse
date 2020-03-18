//
// Created by ptl on 2020-03-10.
//

#ifndef RANDOMIZEDHOUGHELLIPSE_RANDOMIZEDHOUGH_H
#define RANDOMIZEDHOUGHELLIPSE_RANDOMIZEDHOUGH_H

#include "utils.h"


using namespace cv;


class AccumulatorElement{
public:
    int centerI;
    int centerJ;
    int semiMajor;
    int semiMinor;
    double angle;
    double score;

    AccumulatorElement(int centerI, int centerJ, int semiMajor, int semiMinor, double angle){
        this->centerI = centerI;
        this->centerJ = centerJ;
        this->semiMajor = semiMajor;
        this->semiMinor = semiMinor;
        this->angle = angle;
        this->score = 0.0;
    }
};


class RandomizedHough {
protected:
    //settings
    int maxIter;
    int majorBoundMax;
    int majorBoundMin;
    int minorBoundMax;
    int minorBoundMin;
    double flatteningBound;
    int fittingArea;
    double cannyT1;
    double cannyT2;
    int cannySobelSize;

    vector<Point> PointSelection(vector<Point> v);

    Point findCenter(vector<Point> threeP, Mat &edge);


public:

    Mat * phase;
    Mat * mask;

    // accumulator
    vector<AccumulatorElement> accumulator;

    // debug
    bool PlotMode;

    RandomizedHough(bool PlotMode=false){
        // TODO: parameter selection
        maxIter = 800;
        majorBoundMax = 250;
        majorBoundMin = 60;
        minorBoundMax = 250;
        minorBoundMin = 60;
        flatteningBound = 0.8;
        fittingArea = 7;
        cannyT1 = 60;
        cannyT2 = 100;
        cannySobelSize = 3;
        PlotMode = PlotMode;
    };

    void run(Mat &phase, Mat &mask);


};


#endif //RANDOMIZEDHOUGHELLIPSE_RANDOMIZEDHOUGH_H
