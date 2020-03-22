//
// Created by ptl on 2020-03-10.
//

#ifndef RANDOMIZEDHOUGHELLIPSE_RANDOMIZEDHOUGH_H
#define RANDOMIZEDHOUGHELLIPSE_RANDOMIZEDHOUGH_H

#include "utils.h"
#include <Eigen/Dense>
#include <ctime>

using namespace cv;



class Candidate{
public:
    Point center;
    double semiMajor;
    double semiMinor;
    double angle;
    double score;

    Candidate(Point &center, double semiMajor, double semiMinor, double angle){
        this->center = center;
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

    bool assertCenter(Point &c);

    bool findCenter(vector<Point> &shuffleP, Mat &, Point &center, vector<Point> &OutP);
    bool findFitPoint(Mat &edge, Point &p, int width, vector<Point> &pt);
    vector<Point> findIntersection(vector<Line> &t);
    vector<Line> findBisector(vector<Point> &p, vector<Point> &l);
    bool findThisCenter(vector<Line> &t, Point &center);


    bool findAxis(vector<Point> &threeP, Point &center, double &ax1, double &ax2, double &angle);
    bool assertEllipse(double PreA, double PreB, double PreC);
    double getRotationAngle(double PreA, double PreB, double PreC);
    bool getSemi(double angle, double PreA, double PreC, double &ax1, double &ax2);
    bool assertAxisFlatten(double ax1, double ax2);
    bool isOutOfMask(Candidate &e);



public:

    Mat * phase;
    Mat * mask;
    Mat * canvas;
    Mat * canvas2;

    // accumulator
    vector<Candidate> accumulator;

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
