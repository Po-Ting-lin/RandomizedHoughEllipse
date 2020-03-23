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
        this->score = 1.0;
    }

    void averageWith(Candidate &c){
        center.x = (c.center.x + center.x*score) / (score + 1.0);
        center.y = (c.center.y + center.y*score) / (score + 1.0);
        semiMajor = (c.semiMajor + semiMajor*score) / (score + 1.0);
        semiMinor = (c.semiMinor + semiMinor*score) / (score + 1.0);
        angle = (c.angle + angle*score) / (score + 1.0);
        score += 1;
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

    bool canAccumulate(Candidate c, int &idx);
    void displayAccumulator();



public:
    Mat * phase;
    Mat * mask;
    Mat * canvas;
    Mat * origin;

    // accumulator
    vector<Candidate> accumulator;

    // debug
    bool PlotMode;

    RandomizedHough(bool PlotMode=false){
        // TODO: parameter selection
        maxIter = 5000;
        majorBoundMax = 100;
        majorBoundMin = 25;
        minorBoundMax = 100;
        minorBoundMin = 20;
        flatteningBound = 0.8;
        fittingArea = 7;
        cannyT1 = 50;
        cannyT2 = 90;
        cannySobelSize = 3;
        PlotMode = PlotMode;
    };

    void run(Mat &phase, Mat &mask);


};


struct compareScore{
    inline bool operator() (const Candidate &c1, const Candidate &c2){
        return (c1.score > c2.score);
    }
};


#endif //RANDOMIZEDHOUGHELLIPSE_RANDOMIZEDHOUGH_H
