#pragma once
#include "utils.h"
#include <Eigen/Dense>
#include <ctime>
#include <random>
#include <algorithm>
#define _USE_MATH_DEFINES
#include "math.h"

class Candidate {
public:
    cv::Point center;
    double semiMajor;
    double semiMinor;
    double angle;
    double score;

    Candidate(cv::Point& center, double semiMajor, double semiMinor, double angle) {
        this->center = center;
        this->semiMajor = semiMajor;
        this->semiMinor = semiMinor;
        this->angle = angle;
        this->score = 1.0;
    }

    void averageWith(Candidate& c) {
        center.x = (c.center.x + center.x * score) / (score + 1.0);
        center.y = (c.center.y + center.y * score) / (score + 1.0);
        semiMajor = (c.semiMajor + semiMajor * score) / (score + 1.0);
        semiMinor = (c.semiMinor + semiMinor * score) / (score + 1.0);
        angle = (c.angle + angle * score) / (score + 1.0);
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

    bool assertCenter(cv::Point& c);
    bool assertInput();

    bool findCenter(std::vector<cv::Point>& shuffleP, cv::Mat&, cv::Point& center, std::vector<cv::Point>& OutP);
    bool findFitPoint(cv::Mat& edge, cv::Point& p, int width, std::vector<cv::Point>& pt);
    std::vector<cv::Point> findIntersection(std::vector<Line>& t);
    std::vector<Line> findBisector(std::vector<cv::Point>& p, std::vector<cv::Point>& l);
    bool findThisCenter(std::vector<Line>& t, cv::Point& center);

    bool findAxis(std::vector<cv::Point>& threeP, cv::Point& center, double& ax1, double& ax2, double& angle);
    bool assertEllipse(double PreA, double PreB, double PreC);
    double getRotationAngle(double PreA, double PreB, double PreC);
    bool getSemi(double angle, double PreA, double PreC, double& ax1, double& ax2);
    bool assertAxisFlatten(double ax1, double ax2);
    bool isOutOfMask(Candidate& e);

    bool canAccumulate(Candidate c, int& idx);
    void displayAccumulator();



public:
    cv::Mat* phase;
    cv::Mat* mask;
    cv::Mat* canvas;
    cv::Mat* origin;

    // accumulator
    std::vector<Candidate> accumulator;

    // debug
    bool PlotMode;

    RandomizedHough(bool PlotMode = false) {
        // TODO: parameter selection
        maxIter = 5000;
        majorBoundMax = 100;
        majorBoundMin = 45;
        minorBoundMax = 100;
        minorBoundMin = 40;
        flatteningBound = 0.4;
        fittingArea = 7;
        cannyT1 = 50;
        cannyT2 = 90;
        cannySobelSize = 3;
        PlotMode = PlotMode;
    };

    void run(cv::Mat& phase, cv::Mat& mask);


};


struct compareScore {
    inline bool operator() (const Candidate& c1, const Candidate& c2) {
        return (c1.score > c2.score);
    }
};
