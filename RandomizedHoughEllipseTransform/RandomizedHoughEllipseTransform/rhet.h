#pragma once
#include "utils.h"
#include <Eigen/Dense>
#include <ctime>
#include <random>
#include <algorithm>
#define _USE_MATH_DEFINES
#include "math.h"

#define VERBOSE_MODE false
#define PLOT_MODE false

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

    void operator<<=(Candidate c) {
        center.x = (c.center.x + center.x * score) / (score + 1.0);
        center.y = (c.center.y + center.y * score) / (score + 1.0);
        semiMajor = (c.semiMajor + semiMajor * score) / (score + 1.0);
        semiMinor = (c.semiMinor + semiMinor * score) / (score + 1.0);
        angle = (c.angle + angle * score) / (score + 1.0);
        score += 1;
    }
};

class RandomizedHough {
public:
    cv::Mat* image;
    cv::Mat* mask;
    std::vector<Candidate> accumulator;

    RandomizedHough() {
        // TODO: parameter selection
        maxIter = 1000;
        _majorBoundMax = 100;
        _majorBoundMin = 45;
        _minorBoundMax = 100;
        _minorBoundMin = 40;
        _flatteningBound = 0.4;
        _fittingArea = 7;
        _cannyT1 = 70;
        _cannyT2 = 120;
        _cannySobelSize = 3;
    };
    void Process(cv::Mat& phase, cv::Mat& mask);
    void Process(cv::Mat& phase, cv::Mat& mask, bool mp);

protected:
    //settings
    int maxIter;
    int _majorBoundMax;
    int _majorBoundMin;
    int _minorBoundMax;
    int _minorBoundMin;
    int _cannySobelSize;
    int _fittingArea;
    double _flatteningBound;
    double _cannyT1;
    double _cannyT2;

    void displayAccumulator();

    bool assertImage();
    bool findCenter(std::vector<cv::Point>& shuffleP, cv::Mat&, cv::Point& center, std::vector<cv::Point>& OutP);
    bool findFitPoint(cv::Mat& edge, cv::Point& p, int width, std::vector<cv::Point>& pt);
    bool findThisCenter(std::vector<Line>& t, cv::Point& center);
    bool findAxis(std::vector<cv::Point>& threeP, cv::Point& center, double& ax1, double& ax2, double& angle);
    bool isOutOfMask(Candidate& e);
    bool canAccumulate(Candidate c, int& idx);
    bool getSemiAxis(double angle, double PreA, double PreC, double& ax1, double& ax2);
    double getRotationAngle(double PreA, double PreB, double PreC);
    inline bool assertCenter(cv::Point& c);
    inline bool IsEllipse(double PreA, double PreB, double PreC);
    inline bool isValidAxisLength(double ax1, double ax2);
    inline bool isValidAxisFlattening(double ax1, double ax2);
    std::vector<cv::Point> findIntersection(std::vector<Line>& t);
    std::vector<Line> findBisector(std::vector<cv::Point>& p, std::vector<cv::Point>& l);
};


struct compareScore {
    inline bool operator() (const Candidate& c1, const Candidate& c2) {
        return (c1.score > c2.score);
    }
};
