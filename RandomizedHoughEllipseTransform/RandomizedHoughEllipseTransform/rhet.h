#pragma once
#include <Eigen/Dense>
#include <ctime>
#include <random>
#include <thread>
#include <mutex>
#include <algorithm>
#include <opencv2/opencv.hpp>
#define _USE_MATH_DEFINES
#include "math.h"
#include "utils.h"
#include "candidate.h"

#define THREAD_NUM 6
#define VERBOSE_MODE false
#define PLOT_MODE false



class RandomizedHough {
public:
    cv::Mat* image;
    cv::Mat* mask;
    std::vector<Candidate> accumulator;

    RandomizedHough() {
        _maxIter = 1500;
        _majorBoundMax = 100;
        _majorBoundMin = 45;
        _minorBoundMax = 100;
        _minorBoundMin = 40;
        _flatteningBound = 0.4;
        _fittingArea = 7;
        _cannyT1 = 70;
        _cannyT2 = 120;
        _cannySobelSize = 3;
        _lock = new std::mutex();
    };
    ~RandomizedHough() {
        delete _lock;
    }
    void Process(cv::Mat& phase, cv::Mat& mask, CandidateInfo& dstCandidate);

protected:
    //settings
    int _maxIter;
    int _majorBoundMax;
    int _majorBoundMin;
    int _minorBoundMax;
    int _minorBoundMin;
    int _cannySobelSize;
    int _fittingArea;
    double _flatteningBound;
    double _cannyT1;
    double _cannyT2;
    std::mutex* _lock;

    void _displayAccumulator();
    void _accumulate(Candidate* candidate);
    void _findCandidate(cv::Mat& image, Candidate*& dstCandidate, std::vector<cv::Point>& locations);
    void _mainFindCandidate(cv::Mat& image, std::vector<cv::Point>& locations);
    void _mainLoopFindCandidate(cv::Mat& image, std::vector<cv::Point>& locations, int iters);
    bool _assertImage();
    bool _findCenter(std::vector<cv::Point>& shuffleP, cv::Mat&, cv::Point& center, std::vector<cv::Point>& OutP);
    bool _findFitPoint(cv::Mat& edge, cv::Point& p, int width, std::vector<cv::Point>& pt);
    bool _findThisCenter(std::vector<Line>& t, cv::Point& center);
    bool _findAxis(std::vector<cv::Point>& threeP, cv::Point& center, double& ax1, double& ax2, double& angle);
    bool _isOutOfMask(Candidate& e);
    bool _canAccumulate(Candidate c, int& idx);
    bool _getSemiAxis(double angle, double PreA, double PreC, double& ax1, double& ax2);
    double _getRotationAngle(double PreA, double PreB, double PreC);
    inline bool _assertCenter(cv::Point& c);
    inline bool _isEllipse(double PreA, double PreB, double PreC);
    inline bool _isValidAxisLength(double ax1, double ax2);
    inline bool _isValidAxisFlattening(double ax1, double ax2);
    std::vector<cv::Point> _findIntersection(std::vector<Line>& t);
    std::vector<Line> _findBisector(std::vector<cv::Point>& p, std::vector<cv::Point>& l);
};


struct compareScore {
    inline bool operator() (const Candidate& c1, const Candidate& c2) {
        return (c1.candidateInfo.score > c2.candidateInfo.score);
    }
};
