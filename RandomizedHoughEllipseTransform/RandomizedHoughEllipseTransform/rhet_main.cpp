#include "rhet.h"
#include "rhet_exception.h"

void RandomizedHough::Process(cv::Mat& imageSrc, cv::Mat& maskSrc, CandidateInfo& dstCandidateInfo) {
    std::vector<cv::Point> locations;
    image = &imageSrc;
    mask = &maskSrc;
    _assertImage();

    // canny
    cv::Mat edge_image;
    cv::Mat clean_edge_image(edge_image.size(), edge_image.type());
    cv::Canny(*image, edge_image, _cannyT1, _cannyT2, _cannySobelSize);
    cv::bitwise_and(edge_image, edge_image, clean_edge_image, *mask);

    // load points into locations
    cv::findNonZero(clean_edge_image, locations);
#if VERBOSE_MODE
    std::cout << "length of locations: " << locations.size() << std::endl;
#endif
    if (locations.size() < 3) throw NoEdgeException();

    int iters = _maxIter / THREAD_NUM;
    std::thread threads[THREAD_NUM];
    for (int i = 0; i < THREAD_NUM; i++) {
        threads[i] = std::thread(&RandomizedHough::_mainLoopFindCandidate, this, clean_edge_image, locations, iters);
    }
    for (int i = 0; i < THREAD_NUM; i++) {
        threads[i].join();
    }

    if (accumulator.size() > 1) {
        sort(accumulator.begin(), accumulator.end(), compareScore());
    }
    _displayAccumulator();

#if VERBOSE_MODE
    CandidateInfo best_info = accumulator[0].candidateInfo;
    cv::Point best_center(best_info.centerX, best_info.centerY);
    ellipse(*image, best_center, cv::Size(best_info.semiMajor, best_info.semiMinor), best_info.angle * 180.0 / M_PI, 0, 360, 240, 1);
    displayImage(*image);
#endif
    dstCandidateInfo = accumulator[0].candidateInfo;
}
