#include "rhet.h"
#include "rhet_exception.h"

void RandomizedHough::Process(cv::Mat& src, cv::Mat& dst) {
    std::vector<cv::Point> locations;
    image = &src;
    mask = &dst;
    assertImage();

    // canny
    cv::Mat edgeImage;
    cv::Mat clean(edgeImage.size(), edgeImage.type());
    cv::Canny(*image, edgeImage, _cannyT1, _cannyT2, _cannySobelSize);
    cv::bitwise_and(edgeImage, edgeImage, clean, *mask);

    // load points into locations
    cv::findNonZero(clean, locations);
#if VERBOSE_MODE
    std::cout << "length of locations: " << locations.size() << std::endl;
#endif
    if (locations.size() < 3) throw NoEdgeException();

    for (int i = 0; i < maxIter; i++) {
        cv::Point center(0.0, 0.0);
        std::vector<cv::Point> points_on_ellipse;
        double ax1 = 0, ax2 = 0, angle = 0;

        // copy locations
        std::vector<cv::Point> shuffle_points;
        shuffle_points.assign(locations.begin(), locations.end());

        // shuffle locations
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::shuffle(shuffle_points.begin(), shuffle_points.end(), std::default_random_engine(seed));

        // find center
        if (!findCenter(shuffle_points, clean, center, points_on_ellipse)) continue;

        // find semi axis
        if (!findAxis(points_on_ellipse, center, ax1, ax2, angle)) continue;

        // assert
        Candidate candidate(center, ax1, ax2, angle);
        if (isOutOfMask(candidate)) continue;

#if VERBOSE_MODE
        break;
#endif

        // accumulate
        int idx = -1;
        if (canAccumulate(candidate, idx)) {
            accumulator[idx] <<= candidate;
        }
        else {
            accumulator.emplace_back(center, ax1, ax2, angle);
        }
    }

    if (accumulator.size() > 1) {
        sort(accumulator.begin(), accumulator.end(), compareScore());
    }
    displayAccumulator();

#if !VERBOSE_MODE
    Candidate best = accumulator[0];
    ellipse(*image, best.center, cv::Size(best.semiMajor, best.semiMinor), best.angle * 180.0 / M_PI, 0, 360, 240, 1);
    displayImage(*image);
#endif
}