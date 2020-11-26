#include "rhet.h"
#include "rhet_exception.h"

void RandomizedHough::run(cv::Mat& p, cv::Mat& m) {
    std::vector<cv::Point> locations;

    // load image
    phase = &p;
    mask = &m;
    assertInput();

    // canny
    cv::Mat edgeImage;
    cv::Mat clean(edgeImage.size(), edgeImage.type());
    cv::Canny(*phase, edgeImage, cannyT1, cannyT2, cannySobelSize);
    // apply mask
    cv::bitwise_and(edgeImage, edgeImage, clean, *mask);
    //displayImage(clean);

    // canvas
    cv::Mat canvasImage, canvasImage2;
    canvas = &canvasImage;
    origin = &canvasImage2;
    clean.copyTo(*canvas);
    clean.copyTo(*origin);

    // load points into locations
    cv::findNonZero(clean, locations);
    if (PlotMode) {
        std::cout << "length of locations: " << locations.size() << std::endl;
    }
    if (locations.size() < 3) throw NoEdgeException();

    // main loop
    for (int i = 0; i < maxIter; i++) {
        cv::Point center(0.0, 0.0);
        std::vector<cv::Point> points_on_ellipse;
        double ax1 = 0, ax2 = 0, angle = 0;

        // copy locations
        std::vector<cv::Point> shuffleP;
        shuffleP.assign(locations.begin(), locations.end());
        // shuffle locations
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::shuffle(shuffleP.begin(), shuffleP.end(), std::default_random_engine(seed));

        // find center
        if (!findCenter(shuffleP, clean, center, points_on_ellipse)) {
            continue;
        }

        // find semi axis
        if (!findAxis(points_on_ellipse, center, ax1, ax2, angle)) {
            continue;
        }

        // assert
        Candidate candidate(center, ax1, ax2, angle);
        if (isOutOfMask(candidate)) {
            continue;
        }

        //// display ellipse
        //if (PlotMode) {
        //    ellipse(*canvas, center, cv::Size(ax1, ax2), angle * 180.0 / M_PI, 0, 360, 240, 1);
        //}

        // accumulate
        int idx = -1;
        if (canAccumulate(candidate, idx)) {
            // average weight
            accumulator[idx].averageWith(candidate);
        }
        else {
            // append candidate
            accumulator.emplace_back(center, ax1, ax2, angle);
        }
    }

    // sort accumulator
    sort(accumulator.begin(), accumulator.end(), compareScore());
    displayAccumulator();

    // best candidate
    Candidate best = accumulator[0];
    ellipse(*phase, best.center, cv::Size(best.semiMajor, best.semiMinor), best.angle * 180.0 / M_PI, 0, 360, 240, 1);
    displayImage(*phase);
}