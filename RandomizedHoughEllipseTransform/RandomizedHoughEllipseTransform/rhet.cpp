#include "rhet.h"
#include "rhet_exception.h"

#define ERROR_VERBOSE false
#define PLOT false

bool RandomizedHough::findCenter(std::vector<cv::Point>& shufflePoint, cv::Mat& edgeImage, cv::Point& center, std::vector<cv::Point>& pointOnEllipse) {
    std::vector<cv::Point> chosen_point;
    std::vector<Line> tan_line;
    std::vector<cv::Point> intersection;
    std::vector<Line> bisector;
    int width = fittingArea / 2;

    for (cv::Point& current_point : shufflePoint) {
        // loop crop image, find where == 255
        std::vector<cv::Point> fit_point;
        if (!findFitPoint(edgeImage, current_point, width, fit_point)) {
#if ERROR_VERBOSE
            printf("Fitting points are less than two!\n");
#endif
            continue;
        }

        // fitting
        Line line(0.0, 0.0);
        if (!fitPoints(fit_point, line)) {
#if ERROR_VERBOSE
            printf("Vertical tanLine!\n");
#endif
            continue;
        }

#if ERROR_VERBOSE
            std::cout << "current_point : " << current_point.x << " " << current_point.y << std::endl;
            std::cout << "tan_line: " << line.m << " " << line.c << std::endl;
#endif

        tan_line.push_back(line);
        chosen_point.push_back(current_point);

        if (tan_line.size() == 3) {
            break;
        }
    }

    // three line -> two intersection
    intersection = findIntersection(tan_line);

    // find bisector
    bisector = findBisector(chosen_point, intersection);

    // update output
    bool is_success = false;
    if (findThisCenter(bisector, center)) {
        pointOnEllipse.assign(chosen_point.begin(), chosen_point.end());
        is_success = true;
    }
    
    if (is_success) {
#if PLOT
        cv::Mat debug_black;
        edgeImage.copyTo(debug_black);
        for (int i = 0; i < chosen_point.size(); i++) {
            drawPoint(debug_black, chosen_point[i].x, chosen_point[i].y, 100);
            drawText(debug_black, chosen_point[i].x, chosen_point[i].y, patch::to_string(i));
            drawFullImageLine(debug_black, tan_line[i].m, tan_line[i].c, 200);
        }
        for (int i = 0; i < intersection.size(); i++) {
            drawPoint(debug_black, intersection[i].x, intersection[i].y, 50);
        }
        drawPoint(debug_black, center.x, center.y, 255);
        drawText(debug_black, center.x, center.y, "center");
        displayImage(debug_black, true);
#endif
    }
    return is_success;
}

bool RandomizedHough::findAxis(std::vector<cv::Point>& threeP, cv::Point& center, double& ax1, double& ax2, double& angle) {
    // shift to origin
    for (cv::Point& eachP : threeP) {
        eachP.x = eachP.x - center.x;
        eachP.y = eachP.y - center.y;
    }

    // find A B C
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    double PreA = 0, PreB = 0, PreC = 0;
    for (int i = 0; i < 3; i++) {
        A(i, 0) = (double)threeP[i].x * threeP[i].x;
        A(i, 1) = (double)2.0 * threeP[i].x * threeP[i].y;
        A(i, 2) = (double)threeP[i].y * threeP[i].y;
        b(i) = 1.0;
    }
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
    PreA = x(0);
    PreB = x(1);
    PreC = x(2);

    // is valid ellipse?
    if (!assertEllipse(PreA, PreB, PreC)) {
        return false;
    }

    // calculate angle
    angle = getRotationAngle(PreA, PreB, PreC); // assign angle

    // calculate semi axis
    if (!getSemi(angle, PreA, PreC, ax1, ax2)) { // assign ax1 ax2
        return false;
    }

    // assert constraint
    if (!assertAxisFlatten(ax1, ax2)) {
        return false;
    }

    if (PlotMode) {
        std::cout << "angle: " << angle << std::endl;
        std::cout << "semi axis: " << ax1 << ", " << ax2 << std::endl;
    }
    return true;
}


bool RandomizedHough::findFitPoint(cv::Mat& edgeImage, cv::Point& currentPoint, int width, std::vector<cv::Point>& outputPoint) {
    std::vector<cv::Point> tmp;
    uchar* edge_image_ptr = edgeImage.data;
    for (int y = currentPoint.y - width; y < currentPoint.y + width + 1; y++) {
        for (int x = currentPoint.x - width; x < currentPoint.x + width + 1; x++) {
            int pixel = edge_image_ptr[y * edgeImage.cols + x];
            if (pixel == 255) {
                // output coordinate
                tmp.emplace_back(x, y);
            }
        }
    }
    if (tmp.size() > 3) {
        outputPoint.assign(tmp.begin(), tmp.end());
        return true;
    }
    else {
        return false;
    }
}

std::vector<cv::Point> RandomizedHough::findIntersection(std::vector<Line>& t) {
    Eigen::Matrix2d A;
    Eigen::Vector2d b;
    std::vector<cv::Point> intersection;
    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < 2; i++) {
            A(i, 0) = t[i + j].m;
            A(i, 1) = -1;
            b(i) = -t[i + j].c;
        }
        Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
        intersection.emplace_back(x(0), x(1));
#if ERROR_VERBOSE
        std::cout << "intersection " << x(0) << ", " << x(1) << std::endl;
#endif
    }
    return intersection;
}

std::vector<Line> RandomizedHough::findBisector(std::vector<cv::Point>& p, std::vector<cv::Point>& l) {
    std::vector<Line> Bisector;
    for (int i = 0; i < 2; i++) {
        cv::Point mid((double)(p[i].x + p[i + 1].x) / 2.0, (double)(p[i].y + p[i + 1].y) / 2.0);
        double m = (double)(mid.y - l[i].y) / (double)(mid.x - l[i].x);
        double c = (double)(mid.x * l[i].y - l[i].x * mid.y) / (double)(mid.x - l[i].x);
        Bisector.emplace_back(m, c);
#if ERROR_VERBOSE
        std::cout << "Bisector " << m << ", " << c << std::endl;
#endif
    }
    return Bisector;
}

bool RandomizedHough::findThisCenter(std::vector<Line>& t, cv::Point& center) {
    Eigen::Matrix2d A;
    Eigen::Vector2d b;
    for (int i = 0; i < 2; i++) {
        A(i, 0) = t[i].m;
        A(i, 1) = -1;
        b(i) = -t[i].c;
    }
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
    center.x = x(0);
    center.y = x(1);
    if (isnan(x(0)) || isnan(x(1))) return false;
#if ERROR_VERBOSE
    std::cout << "center: " << x(0) << ", " << x(1) << std::endl;
#endif
    return assertCenter(center);
}

double RandomizedHough::getRotationAngle(double PreA, double PreB, double PreC) {
    double angle = 0.0;
    if (PreA == PreC) {
        angle = 0.0;
    }
    else {
        angle = 0.5 * atan((2.0 * PreB) / (PreA - PreC));
    }
    if (PreA > PreC) {
        if (PreB < 0) angle += 0.5 * M_PI;  // +90 deg
        else if (PreB > 0) angle -= 0.5 * M_PI;  // -90 deg
    }
    return angle;
}

bool RandomizedHough::getSemi(double angle, double PreA, double PreC, double& ax1, double& ax2) {
    Eigen::Matrix2d A;
    Eigen::Vector2d b;
    A(0, 0) = sin(angle) * sin(angle);
    A(0, 1) = cos(angle) * cos(angle);
    A(1, 0) = cos(angle) * cos(angle);
    A(1, 1) = sin(angle) * sin(angle);
    b(0) = PreA;
    b(1) = PreC;
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
    if (x(0) > 0 && x(1) > 0) {
        ax1 = 1.0 / sqrt(std::min(x(0), x(1))); // semi major
        ax2 = 1.0 / sqrt(std::max(x(0), x(1))); // semi minor
        return true;
    }
    else {
        return false;
    }
}


bool RandomizedHough::canAccumulate(Candidate c, int& idx) {
    if (accumulator.empty()) return false;
    for (int i = 0; i < accumulator.size(); i++) {
        Candidate old = accumulator[i];
        double CenterDist = sqrt(pow((old.center.x - c.center.x), 2) + pow((old.center.y - c.center.y), 2));
        double AngleDiff = abs(old.angle - c.angle);
        double Angle180 = (c.angle > 0) ? c.angle - M_PI : c.angle + M_PI;
        double AngleDiff180 = abs(old.angle - Angle180);
        double AngleDist = std::min(AngleDiff, Angle180);
        double SemiMajorDist = abs(old.semiMajor - c.semiMajor);
        double SemiMinorDist = abs(old.semiMinor - c.semiMinor);
        if (CenterDist < 5 && AngleDist < M_PI / 18.0 && SemiMajorDist < 10 && SemiMinorDist < 10) {
            idx = i;
            return true;
        }
    }
    return false;
}

void RandomizedHough::displayAccumulator() {
    std::cout << "********** Accumulator ***********" << std::endl;
    int count = 0;
    if (accumulator.empty()) return;
    for (Candidate& c : accumulator) {
        std::cout << "Center: " << c.center << "  ";
        std::cout << "Semi axis: [" << c.semiMajor << ", " << c.semiMinor << "]  ";
        std::cout << "Angle: " << c.angle << " ";
        std::cout << "Score: " << c.score << std::endl;
        count++;
        if (count > 15) break;
    }
    std::cout << "**********************************" << std::endl;
}


/********************* assert function ****************************/
bool RandomizedHough::assertInput() {
    // phase
    if (phase->empty()) throw EmptyException("phase");
    if (phase->type() != 0) throw InputException("phase");

    // mask
    if (mask->empty()) throw EmptyException("mask");
    if (mask->type() != 0) throw InputException("mask");
}


bool RandomizedHough::assertCenter(cv::Point& c) {
    if (c.x >= phase->cols || c.x < 0) return false;
    if (c.y >= phase->rows || c.y < 0) return false;
    cv::Mat TestImage(mask->size(), 0, cv::Scalar(0));
    cv::Mat OutImage(mask->size(), 0, cv::Scalar(0));
    std::vector<cv::Point> locations;
    TestImage.at<uchar>(c.y, c.x) = 255;
    cv::bitwise_and(TestImage, TestImage, OutImage, *mask);
    cv::findNonZero(OutImage, locations);
    return !locations.empty();
}

inline bool RandomizedHough::assertEllipse(double PreA, double PreB, double PreC) {
    return (PreA * PreC - PreB * PreB) > 0;
}

inline bool RandomizedHough::assertAxisFlatten(double ax1, double ax2) {
    if (ax1 < majorBoundMin || ax1 > majorBoundMax) return false;
    if (ax2 < minorBoundMin || ax2 > minorBoundMax) return false;
    double flattening = (ax1 - ax2) / ax1;
    return flattening <= flatteningBound;
}

bool RandomizedHough::isOutOfMask(Candidate& e) {
    cv::Mat TestImage(mask->size(), 0, cv::Scalar(0));
    cv::Mat OutImage(mask->size(), 0, cv::Scalar(0));
    cv::Mat AntiMask(mask->size(), 0, cv::Scalar(0));
    std::vector<cv::Point> locations;
    ellipse(TestImage, e.center, cv::Size(e.semiMajor, e.semiMinor), e.angle * 180.0 / M_PI, 0, 360, 255, 1);
    bitwise_not(*mask, AntiMask);
    bitwise_and(TestImage, TestImage, OutImage, AntiMask);
    findNonZero(OutImage, locations);
    return !locations.empty(); // if empty, notOutOfMask(false)
}
