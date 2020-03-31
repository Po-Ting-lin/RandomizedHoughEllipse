//
// Created by ptl on 2020-03-10.
//

#include "RandomizedHough.h"
#include "HoughException.h"
#include <algorithm>

bool RandomizedHough::findCenter(vector<Point> &shuffleP, Mat &edge, Point &center, vector<Point> &OutP) {
    vector<Point> chosenP;
    vector<Line> tanLine;
    vector<Point> intersection;
    vector<Line> bisector;
    int w = fittingArea/2;

    for (Point &eachP: shuffleP){

        // loop crop image, find where == 255
        vector<Point> fitPt;
        if (!findFitPoint(edge, eachP, w, fitPt)){
            // less than two point
            continue;
        }

        // fitting
        Line l(0.0, 0.0);
        if (!fitPoints(fitPt, l)){
            // vertical tanLine
            continue;
        }

        if (PlotMode) {
            cout << "chosenP : " << eachP.x << " " << eachP.y << endl;
            cout << "tanLine: " << l.m << " " << l.c << endl;
        }

        tanLine.push_back(l);
        chosenP.push_back(eachP);

        if (tanLine.size() == 3){
            break;
        }
    }

    // three line -> two intersection
    intersection = findIntersection(tanLine);

    // find bisector
    bisector = findBisector(chosenP, intersection);


    /* ********************* canvas start ******************** */
    if (PlotMode) {
        for (Line ii: tanLine) {
            drawFullImageLine(*canvas, ii.m, ii.c, 100);
        }
        for (Point &pp: chosenP) {
            Point ppp(pp.y, pp.x);
            cv::circle(*canvas, pp, 2, 100, -1); // black (x, y)
        }
        for (Line &ll: bisector) {
            drawFullImageLine(*canvas, ll.m, ll.c, 50);
        }
    }
    /* ********************* canvas end ********************** */

    // update output
    if (findThisCenter(bisector, center)) {
        OutP.assign(chosenP.begin(), chosenP.end());
        return true;
    }
    else{
        return false;
    }
}

bool RandomizedHough::findAxis(vector<Point> &threeP, Point &center, double &ax1, double &ax2, double &angle) {
    // shift to origin
    for (Point &eachP: threeP){
        eachP.x = eachP.x - center.x;
        eachP.y = eachP.y - center.y;
    }

    // find A B C
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    double PreA=0, PreB=0, PreC=0;
    for (int i=0; i<3; i++) {
        A(i, 0) = (double) threeP[i].x * threeP[i].x;
        A(i, 1) = (double) 2.0*threeP[i].x * threeP[i].y;
        A(i, 2) = (double) threeP[i].y * threeP[i].y;
        b(i) = 1.0;
    }
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
    PreA = x(0);
    PreB = x(1);
    PreC = x(2);

    // is valid ellipse?
    if (!assertEllipse(PreA, PreB, PreC)){
        return false;
    }

    // calculate angle
    angle = getRotationAngle(PreA, PreB, PreC); // assign angle

    // calculate semi axis
    if (!getSemi(angle, PreA, PreC, ax1, ax2)){ // assign ax1 ax2
        return false;
    }

    // assert constraint
    if (!assertAxisFlatten(ax1, ax2)){
        return false;
    }

    if (PlotMode) {
        cout << "angle: " << angle << endl;
        cout << "semi axis: " << ax1 << ", " << ax2 << endl;
    }
    return true;
}


bool RandomizedHough::findFitPoint(Mat &edge, Point &p, int width, vector<Point> &pt) {
    vector<Point> tmp;
    for (int i=p.y-width; i<p.y+width+1; i++){
        for (int j=p.x-width; j<p.x+width+1; j++){
            int pixel = edge.at<uchar>(i, j);
            if (pixel == 255){
                // output coordinate
                tmp.emplace_back(j, i);
            }
        }
    }
    if (tmp.size() > 3){
        pt.assign(tmp.begin(), tmp.end());
        return true;
    }
    else{
        return false;
    }
}

vector<Point> RandomizedHough::findIntersection(vector<Line> &t) {
    Eigen::Matrix2d A;
    Eigen::Vector2d b;
    vector<Point> intersection;
    for(int j=0; j<2; j++) {
        for (int i=0; i<2; i++) {
            A(i, 0) = t[i+j].m;
            A(i, 1) = -1;
            b(i) = -t[i+j].c;
        }
        Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
        intersection.emplace_back(x(0), x(1));
        if (PlotMode) {
            cout << "intersection " << x(0) << ", " << x(1) << endl;
        }
    }
    return intersection;
}

vector<Line> RandomizedHough::findBisector(vector<Point> &p, vector<Point> &l) {
    vector<Line> Bisector;
    for (int i=0; i<2; i++){
        Point mid((double)(p[i].x + p[i+1].x)/2.0, (double)(p[i].y + p[i+1].y)/2.0);
        double m = (double) (mid.y - l[i].y) / (double) (mid.x - l[i].x);
        double c = (double) (mid.x*l[i].y - l[i].x*mid.y) / (double) (mid.x - l[i].x);
        Bisector.emplace_back(m,c);
        if (PlotMode){
            cout << "Bisector " << m << ", " << c << endl;
        }
    }
    return Bisector;
}

bool RandomizedHough::findThisCenter(vector<Line> &t, Point &center) {
    Eigen::Matrix2d A;
    Eigen::Vector2d b;
    for (int i=0; i<2; i++) {
        A(i, 0) = t[i].m;
        A(i, 1) = -1;
        b(i) = -t[i].c;
    }
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
    center.x = x(0);
    center.y = x(1);
    if (isnan(x(0)) || isnan(x(1))) return false;
    if (PlotMode) {
        cout << "center: " << x(0) << ", " << x(1) << endl;
    }
    return assertCenter(center);
}

double RandomizedHough::getRotationAngle(double PreA, double PreB, double PreC) {
    double angle = 0.0;
    if (PreA == PreC){
        angle = 0.0;
    }
    else {
        angle = 0.5 * atan((2.0 * PreB) / (PreA - PreC));
    }
    if (PreA > PreC){
        if (PreB < 0) angle += 0.5*M_PI;  // +90 deg
        else if (PreB > 0) angle -= 0.5*M_PI;  // -90 deg
    }
    return angle;
}

bool RandomizedHough::getSemi(double angle, double PreA, double PreC, double &ax1, double &ax2) {
    Eigen::Matrix2d A;
    Eigen::Vector2d b;
    A(0, 0) = sin(angle) * sin(angle);
    A(0, 1) = cos(angle) * cos(angle);
    A(1, 0) = cos(angle) * cos(angle);
    A(1, 1) = sin(angle) * sin(angle);
    b(0) = PreA;
    b(1) = PreC;
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
    if (x(0) > 0 && x(1) > 0){
        ax1 = 1.0/sqrt(min(x(0), x(1))); // semi major
        ax2 = 1.0/sqrt(max(x(0), x(1))); // semi minor
        return true;
    }
    else{
        return false;
    }
}


bool RandomizedHough::canAccumulate(Candidate c, int &idx) {
    if (accumulator.empty()) return false;
    for (int i=0; i<accumulator.size(); i++){
        Candidate old = accumulator[i];
        double CenterDist = sqrt(pow((old.center.x - c.center.x), 2) + pow((old.center.y - c.center.y), 2));
        double AngleDiff = abs(old.angle - c.angle);
        double Angle180 = (c.angle > 0)? c.angle - M_PI: c.angle + M_PI;
        double AngleDiff180 = abs(old.angle - Angle180);
        double AngleDist = min(AngleDiff, Angle180);
        double SemiMajorDist = abs(old.semiMajor - c.semiMajor);
        double SemiMinorDist = abs(old.semiMinor - c.semiMinor);
        if (CenterDist < 5 && AngleDist < M_PI/18.0 && SemiMajorDist < 10 && SemiMinorDist < 10){
            idx = i;
            return true;
        }
    }
    return false;
}

void RandomizedHough::displayAccumulator() {
    cout << "********** Accumulator ***********" << endl;
    int count = 0;
    if (accumulator.empty()) return;
    for (Candidate &c: accumulator){
        cout << "Center: " << c.center << "  ";
        cout << "Semi axis: [" << c.semiMajor << ", " << c.semiMinor << "]  ";
        cout << "Angle: " << c.angle << " ";
        cout << "Score: " << c.score << endl;
        count++;
        if (count > 15) break;
    }
    cout << "**********************************" << endl;
}


void RandomizedHough::run(Mat &p, Mat &m) {
    vector<Point> locations;

    // load image
    phase = &p;
    mask = &m;
    assertInput();

    // canny
    Mat edgeImage;
    Mat clean(edgeImage.size(), edgeImage.type());
    Canny(*phase, edgeImage, cannyT1, cannyT2, cannySobelSize);
    // apply mask
    bitwise_and(edgeImage, edgeImage, clean, *mask);
//    displayImage(clean);

    // canvas
    Mat canvasImage, canvasImage2;
    canvas = &canvasImage;
    origin = &canvasImage2;
    clean.copyTo(*canvas);
    clean.copyTo(*origin);

    // load points into locations
    findNonZero(clean, locations);
    if (PlotMode) {
        cout << "length of locations: " << locations.size() << endl;
    }
    if (locations.size() < 3) throw NoEdgeException();

    // main loop
    for (int i=0; i<maxIter; i++){
        Point center(0.0, 0.0);
        vector<Point> threePoint;
        double ax1=0, ax2=0, angle=0;

        // copy locations
        vector<Point> shuffleP;
        shuffleP.assign(locations.begin(), locations.end());
        // shuffle locations
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::shuffle(shuffleP.begin(), shuffleP.end(), std::default_random_engine(seed));

        // find center
        if (!findCenter(shuffleP, edgeImage, center, threePoint)){
            if (PlotMode) origin->copyTo(*canvas); // clean canvas
            // invalid center
            continue;
        }

        // find semi axis
        if (!findAxis(threePoint, center, ax1, ax2, angle)){
            if (PlotMode) origin->copyTo(*canvas); // clean canvas
            // invalid axis
            continue;
        }

        // assert
        Candidate candi(center, ax1, ax2, angle);
        if (isOutOfMask(candi)){
            if (PlotMode) origin->copyTo(*canvas); // clean canvas
            // this ellipse is out of mask;
            continue;
        }

        // display ellipse
        if (PlotMode){
            ellipse(*canvas, center, Size(ax1, ax2), angle*180.0/M_PI, 0, 360, 240, 1);
        }

        // accumulate
        int idx = -1;
        if (canAccumulate(candi, idx)){
            // average weight
            accumulator[idx].averageWith(candi);
        }
        else{
            // append candidate
            accumulator.emplace_back(center, ax1, ax2, angle);
        }
        if (PlotMode) break;
    }

    // sort accumulator
    sort(accumulator.begin(), accumulator.end(), compareScore());
//    displayAccumulator();

    // best candidate
    Candidate best = accumulator[0];
    ellipse(*phase, best.center, Size(best.semiMajor, best.semiMinor), best.angle*180.0/M_PI, 0, 360, 240, 1);
    displayImage(*phase);
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


bool RandomizedHough::assertCenter(Point &c) {
    if (c.x >= phase->cols || c.x < 0) return false;
    if (c.y >= phase->rows || c.y < 0) return false;
    Mat TestImage(mask->size(), 0, Scalar(0));
    Mat OutImage(mask->size(), 0, Scalar(0));
    vector<Point> locations;
    TestImage.at<uchar>(c.y, c.x) = 255;
    bitwise_and(TestImage, TestImage, OutImage, *mask);
    findNonZero(OutImage, locations);
    return !locations.empty();
}

bool RandomizedHough::assertEllipse(double PreA, double PreB, double PreC) {
    return (PreA * PreC - PreB * PreB) > 0;
}

bool RandomizedHough::assertAxisFlatten(double ax1, double ax2) {
    if (ax1 < majorBoundMin || ax1 > majorBoundMax) return false;
    if (ax2 < minorBoundMin || ax2 > minorBoundMax) return false;
    double flattening = (ax1 - ax2) / ax1;
    return flattening <= flatteningBound;
}

bool RandomizedHough::isOutOfMask(Candidate &e) {
    Mat TestImage(mask->size(), 0, Scalar(0));
    Mat OutImage(mask->size(), 0, Scalar(0));
    Mat AntiMask(mask->size(), 0, Scalar(0));
    vector<Point> locations;
    ellipse(TestImage, e.center, Size(e.semiMajor, e.semiMinor), e.angle*180.0/M_PI, 0, 360, 255, 1);
    bitwise_not(*mask, AntiMask);
    bitwise_and(TestImage, TestImage, OutImage, AntiMask);
    findNonZero(OutImage, locations);
    return !locations.empty(); // if empty, notOutOfMask(false)
}
