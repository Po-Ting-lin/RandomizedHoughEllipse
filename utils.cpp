//
// Created by ptl on 2020-03-10.
//

#include "utils.h"
#include <cmath>
#include <vector>




void displayImage(const cv::Mat &image, bool mag){
    cv::Mat Out;
    if (mag){
        cv::resize(image, Out, cv::Size(), 3, 3);

        for (int i=0; i<image.cols; i=i+20){
            cv::Point coo(i*3, 20);
            cv::putText(Out, patch::to_string(i), coo, cv::FONT_HERSHEY_SIMPLEX, 0.5, 255);
        }
        for (int i=0; i<image.rows; i=i+20){
            cv::Point coo(0, i*3);
            cv::putText(Out, patch::to_string(i), coo, cv::FONT_HERSHEY_SIMPLEX, 0.5, 255);
        }
    }
    else{
        image.copyTo(Out);
    }
    namedWindow( "Display window", cv::WINDOW_AUTOSIZE);
    cv::imshow( "Display window", Out);
    cv::waitKey(0);
}

void printCV8UImage(const cv::Mat& image){
    if (image.depth() != 0)throw exception();
    int count255 = 0;
    int countOther = 0;
    for (int i=0; i<image.rows; i++){
        for (int j=0; j<image.cols; j++){
            int pixel = image.at<uchar>(i,j);
            if (pixel == 0){}
            else if (pixel == 255){count255++;}
            else {countOther++;}
            cout << setw(4) << pixel << " ";
        }
        cout << endl;
    }
    cout << "255 count: "<< count255 << endl;
    cout << "0 count: " << countOther << endl;
}

// from Mark Lakata (https://stackoverflow.com/questions/5083465/fast-efficient-least-squares-fit-algorithm-in-c)
bool fitPoints(vector<cv::Point> &pts, Line &line) {
    int nPoints = pts.size();
    if( nPoints < 2 ) {
        // Fail: infinitely many lines passing through this single point
        cout << "fitPoints: less than 2 points" << endl;
        return false;
    }
    double sumX=0, sumY=0, sumXY=0, sumX2=0, sumY2=0;
    for(int i=0; i<nPoints; i++) {
        double xx = pts[i].x;
        double yy = pts[i].y;
        sumX += xx;
        sumY += yy;
        sumXY += xx * yy;
        sumX2 += xx * xx;
        sumY2 += yy * yy;
    }
    double xMean = sumX / nPoints;
    double yMean = sumY / nPoints;
    cout << "xM, yM" << xMean << " " << yMean << endl;
    double denominator = sumX2 - sumX * xMean;
    // You can tune the eps (1e-7) below for your specific task
    if( std::fabs(denominator) < 1e-7 ) {
        // Fail: it seems a vertical line
        // TODO make vertical line be possible
        cout << "fitPoints: denominator two small" << denominator << endl;
        return false;
    }
    // slope
    double mm = (sumXY - sumX * yMean) / denominator;
    // intercept
    double cc = yMean - mm * xMean;
    // test
    line.m = mm;
    line.c = cc;
    return true;
}

string type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}

void drawFullImageLine(cv::Mat &img, double slope, double intercept, cv::Scalar color){
    //points of line segment which extend the segment P1-P2 to
    //the image borders.
    cv::Point p,q;

    //test if line is vertical, otherwise computes line equation
    //y = ax + b
    if (slope > INT16_MAX){
        p = cv::Point(intercept, 0);
        q = cv::Point(intercept, img.rows);
    }
    else{
        p = cv::Point(0, intercept);
        q = cv::Point(img.cols, slope*img.cols + intercept);

        //clipline to the image borders. It prevents a known bug on OpenCV
        //versions 2.4.X when drawing
        cv::clipLine(cv::Size(img.cols, img.rows), p, q);
    }
    cv::line(img, p, q, color, 1);
}
