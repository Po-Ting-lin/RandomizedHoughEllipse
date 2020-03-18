//
// Created by ptl on 2020-03-10.
//

#include "RandomizedHough.h"
#include "HoughException.h"
#include <ctime>

vector<Point> RandomizedHough::PointSelection(vector<Point> v) {
    // copy v

    vector<Point> output;

    // random seed
    srand(time(nullptr));

    for (int i=0;i<3;i++){

        // range from 0 to v.size
        int ranNumber = rand() % (v.size() + 1);
        output.push_back(v[ranNumber]);

        // not repeated selection
        v.erase(find(v.begin(),v.end(),v[ranNumber]));

        std::cout << "random number: "<< ranNumber << endl;
    }
    return output;
}

Point RandomizedHough::findCenter(vector<Point> threeP, Mat &edge) {
    vector<double> mV;
    vector<double> cV;
    int w = fittingArea/2;

    for (Point &eachP: threeP){
        // loop crop image
        vector<double> fitX;
        vector<double> fitY;

        for (int i=eachP.x-w; i<eachP.x+w+1; i++){
            for (int j=eachP.y-w; i<eachP.y+w+1; j++){
                int pixel = edge.at<uchar>(i, j);
                if (pixel == 255){
                    // output coordinate (int to double)
                    fitX.push_back(i);
                    fitY.push_back(j);
                }
            }
        }

        // fitting
        int n = fitX.size();
        double c = 0.0;
        double m = 0.0;
        linearReg(n, fitY, fitX, &c, &m, nullptr); // transpose X Y
        mV.push_back(m);
        cV.push_back(c);
    }

    // TODO: find a package can solve matrix


    return cv::Point();
}


void RandomizedHough::run(Mat &p, Mat &m) {
    vector<Point> locations;

    // load image
    phase = &p;
    mask = &m;

    // canny
    Mat edgeImage;
    Canny(*phase, edgeImage, cannyT1, cannyT2, cannySobelSize);

    // load points into locations
    findNonZero(edgeImage, locations);
    cout << "length of locations: " << locations.size() << endl;
    if (locations.size() < 3) throw NoEdgeExcception();

//    // main loop
//    for (int i=0; i<maxIter; i++){
//        vector<Point> threeP = PointSelection(locations, edgeImage);
//
//
//
//        break;
//    }
    printCV8UImage(edgeImage);

//    displayImage(edgeImage);

}





