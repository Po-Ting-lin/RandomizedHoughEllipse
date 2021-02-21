#include <iostream>
#include "utils.h"
#include "rhet.h"
#include <ctime>
#include <Eigen/Dense>
#include <random>
#include <ctime>
#include <chrono>


int main() {
    cv::Mat image, mask;
    std::string image_path = R"(phase.png)";
    std::string mask_path = R"(mask.png)";
    image = cv::imread(image_path, 0);
    mask = cv::imread(mask_path, 0);

    if (!image.data || !mask.data) {
        std::cout << "Error: the image wasn't correctly loaded." << std::endl;
        return -1;
    }

    RandomizedHough rhet;
    CandidateInfo candidateInfo;
    auto start = std::chrono::system_clock::now();
    rhet.Process(image, mask, candidateInfo);
    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "time consume: " << elapsed_seconds.count() << std::endl;
    return 0;
}
