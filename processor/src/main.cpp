#include <cstdlib>
#include <iostream>
#include <opencv4/opencv2/core/cuda.hpp>
#include <opencv4/opencv2/opencv.hpp>

int main(int argc, const char* argv[]) {
    std::cout << "Hello, World!" << std::endl;

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <num1> <num2>" << std::endl;
        return 1;
    }

    int num1 = std::atoi(argv[1]);
    int num2 = std::atoi(argv[2]);

    std::cout << "The sum of " << num1 << " and " << num2 << " is " << (num1 + num2) << std::endl;

    std::cout << "OpenCV version: " << CV_VERSION << std::endl;

    // Check if CUDA is available
    int cudaDeviceCount = cv::cuda::getCudaEnabledDeviceCount();
    std::cout << "CUDA-enabled devices found: " << cudaDeviceCount << std::endl;

    return 0;
}
