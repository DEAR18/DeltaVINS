

#include "SlamVisualizer.h"
#include <opencv2/opencv.hpp>

int main() {
    cv::Mat image = cv::imread(
        "Y:\\Euroc\\V201\\mav0\\cam0\\data\\1413393212255760384.png");
    int width_crop = (image.cols - 640) / 2;
    cv::Mat img2 = image.colRange(width_crop, image.cols - width_crop).clone();
    SlamVisualizer* visualizer = new SlamVisualizer(640, 480);

    visualizer->pushImageTexture(img2.data, img2.cols, img2.rows,
                                 img2.channels());

    visualizer->Start();
    visualizer->Join();
}