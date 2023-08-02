#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../include/common.hpp"

using namespace std;
using namespace cv;

void drawFieldGrid(cv::Mat& img);
cv::Mat getHistogram(cv::Mat img, cv::Scalar color);

int main(int argc, char *argv[])
{
    VideoCapture capture("/dev/video0");
    if (!capture.isOpened())
    {
        std::cout << "can not open video device " << std::endl;
        return 1;
    }
    capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(cv::CAP_PROP_FPS, 30);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);
    //capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.20);  //自动曝光开关
    // capture.set(cv::CAP_PROP_ZOOM, 14);
    // capture.set(cv::CAP_PROP_PAN, 20);
    // capture.set(cv::CAP_PROP_XI_OFFSET_X, 1);
    // capture.set(cv::CAP_PROP_XI_OFFSET_Y, 1);

    double rate = capture.get(CAP_PROP_FPS);
    double width = capture.get(CAP_PROP_FRAME_WIDTH);
    double height = capture.get(CAP_PROP_FRAME_HEIGHT);
    std::cout << "Camera Param: frame rate = " << rate << " width = " << width
              << " height = " << height << std::endl;

    while (1)
    {
        Mat frame;
        if (!capture.read(frame))
        {
            std::cout << "no video frame" << std::endl;
            continue;
        }
        Mat imageBinary, imageGray;
        cvtColor(frame, imageGray, cv::COLOR_BGR2GRAY);
		threshold(imageGray, imageBinary, 0, 255, THRESH_OTSU); // OTSU二值化方法

        std::vector<cv::Mat> channels;
        split(frame, channels);
        Mat hist = getHistogram(imageGray, cv::Scalar(255, 255, 255));
        Mat hist_b = getHistogram(channels[0], cv::Scalar(255, 0, 0));
        Mat hist_g = getHistogram(channels[1], cv::Scalar(0, 255, 0));
        Mat hist_r = getHistogram(channels[2], cv::Scalar(0, 0, 255));

        // drawFieldGrid(frame);

        // 将 edges 转换为3通道图像
        cv::Mat binary_color;
        cv::cvtColor(imageBinary, binary_color, cv::COLOR_GRAY2BGR);

        Mat combined11;
        cv::hconcat(frame, binary_color, combined11);
        Mat combined12;
        cv::hconcat(combined11, hist, combined12);

        Mat combined21;
        cv::hconcat(hist_b, hist_g, combined21);
        Mat combined22;
        cv::hconcat(combined21, hist_r, combined22);

        Mat result;
        cv::vconcat(combined12, combined22, result);

        imshow("Result", result);
        if(waitKey(10) == 13) break;
    }
    capture.release();
}

// 绘制田字格：基准线
void drawFieldGrid(cv::Mat& img)
{
    // 绘制田字格：基准线
    uint16_t rows = ROWSIMAGE / 30; // 16
    uint16_t cols = COLSIMAGE / 32; // 20

    for (size_t i = 1; i < rows; i++)
    {
        line(img, Point(0, 30 * i), Point(img.cols - 1, 30 * i), Scalar(211, 211, 211), 1);
    }
    for (size_t i = 1; i < cols; i++)
    {
        if (i == cols / 2)
            line(img, Point(32 * i, 0), Point(32 * i, img.rows - 1), Scalar(0, 0, 255), 1);
        else
            line(img, Point(32 * i, 0), Point(32 * i, img.rows - 1), Scalar(211, 211, 211), 1);
    }
}

cv::Mat getHistogram(cv::Mat img, cv::Scalar color)
{
    int gray[256] = { 0 }; // 每个灰度值下的像素个数
    Mat hist = Mat::zeros(Size(256, 256), CV_8UC3); // 定义直方图画布，使用3通道图像

    if (img.channels() != 1)
        cvtColor(img, img, cv::COLOR_BGR2GRAY);

    // 统计每个灰度值的像素个数
    int val = 0;
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {
            val = img.at<uchar>(i, j);
            gray[val]++;
        }
    }
    // 自适应直方图的高
    int max = 0, min = 0;
    for (int i = 0; i < 256; i++)
    {
        max = (max > gray[i]) ? max : gray[i];
        min = (min > gray[i]) ? gray[i] : min;
    }
    for (int i = 0; i < 256; i++)
    {
        gray[i] = img.rows * (gray[i] - min) / (max - min); // 将直方图的高度映射到[0, img.rows]
    }

    // 绘制柱状图
    int histWidth = 256;
    int histHeight = 256;
    for (int i = 1; i < 256; i++)
    {
        cv::line(hist, cv::Point(histWidth - i, histHeight), cv::Point(histWidth - i, histHeight - gray[i]),
            color, 1, cv::LINE_AA); // 使用line函数绘制直方图
    }

    // 将直方图调整为输入图像的大小
    cv::resize(hist, hist, img.size());

    return hist;
}
