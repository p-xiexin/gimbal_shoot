#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include "../include/common.hpp"	//公共类方法文件

using namespace std;
using namespace cv;

int kernel_size = 3;    // 滤波核大小
int thresh_high = 150;  // canny边缘检测的高阈值
int vote_cnt = 10;      // 确定检测到圆的最小投票数

int main()
{
    Mat image;
    std::string indexCapture = "/dev/video0";
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
    capture.set(cv::CAP_PROP_EXPOSURE, 0.015);

    double rate = capture.get(CAP_PROP_FPS);
    double width = capture.get(CAP_PROP_FRAME_WIDTH);
    double height = capture.get(CAP_PROP_FRAME_HEIGHT);
    double exposure = capture.get(CAP_PROP_EXPOSURE);
    std::cout << "Camera Param: frame rate = " << rate << " width = " << width
              << " height = " << height << " exposure = " << exposure << " ms" << std::endl;

    cv::namedWindow("Params Setting");
    // 创建滑块
    cv::createTrackbar("Kernel Size", "Params Setting", &kernel_size, 10);
    cv::createTrackbar("Thresh High", "Params Setting", &thresh_high, 200);
    cv::createTrackbar("Vote Cnt", "Params Setting", &vote_cnt, 20);

    while (1)
    {
        Mat frame;
        if (!capture.read(frame))
        {
            std::cout << "no video frame" << std::endl;
            continue;
        }

        // 转换为灰度图像
        cv::Mat grayImage;
        cv::cvtColor(frame, grayImage, cv::COLOR_BGR2GRAY);

        Mat kernel = getStructuringElement(MORPH_RECT, Size(kernel_size, kernel_size));//创建结构元
        morphologyEx(grayImage, grayImage, MORPH_CLOSE, kernel, Point(-1, -1));//闭运算

        // 对灰度图像进行边缘检测
        cv::Mat edges;
        cv::Canny(grayImage, edges, 50, 150);

        // 使用霍夫变换检测圆
        vector<Vec3f> circles;
        HoughCircles(grayImage, circles, HOUGH_GRADIENT, 1, edges.rows / 8, thresh_high, vote_cnt, 1, 15);

        // 绘制检测到的圆
        if (!circles.empty())
        {
            for (size_t i = 0; i < circles.size(); i++)
            {
                Vec3i c = circles[i];
                Point center = Point(c[0], c[1]);
                int radius = c[2];
                // // 绘制圆心
                // circle(frame, center, 3, Scalar(0, 255, 0), -1, 8, 0);
                // 绘制圆轮廓
                circle(frame, center, radius, Scalar(0, 0, 255), 3, 8, 0);
                cout << "Point " << i << ": (" << center.x << ", " << center.y << ")  ";
            }cout << endl;
        }

        // 创建一个空白图像作为左右两幅图像的间隔
        cv::Mat blank = cv::Mat::zeros(frame.rows, 10, CV_8UC3);

        // 将滑动条和frame图像水平拼接
        cv::Mat combined1;
        cv::hconcat(frame, blank, combined1);

        // 将 edges 转换为3通道图像
        cv::Mat edges_color;
        cv::cvtColor(edges, edges_color, cv::COLOR_GRAY2BGR);

        // 将combined1和edges图像水平拼接
        cv::Mat combined2;
        cv::hconcat(combined1, edges_color, combined2);

        // 显示结果
        cv::imshow("Params Setting", combined2);
        if (waitKey(5) == 13) break;

        // 更新滑块显示的当前值
        cv::setTrackbarPos("Kernel Size", "Params Setting", kernel_size);
        cv::setTrackbarPos("Thresh High", "Params Setting", thresh_high);
        cv::setTrackbarPos("Vote Cnt", "Params Setting", vote_cnt);
    }

    return 0;
}
