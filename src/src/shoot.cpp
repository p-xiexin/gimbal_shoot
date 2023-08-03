
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include "../include/common.hpp"	//公共类方法文件
#include "../include/serial.hpp"

using namespace std;
using namespace cv;

std::vector<cv::Rect> searchBlocks(cv::Mat img_rgb, cv::Mat &mask, cv::Scalar lowerThreshold, cv::Scalar upperThreshold);
void onTrackbar(int, void*);

int kernel_size = 2;
int min_size = 2;
int max_size = 30;
// 红色色块
int r_lowH = 140, r_lowS = 0, r_lowV = 132;
int r_highH = 180, r_highS = 255, r_highV = 255;
cv::Scalar lowerThreshold_r(r_lowH, r_lowS, r_lowV);
cv::Scalar upperThreshold_r(r_highH, r_highS, r_highV);

// 绿色色块
int g_lowH = 74, g_lowS = 0, g_lowV = 140;
int g_highH = 94, g_highS = 255, g_highV = 255;
cv::Scalar lowerThreshold_g(g_lowH, g_lowS, g_lowV);
cv::Scalar upperThreshold_g(g_highH, g_highS, g_highV);

SerialInterface serialInterface("/dev/ttyACM0", LibSerial::BaudRate::BAUD_115200);
int main(int argc, char const *argv[])
{
    uint16_t counterRunBegin = 1;              // 启动计数器：等待摄像头图像帧稳定


    // 下位机初始化通信
    int ret = serialInterface.open();
    if (ret != 0)
        return 0;
	serialInterface.Start();


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


    cv::namedWindow("Red Threshold Settings");
    cv::createTrackbar("Low H", "Red Threshold Settings", &r_lowH, 180, onTrackbar);
    cv::createTrackbar("High H", "Red Threshold Settings", &r_highH, 180, onTrackbar);
    cv::createTrackbar("Low S", "Red Threshold Settings", &r_lowS, 255, onTrackbar);
    cv::createTrackbar("High S", "Red Threshold Settings", &r_highS, 255, onTrackbar);
    cv::createTrackbar("Low V", "Red Threshold Settings", &r_lowV, 255, onTrackbar);
    cv::createTrackbar("High V", "Red Threshold Settings", &r_highV, 255, onTrackbar);

	cv::namedWindow("Green Threshold Settings");
    cv::createTrackbar("Low H", "Green Threshold Settings", &g_lowH, 180, onTrackbar);
    cv::createTrackbar("High H", "Green Threshold Settings", &g_highH, 180, onTrackbar);
    cv::createTrackbar("Low S", "Green Threshold Settings", &g_lowS, 255, onTrackbar);
    cv::createTrackbar("High S", "Green Threshold Settings", &g_highS, 255, onTrackbar);
    cv::createTrackbar("Low V", "Green Threshold Settings", &g_lowV, 255, onTrackbar);
    cv::createTrackbar("High V", "Green Threshold Settings", &g_highV, 255, onTrackbar);

    cv::namedWindow("Result");
    cv::createTrackbar("Kernel Size: ", "Result", &kernel_size, 10, onTrackbar);
    cv::createTrackbar("Min Size: ", "Result", &min_size, 50, onTrackbar);
    cv::createTrackbar("Max Size: ", "Result", &max_size, 50, onTrackbar);


	while (1)
	{
		std::vector<cv::Point> points_red;
		std::vector<cv::Point> points_green;

		// {
			static auto preTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
			auto startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
			float detFPS = (float)1000.f / (startTime - preTime);
			// std::cout << "run frame time : " << startTime - preTime << "ms  " << "FPS: " << (int)detFPS << std::endl;
			preTime = startTime;
		// }
		Mat frame;
		if (!capture.read(frame))
		{
			std::cout << "no video frame" << std::endl;
			continue;
		}
		Mat mask_red, mask_green;

		std::vector<cv::Rect> coneRects_red = searchBlocks(frame, mask_red, lowerThreshold_r, upperThreshold_r);
		for(const auto& rect : coneRects_red)
		{
			if(rect.height > min_size && rect.height < max_size && rect.width > min_size && rect.width < max_size)
			{
				points_red.push_back(cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2));
				cv::rectangle(frame, rect, cv::Scalar(0, 0, 255), 2);
			}
		}
		std::vector<cv::Rect> coneRects_green = searchBlocks(frame, mask_green, lowerThreshold_g, upperThreshold_g);
		for(const auto& rect : coneRects_green)
		{
			if(rect.height > min_size && rect.height < max_size && rect.width > min_size && rect.width < max_size)
			{
				points_green.push_back(cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2));
				cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 2);
			}
		}

		if(counterRunBegin > 30)
		{
			if(points_red.size() == 1 && points_green.size() == 1)
			{
				int16_t delta_x = (int16_t)points_red[0].x - (int16_t)points_green[0].x;
				int16_t delta_y = (int16_t)points_red[0].y - (int16_t)points_green[0].y;
				std::cout << "(" << points_red[0].x << ", " << points_red[0].y << ")  ";
				std::cout << "(" << points_green[0].x << ", " << points_green[0].y << ")  ";
				std::cout << "X_Delta: " << delta_x << "  Y_Delta: " << delta_y << std::endl;

				serialInterface.set_control(delta_x, delta_y);
			}
			else
				serialInterface.set_control(0, 0);
		}
		else 
			counterRunBegin++;

		putText(frame, "FPS: " + formatDoble2String(detFPS, 2), Point(20, 20), FONT_HERSHEY_PLAIN, 2, Scalar(0, 0, 255), 1);                   // 车速

		cv::imshow("Result", frame);
		cv::imshow("Red Threshold Settings", mask_red);
		cv::imshow("Green Threshold Settings", mask_green);
		if(waitKey(5) == 13) break;
		// 更新滑块显示的当前值
		cv::setTrackbarPos("Low H", "Red Threshold Settings", r_lowH);
		cv::setTrackbarPos("Low S", "Red Threshold Settings", r_lowS);
		cv::setTrackbarPos("Low V", "Red Threshold Settings", r_lowV);
		cv::setTrackbarPos("High H", "Red Threshold Settings", r_highH);
		cv::setTrackbarPos("High S", "Red Threshold Settings", r_highS);
		cv::setTrackbarPos("High V", "Red Threshold Settings", r_highV);

		cv::setTrackbarPos("Low H", "Green  Threshold Settings", g_lowH);
		cv::setTrackbarPos("Low S", "Green  Threshold Settings", g_lowS);
		cv::setTrackbarPos("Low V", "Green  Threshold Settings", g_lowV);
		cv::setTrackbarPos("High H", "Green  Threshold Settings", g_highH);
		cv::setTrackbarPos("High S", "Green  Threshold Settings", g_highS);
		cv::setTrackbarPos("High V", "Green  Threshold Settings", g_highV);

		cv::setTrackbarPos("Kernel Size: ", "Result", kernel_size);
		cv::setTrackbarPos("Min Size: ", "Result", min_size);
		cv::setTrackbarPos("Max Size: ", "Result", max_size);
	}
	return 0;
}

std::vector<cv::Rect> searchBlocks(cv::Mat img_rgb, cv::Mat &mask, cv::Scalar lowerThreshold, cv::Scalar upperThreshold)
{
	std::vector<cv::Rect> coneRects;

    cv::Mat img_hsv;
    cv::cvtColor(img_rgb, img_hsv, cv::COLOR_BGR2HSV);

	// 在RGB图像中根据颜色范围提取锥桶区域
	cv::inRange(img_hsv, lowerThreshold, upperThreshold, mask);

	// 进行形态学操作，去除噪声并提取锥桶区域的轮廓
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
	cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// 查找最大轮廓
	size_t maxContourIndex = 0;
	double maxContourArea = 0.0;
	for (size_t i = 0; i < contours.size(); ++i)
	{
		double contourArea = cv::contourArea(contours[i]);
		if (contourArea > maxContourArea)
		{
			maxContourArea = contourArea;
			maxContourIndex = i;
		}
	}

	// 绘制正方形框选中锥桶区域
	for (const auto& contour : contours)
	{
		cv::Rect boundingRect = cv::boundingRect(contour);
		coneRects.push_back(boundingRect);
	}	
	return coneRects;
}

void onTrackbar(int, void*)
{
    // 从滑块更新阈值参数
    lowerThreshold_r = cv::Scalar(r_lowH, r_lowS, r_lowV);
    upperThreshold_r = cv::Scalar(r_highH, r_highS, r_highV);
	lowerThreshold_g = cv::Scalar(g_lowH, g_lowS, g_lowV);
    upperThreshold_g = cv::Scalar(g_highH, g_highS, g_highV);
}
