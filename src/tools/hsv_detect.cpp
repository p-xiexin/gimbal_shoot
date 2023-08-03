
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include "../include/common.hpp"	//公共类方法文件

using namespace std;
using namespace cv;

std::vector<cv::Rect> searchBlocks(cv::Mat img_rgb, cv::Mat &mask, cv::Scalar lowerThreshold, cv::Scalar upperThreshold);
void onTrackbar(int, void*);

int kernel_size = 4;
int min_height = 3, min_width = 3;
int max_height = 20, max_width = 20;
// 红色色块
int r_lowB = 140, r_lowG = 0, r_lowR = 132;
int r_highB = 180, r_highG = 255, r_highR = 255;
cv::Scalar lowerThreshold_r(r_lowB, r_lowG, r_lowR);
cv::Scalar upperThreshold_r(r_highB, r_highG, r_highR);

// 绿色色块
int g_lowB = 74, g_lowG = 0, g_lowR = 101;
int g_highB = 94, g_highG = 255, g_highR = 255;
cv::Scalar lowerThreshold_g(g_lowB, g_lowG, g_lowR);
cv::Scalar upperThreshold_g(g_highB, g_highG, g_highR);


bool colorThresholdSelected = true;

int main(int argc, char const *argv[])
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


    cv::namedWindow("Red Threshold Settings");
    cv::createTrackbar("Low H", "Red Threshold Settings", &r_lowB, 180, onTrackbar);
    cv::createTrackbar("High H", "Red Threshold Settings", &r_highB, 180, onTrackbar);
    cv::createTrackbar("Low S", "Red Threshold Settings", &r_lowG, 255, onTrackbar);
    cv::createTrackbar("High S", "Red Threshold Settings", &r_highG, 255, onTrackbar);
    cv::createTrackbar("Low V", "Red Threshold Settings", &r_lowR, 255, onTrackbar);
    cv::createTrackbar("High V", "Red Threshold Settings", &r_highR, 255, onTrackbar);

	cv::namedWindow("Green Threshold Settings");
    cv::createTrackbar("Low H", "Green Threshold Settings", &g_lowB, 180, onTrackbar);
    cv::createTrackbar("High H", "Green Threshold Settings", &g_highB, 180, onTrackbar);
    cv::createTrackbar("Low S", "Green Threshold Settings", &g_lowG, 255, onTrackbar);
    cv::createTrackbar("High S", "Green Threshold Settings", &g_highG, 255, onTrackbar);
    cv::createTrackbar("Low V", "Green Threshold Settings", &g_lowR, 255, onTrackbar);
    cv::createTrackbar("High V", "Green Threshold Settings", &g_highR, 255, onTrackbar);

    cv::namedWindow("Result");
    cv::createTrackbar("Kernel Size: ", "Result", &kernel_size, 10, onTrackbar);
    cv::createTrackbar("Min Height: ", "Result", &min_height, 20, onTrackbar);
    cv::createTrackbar("Min Width: ", "Result", &min_width, 20, onTrackbar);
    cv::createTrackbar("Max Height: ", "Result", &max_height, 20, onTrackbar);
    cv::createTrackbar("Max Width: ", "Result", &max_width, 20, onTrackbar);


	while (1)
	{
		{
			static auto preTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
			auto startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
			float detFPS = (float)1000.f / (startTime - preTime);
			std::cout << "run frame time : " << startTime - preTime << "ms  " << "FPS: " << (int)detFPS << std::endl;
			preTime = startTime;
		}
		Mat frame;
		if (!capture.read(frame))
		{
			std::cout << "no video frame" << std::endl;
			continue;
		}
		Mat mask_red, mask_green;

		if(colorThresholdSelected)
		{
			std::vector<cv::Rect> coneRects_red = searchBlocks(frame, mask_red, lowerThreshold_r, upperThreshold_r);
			for(const auto& rect : coneRects_red)
			{
				if(rect.height > min_height && rect.height < max_height && rect.width > min_width && rect.width < max_width)
				{
					cv::rectangle(frame, rect, cv::Scalar(0, 0, 255), 2);
				}
			}
			std::vector<cv::Rect> coneRects_green = searchBlocks(frame, mask_green, lowerThreshold_g, upperThreshold_g);
			for(const auto& rect : coneRects_green)
			{
				if(rect.height > min_height && rect.height < max_height && rect.width > min_width && rect.width < max_width)
				{
					cv::rectangle(frame, rect, cv::Scalar(0, 255, 0), 2);
				}
			}
			cv::imshow("Result", frame);
        	cv::imshow("Red Threshold Settings", mask_red);
        	cv::imshow("Green Threshold Settings", mask_green);
			if(waitKey(5) == 13) break;
			// 更新滑块显示的当前值
			cv::setTrackbarPos("Low H", "Red Threshold Settings", r_lowB);
			cv::setTrackbarPos("Low S", "Red Threshold Settings", r_lowG);
			cv::setTrackbarPos("Low V", "Red Threshold Settings", r_lowR);
			cv::setTrackbarPos("High H", "Red Threshold Settings", r_highB);
			cv::setTrackbarPos("High S", "Red Threshold Settings", r_highG);
			cv::setTrackbarPos("High V", "Red Threshold Settings", r_highR);

			cv::setTrackbarPos("Low H", "Green  Threshold Settings", g_lowB);
			cv::setTrackbarPos("Low S", "Green  Threshold Settings", g_lowG);
			cv::setTrackbarPos("Low V", "Green  Threshold Settings", g_lowR);
			cv::setTrackbarPos("High H", "Green  Threshold Settings", g_highB);
			cv::setTrackbarPos("High S", "Green  Threshold Settings", g_highG);
			cv::setTrackbarPos("High V", "Green  Threshold Settings", g_highR);

			cv::setTrackbarPos("Kernel Size: ", "Result", kernel_size);
			cv::setTrackbarPos("Min Height: ", "Result", min_height);
			cv::setTrackbarPos("Min Width: ", "Result", min_width);
			cv::setTrackbarPos("Max Height: ", "Result", max_height);
			cv::setTrackbarPos("Max Width: ", "Result", max_width);
		}
		else
		{
			// 绘制方框在图像中心
            int centerX = frame.cols / 2;
            int centerY = frame.rows / 2;
            int boxSize = 20; // 方框的大小
            cv::Rect boxRect(centerX - boxSize / 2, centerY - boxSize / 2, boxSize, boxSize);

            // 在方框内提取颜色阈值参
            cv::Mat boxROI = frame(boxRect);
            cv::Scalar meanColor = cv::mean(boxROI);
            cv::rectangle(frame, boxRect, cv::Scalar(0, 255, 0), 2);

			// 绘制颜色阈值提示框在图像中心左下方
			int roiX = frame.cols / 2 - 10;
			int roiY = frame.rows / 2 + 10;
			cv::Rect roiRect(roiX- boxSize / 2, roiY - boxSize / 2, boxSize, boxSize);
			cv::rectangle(frame, roiRect, cv::Scalar(meanColor[0], meanColor[1], meanColor[2]), -1);

            cv::imshow("Select Color Threshold", frame);
			if(cv::waitKey(5) == 13)
			{
				// 设置锥桶颜色的RGB范围
				lowerThreshold_r = cv::Scalar(meanColor[0] - 10, meanColor[1] - 50, meanColor[2] - 50);
				upperThreshold_r = cv::Scalar(meanColor[0] + 10, meanColor[1] + 50, meanColor[2] + 50);
				colorThresholdSelected = true;
				cout << "Selected Thresholds: " << lowerThreshold_r << " - " << upperThreshold_r << endl;
			}
		}
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
    lowerThreshold_r = cv::Scalar(r_lowB, r_lowG, r_lowR);
    upperThreshold_r = cv::Scalar(r_highB, r_highG, r_highR);
	lowerThreshold_g = cv::Scalar(g_lowB, g_lowG, g_lowR);
    upperThreshold_g = cv::Scalar(g_highB, g_highG, g_highR);
}
