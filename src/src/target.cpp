#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include "../include/common.hpp" //公共类方法文件
#include "../include/serial.hpp"

using namespace std;
using namespace cv;

int type = 1;
int canny_low = 10, canny_high = 50;   
int sobel_scale = 1;
int min_height = 120, min_width = 160;
int dilate_size = 11;

// 红色色块
int kernel_size = 2;
int min_size = 2;
int max_size = 30;
int r_lowH = 140, r_lowS = 0, r_lowV = 132;
int r_highH = 180, r_highS = 255, r_highV = 255;
cv::Scalar lowerThreshold_r(r_lowH, r_lowS, r_lowV);
cv::Scalar upperThreshold_r(r_highH, r_highS, r_highV);

std::vector<cv::Rect> searchBlocks(cv::Mat img_rgb, cv::Mat &mask, cv::Scalar lowerThreshold, cv::Scalar upperThreshold);
// 比较函数，用于按极角从小到大排序
bool comparePoints(cv::Point pt1, cv::Point pt2)
{
    cv::Point2f center(COLSIMAGE / 2.0, ROWSIMAGE / 2.0); // 图像中心点
    double angle1 = atan2(pt1.y - center.y, pt1.x - center.x);
    double angle2 = atan2(pt2.y - center.y, pt2.x - center.x);
    return angle1 < angle2;
}
shared_ptr<Driver> driver = nullptr;
int main()
{
    uint16_t counterRunBegin = 1;              // 启动计数器：等待摄像头图像帧稳定

	// USB转串口的设备名为 / dev/ttyUSB0
	driver = std::make_shared<Driver>("/dev/ttyUSB0", BaudRate::BAUD_115200);
	// driver = std::make_shared<Driver>("/dev/ttyACM0", BaudRate::BAUD_115200);
	if (driver == nullptr)
	{
		std::cout << "Create uart-driver error!" << std::endl;
		return -1;
	}
	// 串口初始化，打开串口设备及配置串口数据格式
	int ret = driver->open();
	if (ret != 0)
	{
		std::cout << "Uart open failed!" << std::endl;
		return -1;
	}

	std::string indexCapture = "/dev/video0";
	VideoCapture capture("/dev/video0");
	if (!capture.isOpened())
	{
		std::cout << "can not open video device " << std::endl;
		return 1;
	}
	capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	capture.set(cv::CAP_PROP_FPS, 60);
	capture.set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);
	capture.set(cv::CAP_PROP_EXPOSURE, 0.015);
	capture.set(cv::CAP_PROP_ZOOM, 15);

	double rate = capture.get(CAP_PROP_FPS);
	double width = capture.get(CAP_PROP_FRAME_WIDTH);
	double height = capture.get(CAP_PROP_FRAME_HEIGHT);
	double exposure = capture.get(CAP_PROP_EXPOSURE);
	std::cout << "Camera Param: frame rate = " << rate << " width = " << width
			  << " height = " << height << " exposure = " << exposure << " ms" << std::endl;

	while (1)
	{
		std::vector<cv::Point> points_red;

		Mat frame;
		if (!capture.read(frame))
		{
			std::cout << "no video frame" << std::endl;
			continue;
		}
		cv::Mat mask_red;
		std::vector<cv::Rect> coneRects_red = searchBlocks(frame, mask_red, lowerThreshold_r, upperThreshold_r);
		for(const auto& rect : coneRects_red)
		{
			if(rect.height > min_size && rect.height < max_size && rect.width > min_size && rect.width < max_size)
			{
				points_red.push_back(cv::Point(rect.x + rect.width / 2, rect.y + rect.height / 2));
				cv::rectangle(frame, rect, cv::Scalar(0, 0, 255), 2);
			}
		}
		if(points_red.size() == 1)
		{
			std::cout << "red point: " << "(" << points_red[0].x << ", " << points_red[0].y << ")  ";

			driver->circleControl(points_red[0].x, points_red[0].y);
		}
		else
			driver->circleControl(0, 0);


		// 转换为灰度图像
		cv::Mat grayImage;
		cv::cvtColor(frame, grayImage, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(grayImage, grayImage, cv::Size(5, 5), 0, 0);

		// 对灰度图像进行边缘检测
		cv::Mat edges;
		if(type == 1)
		{
			cv::Canny(grayImage, edges, canny_low, canny_high);
            putText(frame, "Canny", Point(40, 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		}
		else if(type == 2)
		{
			cv::Sobel(frame, edges, -1, 1, 1, 3, sobel_scale);
            putText(frame, "Sobel", Point(40, 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		}
		else if(type == 0)
		{
			cv::threshold(grayImage, edges, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
            putText(frame, "OTSU", Point(40, 40), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1);
		}

		if(edges.channels() > 1)
			cv::cvtColor(edges, edges, cv::COLOR_BGR2GRAY);

		cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_size, dilate_size));//创建结构元
		cv::morphologyEx(edges, edges, cv::MORPH_DILATE, kernel, cv::Point(-1, -1));//闭运算

		// 查找轮廓
		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		// 找出符合条件的矩形轮廓
		std::vector<cv::Point> approx;
		std::vector<cv::Point> rectPoints;
		uint16_t counter = 0;
		for (size_t i = 0; i < contours.size(); i++)
		{
			// 使用多边形逼近来近似轮廓
			cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true) * 0.02, true);

			// 如果多边形有4个顶点，则认为是矩形
			if (approx.size() == 4)
			{
                // 获取轮廓的外接矩形
                Rect boundingRect = cv::boundingRect(approx);

                if(boundingRect.height < min_height || boundingRect.width < min_width || boundingRect.height > ROWSIMAGE - 10 || boundingRect.width > COLSIMAGE - 10)
                    continue;

                // 绘制外接矩形
                rectangle(frame, boundingRect, Scalar(255, 0, 0), 2);
				// 绘制矩形
				cv::polylines(frame, approx, true, cv::Scalar(0, 255, 0), 2);

				std::sort(approx.begin(), approx.end(), comparePoints);

				// 绘制四个角点
				for (size_t j = 0; j < approx.size(); j++)
				{
					cv::circle(frame, approx[j], 5, cv::Scalar(0, 0, 255), -1);
                    // 输出角点坐标
                    cout << "Point " << j << ": (" << approx[j].x << ", " << approx[j].y << ")  ";
				}
                // 计算矩形中心坐标并输出
                int centerX = (approx[0].x + approx[1].x + approx[2].x + approx[3].x) / 4;
                int centerY = (approx[0].y + approx[1].y + approx[2].y + approx[3].y) / 4;
                cv::circle(frame, Point(centerX, centerY), 5, cv::Scalar(226, 43, 138), -1);
                cout << "Rectangle Center: (" << centerX << ", " << centerY << ")  " << counter++ << endl;

				{
					rectPoints.push_back(approx[0]);
					rectPoints.push_back(approx[1]);
					rectPoints.push_back(approx[2]);
					rectPoints.push_back(approx[3]);
					rectPoints.push_back(cv::Point(centerX, centerY));
					driver->rectControl(rectPoints);
				}
			}
		}


        // // 创建一个空白图像作为左右两幅图像的间隔
        // cv::Mat blank = cv::Mat::zeros(frame.rows, 10, CV_8UC3);
        // // 将滑动条和frame图像水平拼接
        // cv::Mat combined1;
        // cv::hconcat(frame, blank, combined1);
        // // 将 edges 转换为3通道图像
        // cv::Mat edges_color;
        // cv::cvtColor(edges, edges_color, cv::COLOR_GRAY2BGR);
        // // 将combined1和edges图像水平拼接
        // cv::Mat combined2;
        // cv::hconcat(combined1, edges_color, combined2);
        // // 显示结果
        cv::imshow("Detected Rectangles", frame);
		if(waitKey(5) == 13) break;
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

