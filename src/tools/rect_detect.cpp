#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include "../include/common.hpp" //公共类方法文件

using namespace std;
using namespace cv;

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

    cv::namedWindow("Detected Rectangles");

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

		// 对灰度图像进行边缘检测
		cv::Mat edges;
		// cv::Canny(grayImage, edges, 50, 150);
        cv::threshold(grayImage, edges, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

		// 查找轮廓
		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		// 找出符合条件的矩形轮廓
		std::vector<cv::Point> approx;
		for (size_t i = 0; i < contours.size(); i++)
		{
			// 使用多边形逼近来近似轮廓
			cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true) * 0.02, true);

			// 如果多边形有4个顶点，则认为是矩形
			if (approx.size() == 4)
			{
                // 获取轮廓的外接矩形
                Rect boundingRect = cv::boundingRect(approx);

                if(boundingRect.height < 40 || boundingRect.width < 40 || boundingRect.height > ROWSIMAGE - 10 || boundingRect.width > COLSIMAGE - 10)
                    continue;

                // 绘制外接矩形
                rectangle(frame, boundingRect, Scalar(255, 0, 0), 2);
				// 绘制矩形
				cv::polylines(frame, approx, true, cv::Scalar(0, 255, 0), 2);
				// 绘制四个角点
				for (size_t j = 0; j < approx.size(); j++)
				{
					cv::circle(frame, approx[j], 5, cv::Scalar(0, 0, 255), -1);
				}
			}
		}

		// 显示结果
        // 将 edges 转换为3通道图像
        cv::Mat edges_color;
        cv::cvtColor(edges, edges_color, cv::COLOR_GRAY2BGR);

        // 将combined1和edges图像水平拼接
        cv::Mat combined;
        cv::hconcat(frame, edges_color, combined);

        // 显示结果
        cv::imshow("Detected Rectangles", combined);
		if(waitKey(5) == 13) break;
	}

	return 0;
}
