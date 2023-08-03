#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include "../include/common.hpp" //公共类方法文件

using namespace std;
using namespace cv;

int type = 0;
int canny_low = 50, canny_high = 150;
int sobel_scale = 1;
int min_height = 100, min_width = 100;
int dilate_size = 1;

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

    cv::namedWindow("Detected Rectangles");
    // 创建滑块
    cv::createTrackbar("Type", "Detected Rectangles", &type, 2);
    cv::createTrackbar("Min Height", "Detected Rectangles", &min_height, ROWSIMAGE);
    cv::createTrackbar("Min Width", "Detected Rectangles", &min_width, COLSIMAGE);
    cv::createTrackbar("Canny Low", "Detected Rectangles", &canny_low, 255);
    cv::createTrackbar("Canny High", "Detected Rectangles", &canny_high, 255);
    cv::createTrackbar("Sobel Scale", "Detected Rectangles", &sobel_scale, 100);
    cv::createTrackbar("Dilate Size", "Detected Rectangles", &dilate_size, 10);
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
        std::vector<std::vector<cv::Point>> rectVertices; // 存储矩形顶点坐标
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

                // 储存顶点坐标
                rectVertices.push_back(approx);
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
                cout << "Rectangle Center: (" << centerX << ", " << centerY << ")" << endl;
			}
		}

		// 显示结果
        // // 将 edges 转换为3通道图像
        // cv::Mat edges_color;
        // cv::cvtColor(edges, edges_color, cv::COLOR_GRAY2BGR);

        // // 将combined1和edges图像水平拼接
        // cv::Mat combined;
        // cv::hconcat(frame, edges_color, combined);

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
        cv::imshow("Detected Rectangles", combined2);
		if(waitKey(5) == 13) break;

		cv::setTrackbarPos("Type", "Detected Rectangles", type);
		cv::setTrackbarPos("Canny Low", "Detected Rectangles", canny_low);
		cv::setTrackbarPos("Canny High", "Detected Rectangles", canny_high);
		cv::setTrackbarPos("Sobel Scale", "Detected Rectangles", sobel_scale);
		cv::setTrackbarPos("Min Height", "Detected Rectangles", min_height);
		cv::setTrackbarPos("Min Width", "Detected Rectangles", min_width);
		cv::setTrackbarPos("Dilate Size", "Detected Rectangles", dilate_size);
	}

	return 0;
}
