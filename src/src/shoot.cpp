#include <iostream>
#include <memory>
#include <string>
#include <ctime>
#include <signal.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "../include/uart.hpp"
#include "./detection/laser_detection.cpp"

void callbackSignal(int signum);//系统退出回调函数

shared_ptr<Driver> driver = nullptr;
shared_ptr<VideoCapture> capture = nullptr;


int main(int argc, char *argv[])
{
    LaserDetection laserDetection;

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

    // 摄像头初始化
    capture = std::make_shared<VideoCapture>();
	capture->open("/dev/video0");
	if(capture == nullptr)
	{
		std::cout << "Camera create failed!" << std::endl;
		return -1;
	}
	if(!capture->isOpened())
	{
		std::cout << "Camera open failed!" << std::endl;
		return -1;
	}
    capture->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture->set(cv::CAP_PROP_FPS, 30);
    capture->set(cv::CAP_PROP_FRAME_WIDTH, COLSIMAGE);
    capture->set(cv::CAP_PROP_FRAME_HEIGHT, ROWSIMAGE);
    capture->set(cv::CAP_PROP_EXPOSURE, 0.015);

    double rate = capture->get(cv::CAP_PROP_FPS);
    double width = capture->get(cv::CAP_PROP_FRAME_WIDTH);
    double height = capture->get(cv::CAP_PROP_FRAME_HEIGHT);
    double exposure = capture->get(cv::CAP_PROP_EXPOSURE);
    std::cout << "Camera Param: frame rate = " << rate << " width = " << width
              << " height = " << height << " exposure = " << exposure << " ms" << std::endl;

    for (int i = 3; i > 0; i--) // 3秒后发车
    {
		cout << "------------- " << i << " -----------" << endl;
		driver->gimbalControl(0, 0); // 智能车停止运动|建立下位机通信
		cv::waitKey(1000);
    }
    cout << "--------- System start!!! -------" << endl;


    while(1)
    {
		/*图像预处理*/
		cv::Mat frame;
		*capture >> frame;

        laserDetection.laserDetection(frame);

        cv::Mat imageLaser = cv::Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 初始化图像

        if(laserDetection.lasers.size() > 0)
        {
            driver->gimbalControl(laserDetection.lasers[0].x_delta, laserDetection.lasers[0].y_delta);
        }
        else
        {
            driver->gimbalControl(0, 0);
        }

        laserDetection.print();

        laserDetection.drawImage(imageLaser);
        cv::imshow("imageRecognition", imageLaser);
        cv::imshow("Frame", frame);

        if (waitKey(5) == 13) break;
    }

    return 0;
}

/**
 * @brief 系统信号回调函数：系统退出
 *
 * @param signum 信号量
 */
void callbackSignal(int signum)
{

    cout << "====System Exit!!!  -->  Stopping! " << signum << endl;
    exit(signum);
}

