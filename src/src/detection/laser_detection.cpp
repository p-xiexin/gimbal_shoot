#pragma once
/**
 * @file laser_detection.cpp
 * @author pxx
 * @brief 激光目标检测
 * @version 0.1
 * @date 2023-05-17
 *
 */

#include "../../include/common.hpp"
#include "../../include/json.hpp"
#include <cmath>
#include <iostream>
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

struct Laser
{
    cv::Point center = cv::Point(0, 0);
    int radius = 0;
    cv::Vec3b bgr = {0, 0, 0};

    int16_t x_delta = 0;
    int16_t y_delta = 0;
};

class LaserDetection
{
public:
    std::vector<Laser> lasers;
    
public:
    LaserDetection()
    {
        loadParams();
    }
    ~LaserDetection() {}

    /**
     * @brief 使用霍夫变换检测激光圆形光斑
     *
     * @param img 摄像头图片
     */
    bool laserDetection(cv::Mat img)
    {
        lasers.clear();

        if(img.channels() > 1)
            cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params.kernel_size, params.kernel_size));//创建结构元
        cv::morphologyEx(img, img, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1));//闭运算

        // 使用霍夫变换检测圆
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(img, circles, cv::HOUGH_GRADIENT, 1, img.rows / 8, params.thresh_high, params.vote_cnt, 1, 15);

        if (!circles.empty())
        {
            for (size_t i = 0; i < circles.size(); i++)
            {
                Laser temp;
                Vec3i c = circles[i];
                temp.center = cv::Point(c[0], c[1]);
                temp.radius = c[2];

                // // 统计圆内BGR三通道的灰度平均值
                // int sumB = 0, sumG = 0, sumR = 0;
                // int count = 0;
                // for (int y = std::max(0, temp.center.y - temp.radius); y < std::min(img.rows, temp.center.y + temp.radius); y++)
                // {
                //     for (int x = std::max(0, temp.center.x - temp.radius); x < std::min(img.cols, temp.center.x + temp.radius); x++)
                //     {
                //         // 获取BGR像素值
                //         cv::Vec3b pixel = img.at<cv::Vec3b>(y, x);
                //         sumB += pixel[0];
                //         sumG += pixel[1];
                //         sumR += pixel[2];
                //         count++;
                //     }
                // }
                // if (count > 0)
                // {
                //     temp.bgr[0] = static_cast<uchar>(sumB / count);
                //     temp.bgr[1] = static_cast<uchar>(sumG / count);
                //     temp.bgr[2] = static_cast<uchar>(sumR / count);
                // }

                // 统计圆内BGR三通道的灰度极大值
                int sumB = 0, sumG = 0, sumR = 0;
                int count = 0;
                for (int y = std::max(0, temp.center.y - temp.radius); y < std::min(img.rows, temp.center.y + temp.radius); y++)
                {
                    for (int x = std::max(0, temp.center.x - temp.radius); x < std::min(img.cols, temp.center.x + temp.radius); x++)
                    {
                        // 获取BGR像素值
                        cv::Vec3b pixel = img.at<cv::Vec3b>(y, x);
                        if(sumB < pixel[0]) sumB = pixel[0];
                        if(sumG < pixel[1]) sumG = pixel[1];
                        if(sumR < pixel[2]) sumR = pixel[2];
                    }
                }
                temp.bgr[0] = static_cast<uchar>(sumB);
                temp.bgr[1] = static_cast<uchar>(sumG);
                temp.bgr[2] = static_cast<uchar>(sumR);

                temp.x_delta = (int16_t)ROWSIMAGE / 2 - (int16_t)temp.center.y;
                temp.y_delta = (int16_t)COLSIMAGE / 2 - (int16_t)temp.center.x;

                // 将Laser对象加入lasers向量中
                lasers.push_back(temp);                
            }
        }

        if (!lasers.empty())
            return true;
        else
            return false;
    }

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(cv::Mat &image)
    {
        for(size_t i = 0; i < lasers.size(); i++)
        {
            circle(image, lasers[i].center, lasers[i].radius, Scalar(0, 0, 255), 3, 8, 0);
        }

        // 获取图像中心点
        cv::Point center(image.cols / 2, image.rows / 2);

        // 绘制白色小十字
        int crossSize = 10; // 十字的大小
        cv::Scalar crossColor(255, 255, 255); // 白色

        // 水平线
        cv::line(image, cv::Point(center.x - crossSize, center.y), cv::Point(center.x + crossSize, center.y), crossColor, 2, cv::LINE_AA);
        // 垂直线
        cv::line(image, cv::Point(center.x, center.y - crossSize), cv::Point(center.x, center.y + crossSize), crossColor, 2, cv::LINE_AA);
    }

    /**
     * @brief 识别结果打印
     *
     */
    void print(void)
    {
        if (!lasers.empty())
        {
            for(size_t i = 0; i < lasers.size(); i++)
            {
                std::cout << "(" << lasers[i].center.x << ", " << lasers[i].center.y << ")  ";
                std::cout << "(" << (int)lasers[i].bgr[0] << ", " << (int)lasers[i].bgr[1] << ", " << (int)lasers[i].bgr[2] << ")  ";
                std::cout << "X_Delta: " << lasers[i].x_delta << "  Y_Delta: " << lasers[i].y_delta;
            }
            std::cout << std::endl;
        }
    }



private:
    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../src/config/laser.json";
        std::ifstream config_is(jsonPath);
        if (!config_is.good()) 
        {
            std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
            exit(-1);
        }

        nlohmann::json js_value;
        config_is >> js_value;

        try 
        {
            params = js_value.get<Params>();
        }
        catch (const nlohmann::detail::exception &e) 
        {
            std::cerr << "Json Params Parse failed :" << e.what() << '\n';
            exit(-1);
        }
    }

private:
    /**
     * @brief 核心参数
     *
     */
    struct Params
    {
        uint16_t kernel_size = 3;
        uint16_t thresh_high = 150;
        uint16_t vote_cnt = 10;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, kernel_size, thresh_high, vote_cnt); // 添加构造函数
    };
    Params params;
};
