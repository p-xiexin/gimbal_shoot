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
};

class LaserDetection
{
public:
    Laser red_point;
    Laser green_point;
    
public:
    LaserDetection()
    {
        loadParams();
    }
    ~LaserDetection() {}

    /**
     * @brief 检测激光圆形光斑
     *
     * @param img 摄像头图片
     */
    void laserDetection(cv::Mat img)
    {
        red_point.center = cv::Point(0, 0);
        green_point.center = cv::Point(0, 0);


    }

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(cv::Mat &image)
    {
        // for(size_t i = 0; i < lasers.size(); i++)
        // {
        //     circle(image, lasers[i].center, lasers[i].radius, Scalar(0, 0, 255), 3, 8, 0);
        // }

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
        bool debug = true;
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, debug); // 添加构造函数
    };
    Params params;
};
