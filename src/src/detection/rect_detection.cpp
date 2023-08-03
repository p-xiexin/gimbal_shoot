#pragma once
/**
 * @file laser_detection.cpp
 * @author pxx
 * @brief 矩形目标检测
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

class RectDetection
{
public:
    RectDetection()
    {
        ;
    }
    ~RectDetection() {}


private:
    /**
     * @brief 加载配置参数Json
     */
    void loadParams() 
    {
        string jsonPath = "../src/config/rect.json";
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
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, kernel_size); // 添加构造函数
    };
    Params params;
};
