#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>

#include "uart.hpp"
#include "common.hpp"

class SerialInterface
{
public:
    SerialInterface(std::string serial_path, LibSerial::BaudRate bps) 
    : _x_delta(0)
    , _y_delta(0)
    {
        _serial_path = serial_path;
        _bps = bps;
    }
    ~SerialInterface() {}

    void set_control(int16_t x_delta, int16_t y_delta)
    {
        _x_delta = x_delta;
        _y_delta = y_delta;
        _ctrl = true;
    }

    void buzzerSound(unsigned char sound)
    {
        if(_sound == 0)
        {
            _sound = sound;
        }
    }

    void set_PID(float Kp, float Ki, float Kd)
    {
        _driver->PID_init(Kp, Ki, Kd);
    }

    float get_speed()
    {
        return _recvSpeed;
    }

    int open()
    {
        return _open();
    }

    void Start()
    {
        _loop = true;
        send();
        // recv();
    }

    void Stop()
    {
        _loop = false;
        if (_thread_send && _thread_send->joinable())
            _thread_send->join();
        if (_thread_recv && _thread_recv->joinable())
            _thread_recv->join();
        if (_driver)
            _driver->close();

        std::cout << "serial exit" << std::endl;
    }


    void send()
    {
        _thread_send = std::make_unique<std::thread>([this]{
            _driver->gimbalControl(0, 0);
            while(_loop)
            {
                if(_sound)
                {
                    _driver->buzzerSound(_sound);
                    _sound = 0;
                }
                // if(_ctrl)
                // {
                //     _driver->gimbalControl(_x_delta, _y_delta);
                //     _ctrl = false;
                // }
                _driver->gimbalControl(_x_delta, _y_delta);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));

            }
        });
    }

    void recv()
    {
        _thread_recv = std::make_unique<std::thread>([this]{
            while(_loop)
            {
                uint8_t data_addr;
                data_addr = _driver->receiveData();
                if(data_addr == 0x08)
                {
                    _recvSpeed = _driver->speed_unpack();
                }
                else if(data_addr == 0x06)
                {
                    _carSignal = true;
                }
                // else
                // {
                //     std::this_thread::sleep_for(std::chrono::milliseconds(8));
                // }
            }
        });
    }

private:
    int _open()
    {
        _driver = std::make_shared<Driver>(_serial_path, _bps);
        if (_driver == nullptr)
        {
            std::cout << "Create uart-driver error!" << std::endl;
            return -1;
        }
        // 串口初始化，打开串口设备及配置串口数据格式
        int ret = _driver->open();
        if (ret != 0)
        {
            std::cout << "Uart open failed!" << std::endl;
            return -1;
        }
        return 0;
    }
    bool _loop;
    std::string _serial_path;
    LibSerial::BaudRate _bps;
    std::shared_ptr<Driver> _driver;
    unsigned char _sound;

    bool _ctrl = false;
    bool _carSignal = false;
    float _x_delta;
    float _y_delta;
    float _recvSpeed;

    std::mutex _mutex;
    std::condition_variable cond_;
    std::unique_ptr<std::thread> _thread_send;
    std::unique_ptr<std::thread> _thread_recv;
};