#include "../include/uart.hpp"
#include <iostream>
#include "../include/serial.hpp"

using namespace std;

SerialInterface serialInterface("/dev/ttyACM0", LibSerial::BaudRate::BAUD_115200);
// SerialInterface serialInterface("/dev/ttyUSB0", LibSerial::BaudRate::BAUD_115200);
int main()
{
    // 下位机初始化通信
    int ret = serialInterface.open();
    if (ret != 0)
        return 0;
    {
        cout << "-------- 速度闭环控制 -------" << endl;
        // serialInterface.set_PID(9000, 12.5, 4.5);
    }
	serialInterface.Start();


    while(1)
    {
        int16_t x_delta = 0, y_delta = 0;
        std::cin >> x_delta >> y_delta;
		serialInterface.set_control(x_delta, y_delta);
    }
}
