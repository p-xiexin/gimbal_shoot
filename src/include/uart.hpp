#pragma once

#include <iostream>
#include <libserial/SerialPort.h>
#include <string.h>
#include "common.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace LibSerial;

#define UsbFrameHead 0x42	 // USB通信帧头
#define UsbFrameLengthMin 4	 // USB通信帧最短长度（字节）
#define UsbFrameLengthMax 30 // USB通信帧最长长度（字节）

typedef union
{
	uint8_t U8_Buff[2];
	uint16_t U16;
} Buint16_Union;

typedef union
{
	uint8_t U8_Buff[2];
	int16_t U16;
} Bint16_Union;

typedef union
{
	uint8_t U8_Buff[4];
	float Float;
} Bint32_Union;

typedef struct
{
	bool receiveStart;								// 数据接收开始标志
	uint8_t receiveIndex;							// 接收序列号
	bool receiveFinished;							// 接收并校验成功
	uint8_t receiveBuff[UsbFrameLengthMax];			// 临时接收数据区
	uint8_t receiveBuffFinished[UsbFrameLengthMax]; // 校验完成数据区
} Usb_Struct;

class Driver
{

private:
	std::shared_ptr<SerialPort> _serial_port = nullptr;
	std::string _port_name; // 端口名字
	BaudRate _bps;			// 波特率
	bool isOpen = false;
	Usb_Struct usb_Struct;

private:
	int recv(unsigned char &charBuffer, size_t msTimeout = 0)
	{
		/*try检测语句块有没有异常。如果没有发生异常,就检测不到。
		如果发生异常，則交给 catch 处理，执行 catch 中的语句* */
		try
		{
			/*从串口读取一个数据,指定msTimeout时长内,没有收到数据，抛出异常。
			如果msTimeout为0，则该方法将阻塞，直到数据可用为止。*/
			_serial_port->ReadByte(charBuffer, msTimeout); // 可能出现异常的代码段
		}
		catch (const ReadTimeout &) // catch捕获并处理 try 检测到的异常。
		{
			// std::cerr << "The ReadByte() call has timed out." << std::endl;
			return -2;
		}
		catch (const NotOpen &) // catch()中指明了当前 catch 可以处理的异常类型
		{
			std::cerr << "Port Not Open ..." << std::endl;
			return -1;
		}
		return 0;
	};

	int send(unsigned char charbuffer)
	{

		// try检测语句块有没有异常
		try
		{
			_serial_port->WriteByte(charbuffer); // 写数据到串口
		}
		catch (const std::runtime_error &) // catch捕获并处理 try 检测到的异常。
		{
			std::cerr << "The Write() runtime_error." << std::endl;
			return -2;
		}
		catch (const NotOpen &) // catch捕获并处理 try 检测到的异常。
		{
			std::cerr << "Port Not Open ..." << std::endl;
			return -1;
		}
		_serial_port->DrainWriteBuffer(); // 等待，直到写缓冲区耗尽，然后返回。
		return 0;
	}

	int send(const std::vector<unsigned char>& data)
	{
		// try-catch代码块

		try
		{
			_serial_port->Write(data);
		}
		catch (const std::runtime_error&)
		{
			std::cerr << "The Write() runtime_error." << std::endl;
			return -2;
		}
		catch (const NotOpen&)
		{
			std::cerr << "Port Not Open ..." << std::endl;
			return -1;
		}

		// _serial_port->DrainWriteBuffer(); // 非阻塞模式下不需要等待缓冲区耗尽

		return 0;
	}
public:
	// 定义构造函数
	Driver(const std::string &port_name, BaudRate bps)
		: _port_name(port_name), _bps(bps){};
	// 定义析构函数
	~Driver() { close(); };

public:
	int open()
	{
		_serial_port = std::make_shared<SerialPort>();
		if (_serial_port == nullptr)
		{
			std::cerr << "Serial Create Failed ." << std::endl;
			return -1;
		}
		// try检测语句块有没有异常
		try
		{
			_serial_port->Open(_port_name);								  // 打开串口
			_serial_port->SetBaudRate(_bps);							  // 设置波特率
			_serial_port->SetCharacterSize(CharacterSize::CHAR_SIZE_8);	  // 8位数据位
			_serial_port->SetFlowControl(FlowControl::FLOW_CONTROL_NONE); // 设置流控
			_serial_port->SetParity(Parity::PARITY_NONE);				  // 无校验
			_serial_port->SetStopBits(StopBits::STOP_BITS_1);			  // 1个停止位
		}
		catch (const OpenFailed &) // catch捕获并处理 try 检测到的异常。
		{
			std::cerr << "Serial port: " << _port_name << "open failed ..."
					  << std::endl;

			isOpen = false;
			return -2;
		}
		catch (const AlreadyOpen &) // catch捕获并处理 try 检测到的异常。
		{
			std::cerr << "Serial port: " << _port_name << "open failed ..."
					  << std::endl;
			isOpen = false;
			return -3;
		}
		catch (...) // catch捕获并处理 try 检测到的异常。
		{
			std::cerr << "Serial port: " << _port_name << " recv exception ..."
					  << std::endl;
			isOpen = false;
			return -4;
		}

		usb_Struct.receiveStart = false;
		usb_Struct.receiveIndex = 0;
		usb_Struct.receiveFinished = false;

		isOpen = true;
		return 0;
	};

	int senddata(unsigned char charbuffer) { return send(charbuffer); }
	int recvdata(unsigned char &charBuffer, size_t msTimeout)
	{
		return recv(charBuffer, msTimeout);
	}

	/**
	 * @brief 智能车速度pid设置
	 *
	 * @param 略
	 */
	void PID_init(float Kp, float Ki, float Kd)
	{
		if (isOpen)
		{
			Bint32_Union Kp_Union;
			Bint32_Union Ki_Union;
			Bint32_Union Kd_Union;
			unsigned char sendBuff[16];
			unsigned char check = 0;

			Kp_Union.Float = Kp;
			Ki_Union.Float = Ki;
			Kd_Union.Float = Kd;

			sendBuff[0] = 0x42;
			sendBuff[1] = 0x10;
			sendBuff[2] = 16;

			sendBuff[3] = Kp_Union.U8_Buff[0];
			sendBuff[4] = Kp_Union.U8_Buff[1];
			sendBuff[5] = Kp_Union.U8_Buff[2];
			sendBuff[6] = Kp_Union.U8_Buff[3];

			sendBuff[7] = Ki_Union.U8_Buff[0];
			sendBuff[8] = Ki_Union.U8_Buff[1];
			sendBuff[9] = Ki_Union.U8_Buff[2];
			sendBuff[10] = Ki_Union.U8_Buff[3];

			sendBuff[11] = Kd_Union.U8_Buff[0];
			sendBuff[12] = Kd_Union.U8_Buff[1];
			sendBuff[13] = Kd_Union.U8_Buff[2];
			sendBuff[14] = Kd_Union.U8_Buff[3];

			for (size_t i = 0; i < 15; i++)
			{
				check += sendBuff[i];
			}
			sendBuff[15] = check;

			// 循环发送数据
			for (size_t i = 0; i < 16; i++)
			{
				send(sendBuff[i]);
			}
		}
		else
		{
			std::cout << "Error: Uart Open failed!!!!" << std::endl;
		}
	}

	void rectControl(std::vector<cv::Point> rectPoints)
	{
		if(rectPoints.size() != 5)
			return;

		if(isOpen)
		{
			std::vector<unsigned char> sendBuff(24);
			Bint16_Union bint16_data;
			unsigned char check = 0;

			sendBuff[0] = 0xa5; // 帧头
			sendBuff[1] = 0x07; // 地址
			sendBuff[2] = 24;	// 帧长

			for(int i = 0; i < 5; i++)
			{
				bint16_data.U16 = rectPoints[i].x;
				sendBuff[4*i + 3] = bint16_data.U8_Buff[0];
				sendBuff[4*i + 4] = bint16_data.U8_Buff[1];
				bint16_data.U16 = rectPoints[i].y;
				sendBuff[4*i + 5] = bint16_data.U8_Buff[0];
				sendBuff[4*i + 6] = bint16_data.U8_Buff[1];
			}

			for (size_t i = 0; i < 23; i++)
			{
				check += sendBuff[i];
			}
			sendBuff[23] = check;
			send(sendBuff);
		}
		else
		{
			std::cout << "Error: Uart Open failed!!!!" << std::endl;
		}
	}

	void circleControl(int16_t x, int16_t y)
	{
		if (isOpen)
		{
			Bint16_Union bint16_x;
			Bint16_Union bint16_y;
			std::vector<unsigned char> sendBuff(8);
			unsigned char check = 0;

			bint16_x.U16 = x;
			bint16_y.U16 = y;

			sendBuff[0] = 0xa5; // 帧头
			sendBuff[1] = 0x08; // 地址
			sendBuff[2] = 8;	// 帧长

			sendBuff[3] = bint16_x.U8_Buff[0];
			sendBuff[4] = bint16_x.U8_Buff[1];

			sendBuff[5] = bint16_y.U8_Buff[0];
			sendBuff[6] = bint16_y.U8_Buff[1];

			for (size_t i = 0; i < 7; i++)
			{
				check += sendBuff[i];
			}
			sendBuff[7] = check;
			// 发送数据
			send(sendBuff);
			}
		else
		{
			std::cout << "Error: Uart Open failed!!!!" << std::endl;
		}
	}

	/**
	 * @brief 智能车速度与方向控制
	 *
	 * @param x_delta x方向增量
	 * @param y_delta y方向增量
	 */
	void gimbalControl(int16_t x_delta, int16_t y_delta)
	{
		if (isOpen)
		{
			Bint16_Union bint16_x;
			Bint16_Union bint16_y;
			std::vector<unsigned char> sendBuff(8);
			unsigned char check = 0;

			bint16_x.U16 = x_delta;
			bint16_y.U16 = y_delta;

			sendBuff[0] = 0xa5; // 帧头
			sendBuff[1] = 0x02; // 地址
			sendBuff[2] = 8;	// 帧长

			sendBuff[3] = bint16_x.U8_Buff[0];
			sendBuff[4] = bint16_x.U8_Buff[1];

			sendBuff[5] = bint16_y.U8_Buff[0];
			sendBuff[6] = bint16_y.U8_Buff[1];

			for (size_t i = 0; i < 7; i++)
			{
				check += sendBuff[i];
			}
			sendBuff[7] = check;
			// 发送数据
			send(sendBuff);
			
			// for (size_t i = 0; i < 8; i++)
			// {
			// 	std::cout << setw(2) << setfill('0') << hex << static_cast<int>(sendBuff[i]) << "\t";
			// }
			// std::cout << std::endl;
		}
		else
		{
			std::cout << "Error: Uart Open failed!!!!" << std::endl;
		}
	}

	/**
	 * @brief 串口接收下位机信号
	 *
	 */
	uint8_t receiveData(void)
	{
		uint8_t resByte;
		int ret = 0;

		ret = recvdata(resByte, 30);
		if (ret == 0)
		{
			if (resByte == UsbFrameHead && !usb_Struct.receiveStart) // 帧头检测
			{
				usb_Struct.receiveStart = true;
				usb_Struct.receiveFinished = false;
				usb_Struct.receiveBuff[0] = resByte;
				usb_Struct.receiveBuff[2] = UsbFrameLengthMin;
				usb_Struct.receiveIndex = 1;
			}
			else if (usb_Struct.receiveIndex == 2) // 数据长度
			{
				usb_Struct.receiveBuff[usb_Struct.receiveIndex] = resByte;
				usb_Struct.receiveIndex++;

				if (resByte > UsbFrameLengthMax ||
					resByte < UsbFrameLengthMin) // 帧长校验
				{
					usb_Struct.receiveBuff[2] = UsbFrameLengthMin;
					usb_Struct.receiveIndex = 0;
					usb_Struct.receiveStart = false;
				}
			}
			else if (usb_Struct.receiveStart &&
					 usb_Struct.receiveIndex < UsbFrameLengthMax)
			{
				usb_Struct.receiveBuff[usb_Struct.receiveIndex] = resByte;
				usb_Struct.receiveIndex++;
			}

			// 帧接收完毕
			if ((usb_Struct.receiveIndex >= UsbFrameLengthMax ||
				 usb_Struct.receiveIndex >= usb_Struct.receiveBuff[2]) &&
				usb_Struct.receiveIndex > UsbFrameLengthMin)
			{
				uint8_t check = 0;
				uint8_t length = UsbFrameLengthMin;
				length = usb_Struct.receiveBuff[2];

				for (int i = 0; i < length - 1; i++)
					check += usb_Struct.receiveBuff[i];

				if (check == usb_Struct.receiveBuff[length - 1]) // 校验位
				{
					memcpy(usb_Struct.receiveBuffFinished, usb_Struct.receiveBuff,
						   UsbFrameLengthMax);
					usb_Struct.receiveFinished = true;
					return usb_Struct.receiveBuffFinished[1];
				}

				usb_Struct.receiveIndex = 0;
				usb_Struct.receiveStart = false;
			}
		}

		return 0;
	}

	void close()
	{
		if (_serial_port != nullptr)
		{
			/*关闭串口。串口的所有设置将会丢失，并且不能在串口上执行更多的I/O操作。*/
			_serial_port->Close();
			_serial_port = nullptr;
		}
	};
};
