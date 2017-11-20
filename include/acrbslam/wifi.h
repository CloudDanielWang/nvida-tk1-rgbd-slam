#ifndef WIFI_H
#define WIFI_H

#include "acrbslam/common_include.h"
#include "acrbslam/visual_odometry.h"
#include "acrbslam/config.h"
#include "acrbslam/frame.h"
#include "acrbslam/converter.h"
#include "acrbslam/data.h"


namespace acrbslam
{

class wifi_comu:public Frame, public Converter			//继承了frame类
{
	public:
		wifi_comu();		//wifi参数读取函数
		~wifi_comu();

		void wifi_init();		//WiFi初始化函数
		void wifi_init_uav();	//UAV端初始化函数
		void wifi_init_pc();	//PC端初始化函数

		int send_time;		//WiFi传送数据所需时间
	
		struct sockaddr_in Server_Addr;		//服务器，即接收端地址
		struct sockaddr_in Client_Addr;		//客户端，即发送端地址

		//
		int server_sock;
		int client_sock;

	protected:
		//parameters
		unsigned short int SERVER_PORT;		//服务器端口
		unsigned short int CLIENT_PORT;		//客户端端口

		const char* SERVER_IP;	//服务器IP
		const char* CLIENT_IP;	//客户端IP

		int SERVER_LEN;		//服务器地址长度
		int CLIENT_LEN;		//客户端地址长度


	public:
		Data ReceiveTCPDataServer( Data data);
		void SendTCPDataClient(Data data);



/****************************************************************************************/
//以下为旧的测试函数
		
		//void send_data(char *data, unsigned int num);		//WiFi发送数据函数
		//void send_data_new(Mat frame);			//WIFI 发送测试
		//int  receive_data(char *data, long unsigned int num);		//WiFi接受数据函数
		//Mat  receive_data_pc(Mat frame);			//wifi pc 接受新函数


		//void SendTCPDataClient(uchar* TCPData, int TCPDataSize);

		//int receive_data_server_readv(Mat *RGBframe_, Mat *Depthframe_, Mat *Transformation_);
		//int  receive_data_server_readv(Mat *RGBframe_, Mat *Depthframe_, Mat *Transformation_, uchar *End_Flag);
		//int  receive_data_server_readv(Mat *RGBframe_, Mat *Depthframe_, Mat *Transformation_, int *End_Flag);


		//void send_data_client_writev(Mat RGBframe, Mat Depthframe,  Mat Transformation );
		//void send_data_client_writev(Mat RGBframe, Mat Depthframe,  Mat Transformation, uchar End_Flag);
		//void send_data_client_writev(Mat RGBframe, Mat Depthframe,  Mat Transformation, int End_Flag);





};




}	//namespace acrbslam

#endif //WIFI_H