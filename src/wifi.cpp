#include "acrbslam/wifi.h"


namespace acrbslam
{


//wifi 参数读取函数
wifi_comu::wifi_comu()
:send_time(0)
{	
	SERVER_PORT    = Config::get<int> ( "SERVER_PORT" );
	CLIENT_PORT	= Config::get<int> ( "CLIENT_PORT" );

	string  server_IP=Config::get<string> ( "SERVER_IP" );
	SERVER_IP=server_IP.c_str();
	string  client_IP=(Config::get<string> ( "CLIENT_IP" ));
	CLIENT_IP=client_IP.c_str();
}

//wifi释放函数
wifi_comu::~wifi_comu()
{

}


//wifi初始化函数
//根据CSDN 博客书写
void wifi_comu::wifi_init_uav()
{

	if((server_sock=socket(AF_INET,SOCK_STREAM,0))<0) //建立socket	//注意为STREAM类型
	{
		printf("socket error\n");
	}	

	Server_Addr.sin_family = AF_INET;
	Server_Addr.sin_port =htons(SERVER_PORT);		//此处端口为地面站，即接收端的端口号，IP也是
	Server_Addr.sin_addr.s_addr = inet_addr(SERVER_IP);
	SERVER_LEN=sizeof(Server_Addr);

	if((connect(server_sock,(struct sockaddr *)&Server_Addr,SERVER_LEN))<0)
	{
		perror("ERROR connecting");
		close(server_sock);
		exit(1);
	}
	
		

	return;
	

}

//wif函数接收端初始化
void wifi_comu::wifi_init_pc()
{
	if((server_sock=socket(AF_INET,SOCK_STREAM,0))<0) //建立socket
	{
		printf("socket error\n");
	}	

	Server_Addr.sin_family = AF_INET;
	Server_Addr.sin_port = htons(SERVER_PORT);
	Server_Addr.sin_addr.s_addr =htonl(INADDR_ANY);// inet_addr(CLIENT_IP);
	SERVER_LEN=sizeof(Server_Addr);

	if(bind(server_sock,(struct sockaddr *)&Server_Addr,SERVER_LEN)<0)		//此处为本地IP与端口
	{	
		printf("bind error\n");
		perror("wifi_bind");
		return ;
	}	

	if(listen(server_sock,5)==-1)
	{
		printf("listen error\n");
		return;
	}

	socklen_t addr_len=sizeof(Client_Addr);

	cout<<"Wait.."<<endl;
	do
	{
		client_sock=accept(server_sock,(struct sockaddr*)&Client_Addr,&addr_len);
	}while(client_sock<0);

	return;
}






//

void wifi_comu::send_data_client_writev(Mat RGBframe, Mat Depthframe,  Mat Transformation)
{
	const int rgbimgSize = RGBframe.total()*RGBframe.elemSize();
	//cout<<"send rgbimgSize"<<rgbimgSize<<endl;
	const int depthSize = Depthframe.total()*Depthframe.elemSize();
	//cout<<"send depthimgSize"<<depthSize<<endl;
	const int transformationSize = Transformation.total()*Transformation.elemSize();

	struct iovec frame_data[3];
           	frame_data[0].iov_base=RGBframe.data;
           	frame_data[0].iov_len=rgbimgSize;
           	frame_data[1].iov_base=Depthframe.data;
           	frame_data[1].iov_len=depthSize;
           	frame_data[2].iov_base=Transformation.data;
           	frame_data[2].iov_len=transformationSize;

           	ssize_t  sizeof_send_rgb_frame;
	sizeof_send_rgb_frame=writev(server_sock,frame_data, 3);
	if(sizeof_send_rgb_frame==(rgbimgSize+depthSize+transformationSize))
		cout<<"Send Success!!"<<endl;
	//cout<<"sizeof_send_rgb_frame"<<sizeof_send_rgb_frame<<endl;


	return;
		
}

//void wifi_comu::send_data_client_writev(Mat RGBframe, Mat Depthframe,  Mat Transformation, uchar End_Flag)
void wifi_comu::send_data_client_writev(Mat RGBframe, Mat Depthframe,  Mat Transformation, int End_Flag)
{
	const int rgbimgSize = RGBframe.total()*RGBframe.elemSize();
	//cout<<"send rgbimgSize"<<rgbimgSize<<endl;
	const int depthSize = Depthframe.total()*Depthframe.elemSize();
	//cout<<"send depthimgSize"<<depthSize<<endl;
	const int transformationSize = Transformation.total()*Transformation.elemSize();

	const int flagSize = sizeof(End_Flag);


	struct iovec frame_data[4];
           	frame_data[0].iov_base=RGBframe.data;
           	frame_data[0].iov_len=rgbimgSize;
           	frame_data[1].iov_base=Depthframe.data;
           	frame_data[1].iov_len=depthSize;
           	frame_data[2].iov_base=Transformation.data;
           	frame_data[2].iov_len=transformationSize;
           	frame_data[3].iov_base=(char *)&End_Flag;
           	frame_data[3].iov_len=flagSize;

           	ssize_t  sizeof_send_rgb_frame;
	sizeof_send_rgb_frame=writev(server_sock,frame_data, 4);
	if(sizeof_send_rgb_frame==(rgbimgSize+depthSize+transformationSize+flagSize))
		cout<<"Send Success!!"<<endl;
	//cout<<"sizeof_send_rgb_frame"<<sizeof_send_rgb_frame<<endl;


	return;
		
}



int  wifi_comu::receive_data_server_readv(Mat *RGBframe_, Mat *Depthframe_, Mat *Transformation_)
{
	
	Mat RGBframe=*RGBframe_;
	Mat Depthframe=*Depthframe_;
	Mat  Transformation=*Transformation_;

	const int rgbimgSize = RGBframe.total()*RGBframe.elemSize();
	 uchar RGBData[rgbimgSize];
	 //memset(RGBData,0,rgbimgSize);


	const int depthSize = Depthframe.total()*Depthframe.elemSize();
	uchar DepthData[depthSize];
	//memset(DepthData,0,depthSize);

	const int transformationSize = Transformation.total()*Transformation.elemSize();
	uchar TransformationData[transformationSize];

	 struct iovec frame_data[3];
           	frame_data[0].iov_base=RGBData;
           	frame_data[0].iov_len=rgbimgSize;
           	frame_data[1].iov_base=DepthData;
           	frame_data[1].iov_len=depthSize;
           	frame_data[2].iov_base=TransformationData;
           	frame_data[2].iov_len=transformationSize;

           	//cout<<"iovec size"<<sizeof(RGBData)+sizeof(DepthData)<<endl;

           	ssize_t bytes=0;

	bytes=readv(client_sock,frame_data, 3 );
	//cout<<"size of bytes"<<bytes<<endl;


	if (bytes==-1)
	{
		cout<<"Receive Fault! "<<endl;
		close(client_sock);
		exit(-1);
	}
	else
	if (bytes<(rgbimgSize+depthSize+transformationSize)&&bytes>0) 
	//if ((bytes<rgbimgSize)&&bytes>0) 
	{
		//cout<<"this received data is too small!!"<<endl;
		return 0;
	}
	else if (bytes==(rgbimgSize+depthSize+transformationSize)) 
		cout<<"Receive Success!!!"<<endl;
	
	Mat temp_RGBmat(Size(640,480), CV_8UC3, RGBData);
	Mat temp_DEPTHmat(Size(640,480), CV_16UC1, DepthData);
	Mat temp_Transmat(Size(4,4),CV_32F,TransformationData);

	
	*RGBframe_=temp_RGBmat;
	*Depthframe_=temp_DEPTHmat;
	*Transformation_=temp_Transmat;

	return 1;
}


//int  wifi_comu::receive_data_server_readv(Mat *RGBframe_, Mat *Depthframe_, Mat *Transformation_, uchar *End_Flag)
int  wifi_comu::receive_data_server_readv(Mat *RGBframe_, Mat *Depthframe_, Mat *Transformation_, int *End_Flag)
{
	
	Mat RGBframe=*RGBframe_;
	Mat Depthframe=*Depthframe_;
	Mat  Transformation=*Transformation_;

	const int rgbimgSize = RGBframe.total()*RGBframe.elemSize();
	 uchar RGBData[rgbimgSize];
	 //memset(RGBData,0,rgbimgSize);

	const int depthSize = Depthframe.total()*Depthframe.elemSize();
	uchar DepthData[depthSize];
	//memset(DepthData,0,depthSize);

	const int transformationSize = Transformation.total()*Transformation.elemSize();
	uchar TransformationData[transformationSize];

	const int flagSize = sizeof(End_Flag);
	uchar End_Flag_Data[flagSize];

	 struct iovec frame_data[4];
           	frame_data[0].iov_base=RGBData;
           	frame_data[0].iov_len=rgbimgSize;
           	frame_data[1].iov_base=DepthData;
           	frame_data[1].iov_len=depthSize;
           	frame_data[2].iov_base=TransformationData;
           	frame_data[2].iov_len=transformationSize;
           	frame_data[3].iov_base=End_Flag_Data;
           	frame_data[3].iov_len=flagSize;

           	//cout<<"iovec size"<<sizeof(RGBData)+sizeof(DepthData)<<endl;

           	ssize_t bytes=0;

	bytes=readv(client_sock,frame_data, 4 );
	//cout<<"size of bytes"<<bytes<<endl;


	if (bytes==-1)
	{
		cout<<"Receive Fault! "<<endl;
		close(client_sock);
		exit(-1);
	}
	else
	if ((bytes<(rgbimgSize+depthSize+transformationSize+flagSize))&&(bytes>0)) 
	{
		//cout<<"this received data is too small!!"<<endl;
		return 0;
	}
	else if (bytes==(rgbimgSize+depthSize+transformationSize+flagSize)) 
		cout<<"Receive Success!!!"<<endl;
	
	Mat temp_RGBmat(Size(640,480), CV_8UC3, RGBData);
	Mat temp_DEPTHmat(Size(640,480), CV_16UC1, DepthData);
	Mat temp_Transmat(Size(4,4),CV_32F,TransformationData);

	int temp_End_Flag=atoi ((const char *)End_Flag_Data);

	
	*RGBframe_=temp_RGBmat;
	*Depthframe_=temp_DEPTHmat;
	*Transformation_=temp_Transmat;
	*End_Flag=temp_End_Flag;

	return 1;
}	

	//
//wifi 发送数据函数
/*
void wifi_comu::send_data(char *data,unsigned int num)
{
	unsigned int data_num=num;
	char *wifi_data=data;

	if(sendto(pc_sock,wifi_data,data_num,0,(struct sockaddr*)&Remote_Addr,sizeof(Remote_Addr))==-1)
	printf("wifi_send_error\n");	
		
}
*/


//wifi发送新函数
/*
void wifi_comu::send_data_new(Mat frame)
{
	int imgSize = frame.total()*frame.elemSize();

	if((write(server_sock,frame.data, imgSize))==-1)
	{
		printf("wifi_send_error\n");
	}
	//cout<<"send finishied"<<endl;
	return;
		
}

//wifi server端接收新函数
cv::Mat  wifi_comu::receive_data_pc(Mat frame)
{
	const int imgSize = frame.total()*frame.elemSize();
	 uchar sockData[imgSize];

	int bytes=0;

	for (int i=0; i<imgSize; i+=bytes)
	{
		if((bytes=recv(client_sock,sockData+i, imgSize-i, 0 ))==-1)
		{
			cout<<"Fault"<<endl;
			close(client_sock);
			exit(-1);

		}
	}
	Mat temp_mat(Size(640,480), CV_8UC3, sockData);
	
	return temp_mat;
}
*/
//



}//namespace acrbslam

