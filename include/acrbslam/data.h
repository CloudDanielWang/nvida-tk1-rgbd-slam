#ifndef DATA_H
#define DATA_H

#include <acrbslam/common_include.h>
#include <acrbslam/frame.h>
#include "acrbslam/converter.h"

namespace acrbslam
{


class Data:public Converter
{
public:
    	typedef shared_ptr<Data> Ptr;		//模仿VO类进行指针的设定

	int frameID;

	uchar  End_Flag;	//结束标志位
	//int End_Flag;
	uchar TCPRGB[];
	uchar TCPDepth[];
	uchar TCPTransMatirx[];
	uchar TCPSendData[];


	int RGBImgSize ;
	int DepthImgSize ;
	int TransMatrixSize ;
	int EndFlagSize ;
	int TCPSendDataSize;

	Mat CameraImage;	
	Mat Depth;
	Mat T_c_w_mat;		

	SE3 T_c_w;
	Eigen::Isometry3d transfomation;
	Eigen::Matrix3d rotation_estimate;
	Eigen::Vector3d translation_estimate;

	float x,y,z;



   // typedef shared_ptr<Data> Ptr;


public:
	Data();
	~Data();

	void inputData(Frame::Ptr frame);		//将frame中的参数保存在data类中
	Data empty();

//protected:
	




};








}// namesapce acrbslam



#endif	//DATA_H