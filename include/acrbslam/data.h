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
    	//typedef shared_ptr<Data> Ptr;		//模仿VO类进行指针的设定

	int frameID;

	uchar  End_Flag;	//结束标志位

	int RGBImgSize ;
	int DepthImgSize ;
	int TransMatrixSize ;
	int EndFlagSize ;
	int TCPSendDataSize;

	Mat CameraImage;	
	Mat Depth;
	Mat T_c_w_mat;		

	SE3 T_c_w;
	Eigen::Isometry3d EigenTransfomation;
	Eigen::Matrix3d EigenRotationEstimate;
	Eigen::Vector3d EigenTranslationEstimate;

	float x,y,z;

public:
	Data();
	~Data();

	void inputData(Frame::Ptr frame);		//将frame中的参数保存在data类中
	void SE32Eigen();				//将SE3格式的矩阵转化为Eigen
	Data empty();

//protected:
	


};








}// namesapce acrbslam



#endif	//DATA_H