#ifndef DATA_H
#define DATA_H

#include <acrbslam/common_include.h>
#include <acrbslam/frame.h>
#include <acrbslam/converter.h>

namespace acrbslam
{


class Data:public Converter
{
public:

	int frameID;

	uchar End_Flag;//结束标志位
	int Empty_Flag;//data Empty_Flag

	Mat CameraImage;	
	Mat Depth;
	Mat T_c_w_mat;		

	SE3 T_c_w;
	Eigen::Isometry3d transfomation;
	Eigen::Matrix3d rotation_estimate;
	Eigen::Vector3d translation_estimate;






public:
	Data();
	~Data();

	void inputData(Frame::Ptr frame);		//将frame中的参数保存在data类中
	Data empty();

//protected:
	




};








}// namesapce acrbslam



#endif	//DATA_H
