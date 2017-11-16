#include <acrbslam/data.h>

namespace acrbslam
{

Data::Data()
:frameID(0)
{
	Mat CameraImage=Mat::zeros(480,640,CV_8UC3);	
	Mat Depth=Mat::zeros(480,640,CV_16UC1);
	Mat T_c_w_mat=Mat::zeros(4,4,CV_32F);		

	uchar End_Flag='0';
	int Empty_Flag=0;
}

Data::~Data()
{

}


void Data::inputData(Frame::Ptr frame)
{

	CameraImage=frame->color_.clone();// 初始化时第一帧记为关键帧
	Depth=frame->depth_.clone();
	T_c_w=frame->T_c_w_; 
	T_c_w_mat= toCvMat(T_c_w);  		//将SE3转化为MAT输入

	return;

}	



Data Data::empty()
{	
	Data data;
	data.Empty_Flag=1;
	return data;
}






}//namespace acrbslam

