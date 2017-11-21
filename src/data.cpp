#include <acrbslam/data.h>

namespace acrbslam
{

Data::Data()
:frameID(0),RGBImgSize(921600),DepthImgSize(614400),TransMatrixSize(64),EndFlagSize(1),TCPSendDataSize(1536065)
{
	Mat CameraImage=Mat::zeros(480,640,CV_8UC3);	
	Mat Depth=Mat::zeros(480,640,CV_16UC1);
	Mat T_c_w_mat=Mat::zeros(4,4,CV_32F);		

	uchar End_Flag='0';
	//int End_Flag=0;
	//int RGBImgSize = CameraImage.total()*CameraImage.elemSize();	cout<<"RGBImgSize\t"<<RGBImgSize<<endl;
	//int DepthImgSize =Depth.total()*Depth.elemSize();			cout<<"DepthImgSize\t"<<DepthImgSize<<endl;
	//int TransMatrixSize =T_c_w_mat.total()*T_c_w_mat.elemSize();		cout<<"TransMatrixSize\t"<<TransMatrixSize<<endl;
	//int EndFlagSize = sizeof(End_Flag);					cout<<"EndFlagSize\t"<<EndFlagSize<<endl;
	
	//uchar TCPRGB[ RGBImgSize ];
	//uchar TCPDepth[DepthImgSize];
	//uchar TCPTransMatirx[TransMatrixSize];
	
	//int TCPSendDataSize=RGBImgSize+DepthImgSize+TransMatrixSize+EndFlagSize;	//cout<<"TCPSendDataSize\t"<<TCPSendDataSize<<endl;
	//uchar TCPSendData[TCPSendDataSize];					//cout<<"unchar TCPSendDataSize\t"<<sizeof(TCPSendData)<<endl;
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

	//将TCW提取出xyz
	x=T_c_w.translation()(0,0);
	y=T_c_w.translation()(1,0);
	z=T_c_w.translation()(2,0);

	return;

}	

void Data::SE32Eigen()
{
	//EigenTransfomation = T_c_w.matrix(); 
	EigenRotationEstimate = T_c_w.rotation_matrix();
	EigenTranslationEstimate=T_c_w.translation();

}


Data Data::empty()
{	
	Data data;
	return data;
}






}//namespace acrbslam

