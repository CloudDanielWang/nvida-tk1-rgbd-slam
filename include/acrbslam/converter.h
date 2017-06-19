#ifndef CONVERTER_H
#define CONVERTER_H

#include "acrbslam/common_include.h"

namespace acrbslam
{


class Converter
{
public:
	//Converter();
	//~Converter();

	void rgbmat2rgbchar(Converter converter, Mat rgbmat, char **red,char **green,char **blue);	//将RGB图像转化为便于WiFi传输的char*
	void rgbchar2rgbmat(Converter converter, Mat rgbmat, char red[],char green[],char blue[]);
		
	void mat2char(Converter converter, Mat mat_, char** char_);
	void char2mat(Converter converter, Mat mat_, char** char_);

	static Mat toCvMat(const SE3  &SE3);		
	static Mat toCvMat(const Eigen::Matrix4d &m);
	Eigen::Matrix4d toMatrix4d(const cv::Mat &cvMat4);
	SE3 toSE3(const cv::Mat &cvT);
	Affine3d toAffine3d(SE3 &Twc);

	void SplitRGBMat(Mat RGBMat, Mat *ImageBlueChannel, Mat *ImageGreenChannel, Mat *ImageRedChannel);	//将MAT融合成一个通道
	Mat MergeRGBMat( Mat ImageBlueChannel, Mat ImageGreenChannel, Mat ImageRedChannel);			//将三个通道融合

	//void se32char(Sophus::SE3 pose, char rotation_char[3][3], char translation_char[3]);
	void se32char(Sophus::SE3 pose, char **rotation_char, char **translation_char);


	char red_mat[307200];		//640*480
	char green_mat[307200];
	char blue_mat[307200];

	//unsigned short int depth_mat[307200];
	char  depth_mat[76800];	//320*240
};






}//namespace arbslam

#endif //CONVERTER_H