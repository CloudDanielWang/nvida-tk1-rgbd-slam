

#include "acrbslam/converter.h"


namespace acrbslam
{

//converter::converter();
//Converter::Converter()


void Converter::rgbmat2rgbchar(Converter converter, Mat rgbmat, char **red,char **green,char **blue)
{	

	int x,y,temp;
	for( y = 0; y < rgbmat.rows; y++ ) 
	{ 
		temp=y*rgbmat.cols;

		for( x = 0; x < rgbmat.cols; x++ ) 
		{
			converter.red_mat[temp+x]=rgbmat.at<Vec3b>(y,x)[0];
			converter.green_mat[temp+x]=rgbmat.at<Vec3b>(y,x)[1];
			converter.blue_mat[temp+x]=rgbmat.at<Vec3b>(y,x)[2];
		}
	}

	*red=converter.red_mat;
	*green=converter.green_mat;
	*blue=converter.blue_mat;
}

/*
void Converter::rgbmat2rgbchar( Mat rgbmat, Data data)
{	

	int x,y,temp;

	for( y = 0; y < rgbmat.rows; y++ ) 
	{ 
		temp=y*rgbmat.cols;

		for( x = 0; x < rgbmat.cols; x++ ) 
		{
			red_temp[temp+x]=rgbmat.at<Vec3b>(y,x)[0];
			green_temp[temp+x]=rgbmat.at<Vec3b>(y,x)[1];
			blue_temp[temp+x]=rgbmat.at<Vec3b>(y,x)[2];
		}
	}

	*red=red_temp;
	*green=green_temp;
	*blue=blue_temp;
}
*/

void Converter::rgbchar2rgbmat(Converter converter, Mat rgbmat, char red[],char green[],char blue[])
{	

	int x,y,temp;
	for( y = 0; y < rgbmat.rows; y++ ) 
	{ 
		temp=y*rgbmat.cols;

		for( x = 0; x < rgbmat.cols; x++ ) 
		{
			rgbmat.at<Vec3b>(y,x)[0]=red[temp+x];
			rgbmat.at<Vec3b>(y,x)[1]=green[temp+x];
			rgbmat.at<Vec3b>(y,x)[2]=blue[temp+x];
		}
	}

}



void Converter::mat2char(Converter converter, Mat mat_, char** char_)
{	
	int x,y,temp;
	for( y = 0; y < mat_.rows; y++ ) 
	{ 
		temp=y*mat_.cols;

		for( x = 0; x < mat_.cols; x++ ) 
		{
			converter.depth_mat[temp+x]=mat_.at<ushort>(y,x);
		}
	}

	*char_=(char *)converter.depth_mat;	
}


void Converter::char2mat(Converter converter, Mat mat_, char** char_)
{	
	int x,y,temp;
	for( y = 0; y < mat_.rows; y++ ) 
	{ 
		temp=y*mat_.cols;

		for( x = 0; x < mat_.cols; x++ ) 
		{
			mat_.at<ushort>(y,x)=converter.depth_mat[temp+x];
		}
	}

}

void Converter::se32char(Sophus::SE3 pose, char **rotation_char, char **translation_char)
{
	Eigen::Matrix4d transfomation = pose.matrix();
	char rotation_char_[9];
	char translation_char_[3];

	rotation_char_[0]=transfomation(0,0);
	rotation_char_[1]=transfomation(0,1);
	rotation_char_[2]=transfomation(0,2);
	rotation_char_[3]=transfomation(1,0);
	rotation_char_[4]=transfomation(1,1);
	rotation_char_[5]=transfomation(1,2);
	rotation_char_[6]=transfomation(2,0);
	rotation_char_[7]=transfomation(2,1);
	rotation_char_[8]=transfomation(2,2);

	translation_char_[0]=transfomation(0,3);
	translation_char_[1]=transfomation(1,3);
	translation_char_[2]=transfomation(2,3);

	*rotation_char=(char *)rotation_char_;
	*translation_char=(char *)translation_char_;

	return;

}


cv::Mat Converter::toCvMat(const SE3  &SE3)
{
    Eigen::Matrix4d  eigMat = SE3.matrix();
    return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix4d &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

Eigen::Matrix4d Converter::toMatrix4d(const cv::Mat &cvMat4)
{
    Eigen::Matrix4d M;

    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2),cvMat4.at<float>(0,3),
         cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2),cvMat4.at<float>(1,3),
         cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2),cvMat4.at<float>(2,3),
         cvMat4.at<float>(3,0), cvMat4.at<float>(3,1), cvMat4.at<float>(3,2),cvMat4.at<float>(3,3);

    return M;
}

SE3 Converter::toSE3(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return SE3(R,t);
}

Affine3d Converter::toAffine3d(SE3 &Twc)
{
	cv::Affine3d M
	(
            		Affine3d::Mat3 
            		(
                		Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
               		Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
                		Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
           		),
           		 Affine3d::Vec3 
           		(
                		Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            		)
        	);
	return M;
}

/*	未完成编写，仅供参考
void Converter::SE32eigen(const SE3  T_c_w)
{
	transfomation = T_c_w.matrix();
	rotation_estimate = transfomation.rotation();
	translation_estimate=transfomation.translation();

}
*/

void Converter::SplitRGBMat(Mat RGBMat, Mat *ImageBlueChannel, Mat *ImageGreenChannel, Mat *ImageRedChannel)
{
	vector<Mat> channels;
	split(RGBMat,channels);
	*ImageBlueChannel=channels.at(0);
	*ImageGreenChannel=channels.at(1);
	*ImageRedChannel=channels.at(2);
	return;
}

Mat Converter::MergeRGBMat(Mat ImageBlueChannel, Mat ImageGreenChannel, Mat ImageRedChannel)
{	
	Mat ImageRGB;
	vector<Mat> channels={ImageBlueChannel,ImageGreenChannel, ImageRedChannel};
	merge(channels,ImageRGB);
	return ImageRGB;
}


}//namespace acrbslam


