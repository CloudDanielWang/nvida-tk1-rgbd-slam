
#include "acrbslam/frame.h"

namespace acrbslam
{
Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
{

}

Frame::Frame ( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth), is_key_frame_(false)
{

}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;                                     //帧的索引数
    return Frame::Ptr( new Frame(factory_id++) );
}

double Frame::findDepth ( const cv::KeyPoint& kp )
{
    int x = cvRound(kp.pt.x);                       //像素所在列数
    int y = cvRound(kp.pt.y);                       //像素所在行数
    ushort d = depth_.ptr<ushort>(y)[x];
    if ( d!=0 )
    {
        return double(d)/camera_->depth_scale_;
    }
    else 
    {
        // check the nearby points      //寻找了紧邻特征点的周围四个像素点的深度
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/camera_->depth_scale_;
            }
        }
    }
    return -1.0;
}

void Frame::setPose ( const SE3& T_c_w )
{
    T_c_w_ = T_c_w;
}


Vector3d Frame::getCamCenter() const
{
    return T_c_w_.inverse().translation();                                                //T_c_w_.inverse()为T_w_c_
}

bool Frame::isInFrame ( const Vector3d& pt_world )
{
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    // cout<<"P_cam = "<<p_cam.transpose()<<endl;
    if ( p_cam(2,0)<0 ) return false;                                                       //相机坐标的z轴为深度值
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    return pixel(0,0)>0 && pixel(1,0)>0             //像素的u,v大于0
         && pixel(0,0)<color_.cols                        //像素的u小于图像的列数
         && pixel(1,0)<color_.rows;                     //像素的v小于图像的行数
}

}
