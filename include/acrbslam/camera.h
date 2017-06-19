
#ifndef CAMERA_H
#define CAMERA_H

#include "acrbslam/common_include.h"

namespace acrbslam
{

// Pinhole RGBD camera model
class Camera
{
public:                                                                     //此处为了方便将类的数据成员均定义为了公有
    typedef std::shared_ptr<Camera> Ptr;                //智能指针定义
    float   fx_, fy_, cx_, cy_, depth_scale_;                   //相机参数定义

    Camera();
    Camera ( float fx, float fy, float cx, float cy, float depth_scale=0 ) :
        fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depth_scale_ ( depth_scale )
    {}

    // coordinate transform: world, camera, pixel
    //世界坐标系，相机坐标系和像素坐标系的转换函数声明
    Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
    Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
    Vector2d camera2pixel( const Vector3d& p_c );
    Vector3d pixel2camera( const Vector2d& p_p, double depth=1 ); 
    Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 );
    Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );

};

}
#endif // CAMERA_H
