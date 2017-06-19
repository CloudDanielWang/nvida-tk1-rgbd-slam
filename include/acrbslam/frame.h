
#ifndef FRAME_H
#define FRAME_H

#include "acrbslam/common_include.h"
#include "acrbslam/camera.h"

namespace acrbslam 
{
    
// forward declare 
class MapPoint;               //为什么要提前声明？

class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long                  id_;         // id of this frame
    double                         time_stamp_; // when it is recorded
    SE3                            T_c_w_;      // transform from world to camera
    Camera::Ptr                    camera_;     // Pinhole RGBD Camera model 
    Mat                            color_, depth_; // color and depth image 
    // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
    // std::vector<MapPoint*>         map_points_; // associated map points
    bool                           is_key_frame_;  // whether a key-frame

    //

    //
    
public: // data members 
    Frame();                    //帧的初始化函数
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );       //帧的赋值函数
    ~Frame();                   //帧的释放
    
    static Frame::Ptr createFrame(); 
    
    // find the depth in depth map
    double findDepth( const cv::KeyPoint& kp );
    
    // Get Camera Center
    Vector3d getCamCenter() const;
    
    void setPose( const SE3& T_c_w );
    
    // check if a point is in this frame 
    bool isInFrame( const Vector3d& pt_world );
};

}

#endif // FRAME_H
