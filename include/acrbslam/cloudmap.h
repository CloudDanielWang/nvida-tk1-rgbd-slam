#ifndef CLOUDMAP_H
#define CLOUDMAP_H

#include "acrbslam/common_include.h"
#include "acrbslam/visual_odometry.h"
#include "acrbslam/frame.h"

     // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT; 
    typedef pcl::PointCloud<PointT> pointCloud;

    pointCloud::Ptr createPointCloud( acrbslam::Frame::Ptr frame, pointCloud::Ptr orginal_cloud );
    //pointcloud::Ptr joinPointCloud( pointcloud::Ptr original, acrbslam::FRAME& newFrame, Eigen::Isometry3d T ) ;



    
namespace acrbslam
{
	pointCloud::Ptr createPointCloud( acrbslam::Data data ,pointCloud::Ptr orginal_cloud);
}// namespace acrbslam




#endif