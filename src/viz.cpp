#include "acrbslam/viz.h"
#include "acrbslam/data.h"

namespace acrbslam
{

 viz::Viz3d viz_initialize()
{

        cv::viz::Viz3d vis ( "Visual Odometry" );
        cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
        cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
        cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
        vis.setViewerPose ( cam_pose );

        world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
        camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
        vis.showWidget ( "World", world_coor );
        vis.showWidget ( "Camera", camera_coor );

        return  vis;

}

void viz_update(viz::Viz3d vis, Data data)
{
        cv::Affine3d M;
        data.T_c_w=data.toSE3(data.T_c_w_mat);
        SE3 Twc = data.T_c_w.inverse();
        M=data.toAffine3d(Twc);
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false );
        return;
}



}//namespace acrbslam
