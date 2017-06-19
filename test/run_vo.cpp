// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "acrbslam/config.h"
#include "acrbslam/visual_odometry.h"
#include "acrbslam/cloudmap.h"



int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    acrbslam::Config::setParameterFile ( argv[1] );
    acrbslam::VisualOdometry::Ptr vo ( new acrbslam::VisualOdometry );

    acrbslam::Camera::Ptr camera ( new acrbslam::Camera );

    VideoCapture capture(CV_CAP_OPENNI);    //设置视频的来源为OPENNI设备，即Kinect
    
    // visualization
    cv::viz::Viz3d vis ( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );

     pointCloud::Ptr pointCloud_all( new pointCloud ); //存放所有点云
     //pcl::visualization::CloudViewer viewer("cloudmap viewer");

    for ( int i=0; i<200; i++ )
    {
        capture.grab();                 
  
        cout<<"****** loop "<<i<<" ******"<<endl;
        Mat color;
        Mat depth ;
        capture.retrieve( color, CV_CAP_OPENNI_BGR_IMAGE );     
        capture.retrieve( depth, CV_CAP_OPENNI_DEPTH_MAP ); 

        acrbslam::Frame::Ptr pFrame = acrbslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;

        //test
        //    cout<<"imgsize:"<<sizeof(color)<<endl;           //96
        //     cout<<"depthsize:"<<sizeof(depth)<<endl;      //96
        //

        pFrame->depth_ = depth;


        boost::timer timer;
        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == acrbslam::VisualOdometry::LOST )
            break;

        //draw cloudmap
        pointCloud_all=createPointCloud(pFrame, pointCloud_all);
       

        ////////


        SE3 Twc = pFrame->T_c_w_.inverse();

        // show the map and the camera pose
        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
                Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
                Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
            ),
            cv::Affine3d::Vec3 (
                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            )
        );

        Mat img_show = color.clone();
        for ( auto& pt:vo->map_->map_points_ )
        {
            acrbslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }

        cv::imshow ( "image", img_show );
        cv::waitKey ( 0 );
        // viewer.showCloud( pointCloud_all );
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false );
        cout<<endl;
    }   
    
    cout<<"滤波之后，点云共有"<<pointCloud_all->size()<<"个点."<<endl;
         pcl::io::savePCDFileBinary( "data/result.pcd", *pointCloud_all );
         // while( !viewer.wasStopped() )
         //{}

    return 0;
}
