#include <fstream>
#include <boost/timer.hpp>

#include "acrbslam/config.h"
#include "acrbslam/visual_odometry.h"
#include "acrbslam/wifi.h"
#include "acrbslam/data.h"

namespace acrbslam
{
    acrbslam::Data data;    //数据存储类 

    void* vo_thread(void *arg);
    void* wifi_thread(void *arg);
}   //namespace acrbslam 
   
    pthread_mutex_t mutex_data; //互斥锁




int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }
    acrbslam::Config::setParameterFile ( argv[1] );     
    
    int res=pthread_mutex_init(&mutex_data, NULL);
    if (res!=0)
    {
        perror("Mutex initialization failed");
        exit(EXIT_FAILURE);
    }


    pthread_t   thread_wifi;        void *retval_wifi;
    pthread_t   thread_vo;          void *retval_vo;


    int ret_vo=pthread_create(&thread_vo,NULL,acrbslam::vo_thread,NULL);
    if(ret_vo !=0)
    {
        perror("VO Thread Create  Failed");
        exit(EXIT_FAILURE);
    }

    /*int ret_wifi=pthread_create(&thread_wifi,NULL, acrbslam::wifi_thread, NULL);
    if(ret_wifi !=0)
    {
        perror("WIFI Thread Create  Failed");
        exit(EXIT_FAILURE);
    }
	*/
    while(1);
    
    pthread_join(thread_vo,&retval_vo);
    pthread_join(thread_wifi,&retval_wifi);


      

    return 0;
}





namespace acrbslam
{

void* vo_thread(void *arg)
{
    acrbslam::VisualOdometry::Ptr vo ( new acrbslam::VisualOdometry );
    acrbslam::Camera::Ptr camera ( new acrbslam::Camera );


    //openni 输入
    VideoCapture capture(CV_CAP_OPENNI);    //设置视频的来源为OPENNI设备，即Kinect


       for ( int i=0; i<vo->scan_frame_num_ ; i++ )       //循环次数取决与参数文件中的数值
    {
               
        cout<<"****** loop "<<i<<" ******"<<endl;
        //需要切换为OPENNI设备时，将下面部分注释取消
        capture.grab();  
        Mat color;
        Mat depth ;
        capture.retrieve( color, CV_CAP_OPENNI_BGR_IMAGE );        //数据的读取可以做多线程加队列进行优化，加快运行速度（目前未优化）
        capture.retrieve( depth, CV_CAP_OPENNI_DEPTH_MAP ); 
        //OPENNI END


        acrbslam::Frame::Ptr pFrame = acrbslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        

        boost::timer timer;
        pthread_mutex_lock( &mutex_data );      //对data互斥锁住

        data=vo->addFrame(pFrame);
        
        //
        imshow("Camera Image", color);
        waitKey(1);
        //
        data.frameID=i;
        int data_empty_flag=data.CameraImage.empty();
        //data.End_Flag=0;
        //if(i==vo->scan_frame_num_-1 ) {data.End_Flag=1;}
        //cout<<"VO END flag"<<data.End_Flag<<endl;

        pthread_mutex_unlock( &mutex_data);     //对data解锁


        if(data_empty_flag==0)
        {
            cout<<"This Frame is Not The KeyFrame!!!"<<endl;
        }


        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == acrbslam::VisualOdometry::LOST )
            break;
    }
    //return;    //如何关闭线程？？


}


void* wifi_thread(void *arg)
{   
    wifi_comu wifi_comu_;
    wifi_comu_.wifi_init_uav();
    usleep(100);

    while(1)
    {    
        
        pthread_mutex_lock(&mutex_data);        //对data互斥线程锁，以避免在对data判断期间VO线程的干扰
        int flag=data.CameraImage.empty();
        if(flag==0)
         {  
        //Eigen::Matrix4d translation=data.toMatrix4d(data.T_c_w_mat);
        //cout<<"translation"<<translation<<endl;
       // cout<<"wifi send data begin"<<endl;
        wifi_comu_.send_data_client_writev(data.CameraImage, data.Depth, data.T_c_w_mat);
       // wifi_comu_.send_data_client_writev(data.CameraImage, data.Depth, data.T_c_w_mat, data.End_Flag);
        //cout<<"wifi send data finish"<<endl;
        cout<<"frameID:"<<data.frameID<<endl;
        //cout<<"data.End_Flag:"<<data.End_Flag<<endl;
        //if(data.End_Flag==1) break;

       // cv::imshow("wifi_send thread frame",data.CameraImage);
        //cv::waitKey(1);
       
         }//endif 
        pthread_mutex_unlock(&mutex_data);      //互斥锁解锁

        usleep(100);
        
    }
    //return;
        //pthread_mutex_unlock(&mutex_data);      //线程结束后，将互斥锁解锁，便于VO_thread end;
    exit(1);
}



}   //namespace acrbslam

