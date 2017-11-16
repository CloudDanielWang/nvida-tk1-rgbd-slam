#include <fstream>
#include <boost/timer.hpp>

#include "acrbslam/config.h"
#include "acrbslam/visual_odometry.h"
#include "acrbslam/wifi.h"
#include "acrbslam/data.h"
#include "acrbslam/openni.h"

namespace acrbslam
{
    Data data;    //数据存储类 

    void* vo_thread(void *arg);
    void* wifi_thread(void *arg);
}   //namespace acrbslam 
   
    pthread_mutex_t mutex_data; //互斥锁
    sem_t sem_TCP_send;



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
/*
    if ((sem_init(&sem_TCP_send,0,0))==-1)
    {
        perror("semaphore TCP SEND initialization failed");
        exit(EXIT_FAILURE);
    }
*/
    pthread_t   thread_wifi;        void *retval_wifi;
    pthread_t   thread_vo;          void *retval_vo;


    int ret_vo=pthread_create(&thread_vo,NULL,acrbslam::vo_thread,NULL);
    if(ret_vo !=0)
    {
        perror("VO Thread Create Failed");
        exit(EXIT_FAILURE);
    }

    int ret_wifi=pthread_create(&thread_wifi,NULL, acrbslam::wifi_thread, NULL);
    if(ret_wifi !=0)
    {
        perror("WIFI Thread Create  Failed");
        exit(EXIT_FAILURE);
    }

    while(1);
    
    pthread_join(thread_vo,&retval_vo);
    pthread_join(thread_wifi,&retval_wifi);


    return 0;
}





namespace acrbslam
{

void* vo_thread(void *arg)
{
    VisualOdometry::Ptr vo ( new acrbslam::VisualOdometry );
    Camera::Ptr camera ( new acrbslam::Camera );


    //openni 输入
    COpenNI openni;                                                     //设置视频的来源为OPENNI设备，即Kinect
    //cout<<"Begin to get data From OpenNI"<<endl;
    if(!openni.Initial())
	{	
		cout<<"openni Initial faild"<<endl;		
		exit(1) ; 
	}
            
	
    if(!openni.Start())
        {	
		cout<<"openni start faild"<<endl;		
		exit(1) ; 
	}

       for ( int i=0; i<vo->scan_frame_num_ ; i++ )       //循环次数取决与参数文件中的数值
    {
               
        cout<<"****** loop "<<i<<" ******"<<endl;
               
        if(!openni.UpdateData()) {
             exit(1) ;
        }
        /*获取并显示色彩图像*/


        //Method 1 segment fault
        /*
        Mat color_image_src(openni.image_metadata.YRes(), openni.image_metadata.XRes(),
                                        CV_8UC3, (char *)openni.image_metadata.Data());
        Mat color;
        cvtColor(color_image_src, color, CV_RGB2BGR,3);


        Mat depth_image_src(openni.depth_metadata.YRes(), openni.depth_metadata.XRes(),
                            CV_16UC1, (char *)openni.depth_metadata.Data());//因为kinect获取到的深度图像实际上是无符号的16位数据
        Mat depth;
        depth_image_src.convertTo(depth, CV_8U, 255.0/8000,0);
        */
        

/*       //Method 2 
        Mat color,depth;
        Mat temp_color, temp_depth;

        memcpy(temp_color.data, openni.image_metadata.Data(), 640*480*3);	cout<<"memcpy rgb"<<endl;//there leads faild
        cvtColor(temp_color, color, CV_RGB2BGR);
        
        memcpy(depth.data, openni.depth_metadata.Data(), 640*480*2);
        //cvConvertScale(depth,depth,255/4096.0);
        //temp_depth.convertTo(depth, CV_8U, 255.0/4096,0);
*/


	//Method 3
	Mat cColorImg( openni.image_metadata.FullYRes(),openni.image_metadata.FullXRes(),CV_8UC3, (void*)openni.image_metadata.Data());
	//convert from RGB to BGR
	cv::Mat color;
	cvtColor( cColorImg, color, CV_RGB2BGR );
	//imshow("Camera Image", color);
	//waitKey(0);

	//convert to OpenCV form
	cv::Mat depth( openni.depth_metadata.FullYRes(),openni.depth_metadata.FullXRes(),
                         CV_16UC1, (void*)openni.depth_metadata.Data());
	//imshow("Depth Image", depth);
	//waitKey(0);

	
	//cout<<"OPENNI END"<<endl;
        //OPENNI END	




/*******************************************************************************/
	
        Frame::Ptr pFrame = Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        
	ORB orb_;
		Mat a,b;
	vector<cv::KeyPoint> key;
	orb_(a,Mat(),key,b); 


        boost::timer timer;
        //pthread_mutex_lock( &mutex_data );      //对data互斥锁住
	//cout<<"before addframe"<<endl;
        data=vo->addFrame(pFrame,data);
      
        //
        //imshow("Camera Image", color);
        //waitKey(1);
        //
        data.frameID=i;
        int data_empty_flag=data.CameraImage.empty();
        data.End_Flag=0;
        if(i==vo->scan_frame_num_-1 ) {data.End_Flag=1;}
        //cout<<"VO END flag"<<data.End_Flag<<endl;

        //pthread_mutex_unlock( &mutex_data);     //对data解锁


        if(data_empty_flag==0)
        {
            cout<<"This Frame is Not The KeyFrame!!!"<<endl;
        }

	//sem_post(&sem_TCP_send);
        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == VisualOdometry::LOST )
            break;
    }
    //return;    //如何关闭线程？？
 	exit(1);

}


void* wifi_thread(void *arg)
{ 
    wifi_comu wifi_comu_;
    wifi_comu_.wifi_init_uav();
    usleep(100);

    while(1)
    {    
        
        //pthread_mutex_lock(&mutex_data);        //对data互斥线程锁，以避免在对data判断期间VO线程的干扰
	//sem_wait(&sem_TCP_send);
        int flag=data.CameraImage.empty();
        if(flag==0)
         {  
        //Eigen::Matrix4d translation=data.toMatrix4d(data.T_c_w_mat);
        //cout<<"translation"<<translation<<endl;
       // cout<<"wifi send data begin"<<endl;
        //wifi_comu_.send_data_client_writev(data.CameraImage, data.Depth, data.T_c_w_mat);
        wifi_comu_.send_data_client_writev(data.CameraImage, data.Depth, data.T_c_w_mat, data.End_Flag);
	//wifi_comu_.send_data_new(data.CameraImage);
        //cout<<"wifi send data finish"<<endl;
        cout<<"frameID:"<<data.frameID<<endl;
        cout<<"data.End_Flag:"<<data.End_Flag<<endl;
        if(data.End_Flag==1) break;

        //cv::imshow("wifi_send thread frame",data.CameraImage);
        //cv::waitKey(1);
       
         }//endif 
        //pthread_mutex_unlock(&mutex_data);      //互斥锁解锁

        usleep(100);
        
    }
    //return;
        //pthread_mutex_unlock(&mutex_data);      //线程结束后，将互斥锁解锁，便于VO_thread end;
	cout<<"wifi send thread end"<<endl;
    exit(1);
}



}   //namespace acrbslam

