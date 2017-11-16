

#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// define the commonly included file to avoid a long include list
// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SO3;
using Sophus::SE3;

// for cv
#include <opencv2/opencv.hpp>
using cv::Mat;
using namespace cv;

//for OpenNi

#include <XnCppWrapper.h>
using namespace xn;



// std 
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>


///
#include <stdio.h>      /*标准输入输出定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h> 
#include <sys/stat.h>  
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*POSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <pthread.h>
#include <string.h> //和strings.h有什么区别
#include <math.h>
#include <linux/i2c-dev.h>  
#include <sys/ioctl.h> 
#include <signal.h>
#include <sys/time.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stddef.h>  
#include <arpa/inet.h>
#include <sys/socket.h>
#include <stdint.h>
#include <getopt.h>
#include <fcntl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "stddef.h"
#include <malloc.h>
#include <semaphore.h> 
#include <algorithm>
///



using namespace std; 

#endif
