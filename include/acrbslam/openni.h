#include <acrbslam/common_include.h>

namespace acrbslam
{

class COpenNI
{
public:
    ~COpenNI() ;
    bool Initial() ;
    bool Start() ;
    bool UpdateData();

public:
    DepthMetaData depth_metadata;
    ImageMetaData image_metadata;

private:
    //该函数返回真代表出现了错误，返回假代表正确
    bool CheckError(const char* error);

private:
    XnStatus    status;
    Context     context;
    DepthGenerator  depth_generator;
    ImageGenerator  image_generator;
};




}//namespace acrbslam
