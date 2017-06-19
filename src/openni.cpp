#include <acrbslam/openni.h>
namespace acrbslam
{


    COpenNI::~COpenNI() 
    {
        context.Release();//释放空间
    }
    
    bool COpenNI::Initial() 
    {
        //初始化
        status = context.Init();
        if(CheckError("Context initial failed!")) {
            return false;
        }
        context.SetGlobalMirror(true);//设置镜像
        //产生图片node
        status = image_generator.Create(context);
        if(CheckError("Create image generator  error!")) {
            return false;
        }
        //产生深度node
        status = depth_generator.Create(context);
        if(CheckError("Create depth generator  error!")) {
            return false;
        }
        //视角校正
        status = depth_generator.GetAlternativeViewPointCap().SetViewPoint(image_generator);
        if(CheckError("Can't set the alternative view point on depth generator")) {
            return false;
        }

        return true;

    }

    bool COpenNI::Start() {
        status = context.StartGeneratingAll();
        if(CheckError("Start generating error!")) {
            return false;
        }
        return true;
    }

    bool COpenNI::UpdateData() {
        status = context.WaitNoneUpdateAll();
        if(CheckError("Update date error!")) {
            return false;
        }
        //获取数据
        image_generator.GetMetaData(image_metadata);
        depth_generator.GetMetaData(depth_metadata);

        return true;
    }

    //该函数返回真代表出现了错误，返回假代表正确
    bool COpenNI::CheckError(const char* error) {
        if(status != XN_STATUS_OK ) {
            cerr << error << ": " << xnGetStatusString( status ) << endl;
            return true;
        }
        return false;
    }



}//namespace acrbslam


