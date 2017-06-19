#include "acrbslam/common_include.h"
#include "acrbslam/data.h"

namespace acrbslam
{

cv::viz::Viz3d viz_initialize();		
void viz_update(viz::Viz3d vis, Data data);

}//namespace acrbslam
