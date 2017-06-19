
#ifndef MAP_H
#define MAP_H

#include "acrbslam/common_include.h"
#include "acrbslam/frame.h"
#include "acrbslam/mappoint.h"

namespace acrbslam
{
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long, MapPoint::Ptr >  map_points_;        // all landmarks  存放所有的路标点
    unordered_map<unsigned long, Frame::Ptr >     keyframes_;         // all key-frames	存放所有的关键帧

    Map() {}
    
    void insertKeyFrame( Frame::Ptr frame );
    void insertMapPoint( MapPoint::Ptr map_point );
};
}

#endif // MAP_H
