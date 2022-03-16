#ifndef MAP_H
#define MAP_H
#include "vo_practice/common_include.h"
#include "vo_practice/mappoint.h"
#include "vo_practice/frame.h"
namespace vo_practice
{
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long,MapPoint::Ptr> map_points_;     //all landmarks
    unordered_map<unsigned long,Frame::Ptr> keyframes_;        //all key-frames

public:
    Map() { }

    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr map_point);
};
}
#endif