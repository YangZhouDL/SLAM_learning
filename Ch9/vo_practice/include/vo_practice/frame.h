#ifndef FRAME_H
#define FRAME_H
#include "vo_practice/common_include.h"
#include "vo_practice/camera.h"
namespace vo_practice
{
class Frame
{
public:
    typedef shared_ptr<Frame> Ptr;
    unsigned long id_;
    double time_stamp_;
    SE3<double> T_c_w_;
    Camera::Ptr camera_;
    Mat color_,depth_;

public:     //data functions
    Frame();
    Frame(long id,double time_stamp=0.0,SE3<double> T_c_w=SE3<double>(),Camera::Ptr camera=nullptr,Mat color=Mat(),Mat depth=Mat());
    ~Frame();

    //factory function
    static Frame::Ptr createFrame();
    //find the depth in depth map
    double findDepth(const KeyPoint& kp);
    //Get camera center
    Vector3d getCamCenter() const;
    //check if a point is in this frame
    bool isInFrame(const Vector3d& pt_world);

};
}
#endif