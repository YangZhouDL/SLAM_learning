// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "vo_practice/visual_odometry.h"

int main ( int argc, char** argv )
{
    vo_practice::Config::setParameterFile ("../config/default.yaml");
    vo_practice::VisualOdometry::Ptr vo ( new vo_practice::VisualOdometry );

    string dataset_dir = vo_practice::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
        {
            break;
        }
    }

    vo_practice::Camera::Ptr camera ( new vo_practice::Camera );
    camera->cx_=vo_practice::Config::get<float>("camera.cx");
    camera->cy_=vo_practice::Config::get<float>("camera.cy");
    camera->fx_=vo_practice::Config::get<float>("camera.fx");
    camera->fy_=vo_practice::Config::get<float>("camera.fy");
    camera->depth_scale_=vo_practice::Config::get<float>("camera.depth_scale");

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

    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    for ( int i=0; i<rgb_files.size(); i++ )
    {
        cout<<"****** loop "<<i<<" ******"<<endl;
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], -1 );
        if ( color.data==nullptr || depth.data==nullptr )
            break;
        vo_practice::Frame::Ptr pFrame = vo_practice::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];

        boost::timer timer;
        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == vo_practice::VisualOdometry::LOST )
            break;
        SE3d Twc = pFrame->T_c_w_.inverse();
        cout<<"Twc.rotationMatrix:"<<endl<<Twc.rotationMatrix()<<endl;
        cout<<"Twc.translation:"<<Twc.translation().transpose()<<endl<<endl;

        // show the map and the camera pose
        cv::Affine3d::Mat3 Aff_mat;
        for(int i=0;i<3;++i)
        {
            for(int j=0;j<3;++j)
            {
                Aff_mat(i,j)=Twc.rotationMatrix()(i,j);
            }
        }
        cv::Affine3d M (Aff_mat,
            cv::Affine3d::Vec3 (
                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            )
        );

        Mat img_show = color.clone();
        for ( auto& pt:vo->map_->map_points_ )
        {
            vo_practice::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }

        cv::imshow ( "image", img_show );
        cv::waitKey ( 1 );
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false );
        cout<<endl;
    }

    return 0;
}