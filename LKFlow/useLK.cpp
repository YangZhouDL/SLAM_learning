#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
using namespace std; 

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

int main(int argc, char** argv)
{
    string path_to_dataset = "/home/prejudice/learning_slam/src/Ch8/rgbd_dataset_freiburg1_desk";
    string associate_file = path_to_dataset + "/associate.txt";
    
    ifstream fin( associate_file );
    if ( !fin ) 
    {
        cerr<<"I cann't find associate.txt!"<<endl;
        return -1;
    }
    
    string rgb_file, depth_file, time_rgb, time_depth;
    list<cv::Point2f> keypoints;      // 因为要删除跟踪失败的点，使用list
    cv::Mat color, depth, last_color;
    
    for (int index=0; index<100; index++)
    {
        fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
        color = cv::imread( path_to_dataset+"/"+rgb_file );
        depth = cv::imread( path_to_dataset+"/"+depth_file, -1 );
        if(color.data==nullptr||depth.data==nullptr)
        {
            cout<<"图片打开失败！"<<endl;
        }
        if (index ==0)
        {
            // 对第一帧提取FAST特征点
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect(color, kps);
            for (auto kp:kps)
            {
                keypoints.push_back(kp.pt);
            }
            last_color = color;
            cout<<"第"<<index+1<<"张图像 ";
            cout<<"tracked keypoints: "<<keypoints.size()<<endl<<endl;
            continue;
        }
        if (color.data==nullptr || depth.data==nullptr)
        {
            continue;
        }
        // 对其他帧用LK跟踪特征点
        vector<cv::Point2f> next_keypoints; 
        vector<cv::Point2f> prev_keypoints;
        for (auto kp:keypoints)
        {
            prev_keypoints.push_back(kp);
        }            
        vector<unsigned char> status;
        vector<float> error; 
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        cv::calcOpticalFlowPyrLK(last_color, color, prev_keypoints, next_keypoints, status, error);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
        cout<<"LK Flow use time："<<time_used.count()<<" seconds."<<endl;
        // 把跟丢的点删掉
        int i=0; 
        for (auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
        {
            if (status[i] == 0)
            {
                iter = keypoints.erase(iter);   //返回下一个数据的位置
                continue;
            }
            *iter = next_keypoints[i];      //更新keypoints中的数据
            iter++;
        }
        cout<<"第"<<index+1<<"张图像 ";
        cout<<"tracked keypoints: "<<keypoints.size()<<endl<<endl;
        if (keypoints.size() == 0)
        {
            cout<<"all keypoints are lost."<<endl;
            break; 
        }
        // 画出 keypoints
        cv::Mat img_show = color.clone();   //用clone防止对原图像的误操作
        for (auto kp:keypoints)
        {
            cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
        }
        cv::imshow("【corners】", img_show);
        cv::waitKey(0);
        last_color = color;     //对last_color做更新
    }
    return 0;
}