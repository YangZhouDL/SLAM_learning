#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

int main(int argc,char **argv)
{
    //读取图像
    Mat raw_image1=imread("../1.JPG");
    Mat raw_image2=imread("../2.JPG");
    
    Mat image1,image2;
    resize(raw_image1,image1,Size(raw_image1.cols/4,raw_image1.rows/4));
    resize(raw_image2,image2,Size(raw_image2.cols/4,raw_image2.rows/4));

    if(image1.data==nullptr||image2.data==nullptr)
    {
        cout<<"读取失败！"<<endl;
    }

    //初始化
    vector<KeyPoint> keypoints_1,keypoints_2;
    Mat descriptors_1,descriptors_2;
    Ptr<ORB> orb=ORB::create(1000,1.2f,8,31,0,2,ORB::HARRIS_SCORE,31,20);

    //第一步：检测Oriented Fast角点位置
    orb->detect(image1,keypoints_1);
    orb->detect(image2,keypoints_2);

    //第二步：根据角点位置计算BRIEF描述子
    orb->compute(image1,keypoints_1,descriptors_1);
    orb->compute(image2,keypoints_2,descriptors_2);

    Mat outimg1;
    drawKeypoints(image1,keypoints_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    imshow("【ORB特征点】",outimg1);

    //第三步：对两幅图像中BRIEF描述子进行匹配，使用Hamming距离
    vector<DMatch> matches;
    BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptors_1,descriptors_2,matches);

    //第四步：匹配点对筛选
    //找出所有匹配之间最小、最大距离，即最相似与最不相似的两组点间的距离
    double min_dst=10000,max_dst=0;
    for(int i=0;i<descriptors_1.rows;i++)
    {
        double dist=matches[i].distance;
        if(dist<min_dst)
        {
            min_dst=dist;
        }
        if(dist>max_dst)
        {
            max_dst=dist;
        }
    }
    cout<<"Max dist="<<max_dst<<endl;
    cout<<"Min dist="<<min_dst<<endl;

    //限制描述子之间的距离，防止误匹配
    //防止最小距离过小，以经验值为下限
    vector<DMatch> good_matches;
    for(int i=0;i<descriptors_1.rows;++i)
    {
        if(matches[i].distance<=max(2*min_dst,30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }

    //绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches(image1,keypoints_1,image2,keypoints_2,matches,img_match);
    drawMatches(image1,keypoints_1,image2,keypoints_2,good_matches,img_goodmatch);
    imshow("【所有匹配点对】",img_match);
    imshow("【优化后的匹配点对】",img_goodmatch);

    waitKey(0);

    return 0;
}