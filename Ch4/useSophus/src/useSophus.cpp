#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;
using namespace Sophus;

int main(int argc,char **argv)
{
    AngleAxisd a(M_PI/2,Vector3d(0,0,1));
    Matrix3d R=a.toRotationMatrix();
    Quaterniond q(a);
    Vector3d vec=a.axis().cast<double>()*a.angle();
    SO3d SO3_R(R);
    SO3d SO3_Q(q);
    cout<<"SO(3) from matrix:\n"<<SO3_R.matrix()<<endl;
    cout<<"SO(3) from quaternion:\n"<<SO3_Q.matrix()<<endl;
    // cout<<"SO(3) from angle_axis:\n"<<SO3_A.matrix()<<endl;
    cout<<"They are equal"<<endl;

    Vector3d so3=SO3_R.log();   //对数映射
    cout<<"so3 = "<<so3.transpose()<<endl;
    Matrix3d skew=SO3d::hat(so3);
    cout<<"so3 hat=\n"<<skew<<endl;   //hat为向量到反对成矩阵
    //vee为反对成矩阵到向量
    Vector3d skew_v=SO3d::vee(skew);
    cout<<"so3 hat vee="<<skew_v.transpose()<<endl;

    Vector3d update_so3(1e-4,0,0);
    SO3d SO3_updated=SO3d::exp(update_so3)*SO3_R;   //指数映射
    cout<<"SO3 updated = \n"<<SO3_updated.matrix()<<endl;

    cout<<"*****************************************************"<<endl;
    Vector3d t(1,0,0);
    SE3d SE3_Rt(R,t);
    SE3d SE3_qt(q,t);
    cout<<"SE3 from R,t = \n"<<SE3_Rt.matrix()<<endl;
    cout<<"SE3 from Q,t = \n"<<SE3_qt.matrix()<<endl;
    typedef Eigen::Matrix<double,6,1> Vector6d;    //se是六维向量，而Eigen无六维向量
    Vector6d se3=SE3_Rt.log();  //对数映射
    cout<<"se3 = "<<se3.transpose()<<endl;  
    cout<<"se3 hat = \n"<<SE3d::hat(se3)<<endl; //并不是真正的反对称矩阵
    cout<<"se3 hat vee = "<<SE3d::vee(SE3d::hat(se3)).transpose()<<endl;

    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0,0)=1e-4;
    cout<<"update_se3 = "<<update_se3.transpose()<<endl;
    SE3d SE3_updated=SE3d::exp(update_se3)*SE3_Rt;
    cout<<"SE3 updated = \n"<<SE3_updated.matrix()<<endl;

    return 0;
}