#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;
using namespace Sophus;

int main()
{
    Matrix3d R;
    R<<1.0,2.0,3.0,
        2.0,3.0,4.0,
        3.0,4.0,5.0;
    R=R*R.transpose();

    Vector3d t(1.0,2.0,3.0);

    SE3d SE(R,t);
    cout<<SE.matrix()<<endl;

    return 0;
}