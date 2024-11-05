
# include "AttitudeDetermination.hpp"

using Eigen::Matrix3d;
using Eigen::Vector3d;

using namespace std;

#define PI 3.14159265358979323846

Vector3d AngularVel::RotAxisFromRotMat(Matrix3d rotMat)
{
    Vector3d u;
    u << rotMat(2, 1) - rotMat(1, 2), rotMat(0,2) - rotMat(2, 0) , rotMat(1, 0) - rotMat(0, 1);
    u.normalize();
    return u;
}

double AngularVel::RotAngleAboutRotAxis(Matrix3d rotMat){

    //manually calculate traace as we had this issue in MATLAB

    double trace {rotMat.trace()};

    double rotAngle {acos((trace - 1) / 2)};
    return rotAngle; //radians
}

 Matrix3d AngularVel::RotMatBodyToBody(Matrix3d RotMat1, Matrix3d RotMat2)
{
    Matrix3d incRotMat;
    incRotMat = RotMat2 * RotMat1.transpose()  ;

    return incRotMat;
}

double AngularVel::time_interval = 1;

Vector3d AngularVel::GetAngularVelVec(Matrix3d RotMat1, Matrix3d RotMat2)
{
    Matrix3d rotMat {RotMatBodyToBody(RotMat1, RotMat2)};

    Vector3d rotAxis {RotAxisFromRotMat(rotMat)};
    double rotAngle {RotAngleAboutRotAxis(rotMat)};

    Vector3d angVel;
    angVel = rotAxis * rotAngle / time_interval;

    return angVel;

}
