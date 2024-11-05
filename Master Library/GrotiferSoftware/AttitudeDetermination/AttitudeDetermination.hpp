#pragma once
# include "Eigen/Dense"
# include "Eigen/Geometry"
using Eigen::Matrix3d;
using Eigen::Vector3d;
class ForwardSol {
    public:
        static Matrix3d MakeRotationMatrixRod(Vector3d k, double theta);
        static Matrix3d MakeRotationMatrix2(int r0, double ang0, int r1, double ang1, int r2, double ang2);
        static Matrix3d MakeRotationMatrix(double a, double b, double c);
        //Main forward solution function
        static void GetSensorAngles(Matrix3d rotMat, double *thxIncl, double *thzIncl, double *thySun, double *thzSun);

};

class BackwardSol {
    private:
        static Vector3d FindU_b(double thySun, double thzSun);
        static Vector3d FindV_b(double thxIncl, double thzIncl);
        static Matrix3d FindM_r();
        static Matrix3d FindM_b(Vector3d u_b, Vector3d v_b);

        //The function being minimized for Nelder Mead
        static double AngleError(double xq[3], double val[]);

    public:
        static void GetEulerFromRot(Matrix3d rotMat, double* thx, double* thy, double* thz);
        //Actual Solution Functions
        static Matrix3d AlgebraicSolutionMatrix( double thxIncl, double thzIncl, double thySun, double thzSun);
        static Matrix3d NelminSolutionMatrix  (double thxIncl, double thzIncl, double thySun,  double thzSun);
};


class AngularVel: public BackwardSol {
    public:
        //extract the rotation axis and the angle about that rotation axis from a rotation matrix
        static Vector3d RotAxisFromRotMat(Matrix3d rotMat);
        static double RotAngleAboutRotAxis(Matrix3d rotMat);

        //Compute the rotation matrix from one rotated frame to another rotated frame using the algebraic solution
        //The sensor angles 1 is the initial frame, the sensor angles 2 are the final frame

        static Matrix3d RotMatBodyToBody(Matrix3d RotMat1,Matrix3d RotMat2);
        //The timestep
        static double time_interval;

        //The initial rotation matrix that we have to multiply in for some reason
        static Matrix3d p_iniRotMat;

        static Matrix3d p_prevRotMat;
        static Matrix3d p_rotMat;

        static Vector3d GetAngularVelVec(Matrix3d RotMat1, Matrix3d RotMat2);

};

void nelmin ( double fn ( double x[], double val[] ), int n, double start[], double xmin[],
  double *ynewlo, double reqmin, double step[], int konvge, int kcount,
  int *icount, int *numres, int *ifault, double val[]);
void timestamp ( );
