
# include "AttitudeDetermination.hpp"

using Eigen::Matrix3d;
using Eigen::Vector3d;

using namespace std;

#define PI 3.14159265358979323846

Vector3d BackwardSol::FindU_b(double thySun, double thzSun)
{
    double ux_b;
    double uy_b;
    double uz_b;



    ux_b = 1 / (sqrt(1 + pow(tan(thySun), 2) + pow(tan(thzSun), 2)));
    uy_b = ux_b * tan(thySun);
    uz_b = ux_b * tan(-thzSun);

    Vector3d u_b;

    u_b << ux_b, uy_b, uz_b;

    return u_b;
}
Vector3d BackwardSol::FindV_b(double thxIncl, double thzIncl)
{
    double vx_b;
    double vy_b;
    double vz_b;

    vy_b = 1 / (sqrt(1 + pow(tan(thxIncl), 2) + pow(tan(thzIncl), 2)));
    vx_b = vy_b * tan(thxIncl);
    vz_b = vy_b * tan(thzIncl);

    Vector3d v_b;

    v_b << vx_b, vy_b, vz_b;
    return v_b;
}

Matrix3d findM_b(Vector3d u_b, Vector3d v_b)
{
    Matrix3d bodyAxes;


    //Get Matrix M_b
    Matrix3d M_b;
    Vector3d q_b;
    Vector3d r_b;
    Vector3d s_b;


    q_b = u_b;
    r_b = u_b.cross(v_b) / (u_b.cross(v_b).norm());
    s_b = q_b.cross(r_b);

    bodyAxes << q_b, r_b, s_b;

    return bodyAxes;
}

Matrix3d BackwardSol::FindM_b(Vector3d u_b, Vector3d v_b)
{
    Matrix3d bodyAxes;


    //Get Matrix M_b
    Matrix3d M_b;
    Vector3d q_b;
    Vector3d r_b;
    Vector3d s_b;


    q_b = u_b;
    r_b = u_b.cross(v_b) / (u_b.cross(v_b).norm());
    s_b = q_b.cross(r_b);

    bodyAxes << q_b, r_b, s_b;

    return bodyAxes;
}

Matrix3d BackwardSol::FindM_r()
{
    Matrix3d referenceAxes;
    //u_r is always the x axis (points toward sun)
    //v_r is always the opposite of gravity vector

    Vector3d u_r;
    u_r << 1, 0, 0;

    Vector3d v_r;
    v_r << 0, 1, 0;

    Vector3d q_r;
    Vector3d r_r;
    Vector3d s_r;

    q_r = u_r;
    r_r = u_r.cross(v_r) / (u_r.cross(v_r).norm());
    s_r = q_r.cross(r_r);

    referenceAxes << q_r, r_r, s_r;

    return referenceAxes;

}
Matrix3d BackwardSol::AlgebraicSolutionMatrix( double thxIncl, double thzIncl, double thySun, double thzSun)
{
    Vector3d u_b {FindU_b(thySun, thzSun)};
    Vector3d v_b {FindV_b(thxIncl, thzIncl)};

    Matrix3d M_b {FindM_b(u_b, v_b)};
    Matrix3d M_r {FindM_r()};

    Matrix3d attitudeMatrix;
    attitudeMatrix = M_b * M_r.transpose();

    //Not very important, but we should try to figure out why this is returning the transpose of the correct matrix

    return attitudeMatrix.transpose();
}


double BackwardSol::AngleError ( double xq[3], double val[] )

//****************************************************************************80
//
//
{
  double fx;

  // Compute the sensor angles for the given rotation matrix angles
  double thxIncl, thzIncl, thySun, thzSun;
  Matrix3d rotMat = ForwardSol::MakeRotationMatrix(xq[0], xq[1], xq[2]);
  ForwardSol::GetSensorAngles(rotMat, &thxIncl, &thzIncl, &thySun, &thzSun);
  double fInc = (thxIncl - val[0]) * (thxIncl - val[0]) + (thzIncl - val[1]) * (thzIncl - val[1]);
  double fSun = (thySun - val[2]) *  (thySun - val[2]) + (thzSun - val[3]) * (thzSun - val[3]);
  fx = fInc + fSun;

  return fx;
}
Matrix3d BackwardSol::NelminSolutionMatrix(double thxIncl, double thzIncl, double thySun,  double thzSun)
{
  int icount;
  int ifault;
  int kcount;
  int konvge;
  int n;
  int numres;
  double reqmin;
  double *start;
  double *step;
  double *xmin;
  double error;

  n = 3;  // Number of unknowns -- the 3 angles to create a rotation matrix

  start = new double[n];
  step = new double[n];
  xmin = new double[n];

  //double start[n];
  //double step[n];
  //double xmin[n];

  start[0] = 0.0;  // Starting "guess" values for Nelder-Mead
  start[1] = 0.0;
  start[2] = 0.0;

  reqmin = 1.0E-10;

  step[0] = 0.1;  // Initial step size for constructing the simplex
  step[1] = 0.1;
  step[2] = 0.1;

  konvge = 50;
  kcount = 500;


  // Axis locations
  Vector3d x, y, z;
  x << 1, 0, 0;
  y << 0, 1, 0;
  z << 0, 0, 1;

  double val[] = {thxIncl, thzIncl, thySun, thzSun};

  error = AngleError ( start, val );

  nelmin ( AngleError, n, start, xmin, &error, reqmin, step,
    konvge, kcount, &icount, &numres, &ifault, val );

  error = AngleError (xmin,val);

  // Compute estimate axis locations
  Matrix3d rotMat;

  rotMat = ForwardSol::MakeRotationMatrix(xmin[0], xmin[1], xmin[2]);
  Vector3d xe = rotMat * x;
  Vector3d ye = rotMat * y;
  Vector3d ze = rotMat * z;

  Matrix3d attitudeMatrix;

  attitudeMatrix << xe, ye, ze;

  return attitudeMatrix;
}

void BackwardSol::GetEulerFromRot(Matrix3d rotMat, double* thx, double* thy, double* thz)
{
    Matrix3d R = rotMat;

    *thy = -asin(R(2,0));
    *thx = atan2(R(2, 1) / cos(*thy), R(2, 2) / cos(*thy));
    *thz = atan2(R(1, 0) / cos(*thy), R(0, 0) / cos(*thy));

}
