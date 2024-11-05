# include "Eigen/Dense"
# include "Eigen/Geometry"
# include "AttitudeDetermination.hpp"

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector3d;

#define PI 3.14159265358979323846

Matrix3d ForwardSol::MakeRotationMatrixRod(Vector3d k, double theta)
{
    // from: https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    // Get rotation matrix to rotate a vector about axis k by angle theta (using Rodrigues' formula)
    Matrix3d unit{{1.0, 0.0, 0.0},{0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};  // Unit matrix
    Matrix3d KK, rotmat;

    k.normalize();  // Make sure it is a unit vector
    KK << 0.0, -k(2), k(1),   k(2), 0.0, -k(0),   -k(1), k(0), 0.0;
    rotmat = unit + sin(theta) * KK + (1.0 - cos(theta)) * KK * KK;
    return rotmat;
}

Matrix3d ForwardSol::MakeRotationMatrix(double a, double b, double c)
{
    Matrix3d rotX, rotY, rotZ;
    rotX << 1.0, 0, 0,   0, cos(a), -sin(a),   0, sin(a), cos(a);
    rotY << cos(b), 0, sin(b),   0, 1, 0,    -sin(b), 0, cos(b);
    rotZ << cos(c), -sin(c), 0,   sin(c), cos(c), 0,   0, 0, 1;

    //Assuming order x - y - z (flipped due to nature of linear transformations)
    Matrix3d rotmat = rotZ * rotY * rotX;

    return rotmat;
}

Matrix3d ForwardSol::MakeRotationMatrix2(int r0, double ang0, int r1, double ang1, int r2, double ang2)
{
    int rt [] = {r0, r1, r2};
    double ang [] = {ang0, ang1, ang2};

    // Rotations about body fixed axes
    Vector3d xp, yp, zp;
    xp << 1, 0, 0;
    yp << 0, 1, 0;
    zp << 0, 0, 1;

    Matrix3d rotf{{1.0, 0.0, 0.0},{0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    Matrix3d rotm;

    for(int i = 0; i < 3; i++)
    {
        switch(rt[i])
        {
            case 0:
                rotm = MakeRotationMatrixRod(xp, ang[0]);
                break;
            case 1:
                rotm = MakeRotationMatrixRod(yp, ang[1]);
                break;
            case 2:
                rotm = MakeRotationMatrixRod(zp, ang[2]);

        }
        rotf = rotm * rotf;
        xp = rotm * xp;
        yp = rotm * yp;
        zp = rotm * zp;
    }
    /*
    printf("1\n%8.5g %8.5g %8.5g  X\n", xp(0), xp(1), xp(2));
    printf("%8.5g %8.5g %8.5g  Y\n", yp(0), yp(1), yp(2));
    printf("%8.5g %8.5g %8.5g  Z\n", zp(0), zp(1), zp(2));
    */

    Matrix3d rotmat = rotf;

    // Check result
    Vector3d x, y, z;
    x << 1, 0, 0;
    y << 0, 1, 0;
    z << 0, 0, 1;
    xp = rotmat * x;
    yp = rotmat * y;
    zp = rotmat * z;
    /*
    printf("2\n%8.5g %8.5g %8.5g  X\n", xp(0), xp(1), xp(2));
    printf("%8.5g %8.5g %8.5g  Y\n", yp(0), yp(1), yp(2));
    printf("%8.5g %8.5g %8.5g  Z\n", zp(0), zp(1), zp(2));
    */
    return rotmat;
}

void ForwardSol::GetSensorAngles(Matrix3d rotMat, double *thxIncl, double *thzIncl, double *thySun, double *thzSun)
{
    Vector3d x, y, z;
    x << 1, 0, 0;
    y << 0, 1, 0;
    z << 0, 0, 1;

    Vector3d xp = rotMat * x;
    Vector3d yp = rotMat * y;
    Vector3d zp = rotMat * z;

    int inclModel = 2; // Model #1 - direct angle from vertical,
                        //model #2 - model using projection (arctan)
    Vector3d xpProjYZ, zpProjXY, ypProjXZ;

    if(inclModel == 1)
    {
        // Direct angle from vertical
        *thxIncl = (acos(xp.dot(y)) - PI / 2.0);
        *thzIncl = (acos(zp.dot(y)) - PI / 2.0);
    }
    else
    {
        // Model 2, using projections, arctan model
        *thxIncl = atan(y.dot(xp) / y.dot(yp));
        *thzIncl = atan(y.dot(zp) / y.dot(yp));
    }

    // Sun sensor
    *thySun = atan(x.dot(yp) / x.dot(xp));
    *thzSun = -atan(x.dot(zp) / x.dot(xp));

    return;
}
