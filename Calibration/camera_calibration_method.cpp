/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "camera_calibration.h"
#include "matrix_algo.h"


using namespace easy3d;



/**
 *
 * @param points_3d   An array of 3D points.
 * @param points_2d   An array of 2D points.
 * @return True on success, otherwise false. On success, the camera parameters are returned by
 *           - fx and fy: the focal length (in our slides, we use 'alpha' and 'beta'),
 *           - cx and cy: the principal point (in our slides, we use 'u0' and 'v0'),
 *           - skew:      the skew factor ('-alpha * cot_theta')
 *           - R:         the 3x3 rotation matrix encoding camera orientation.
 *           - t:         a 3D vector encoding camera location.
 */
bool CameraCalibration::calibration(
        const std::vector<vec3>& points_3d,
        const std::vector<vec2>& points_2d,
        float& fx, float& fy,
        float& cx, float& cy,
        float& skew,
        mat3& R,
        vec3& t)
{
    std::cout << std::endl;
    std::cout << "WE going to implement the calibration() function in the following file:" << std::endl
              << "\t" << __FILE__ <<"\n\n\n"<< std::endl;

    // check if input is valid (e.g., number of correspondences >= 6, sizes of 2D/3D points must match)
    if (points_3d.size() < 6 || points_2d.size() < 6) {std::cout << "Input file contains invalid number (<6) of 2D/3D points.\n"; return false;}
    for (int i = 0; i < points_3d.size(); i++){
        if (points_3d[i].size() !=3){
            std::cout << points_3d.size() << "Input file contains invalid size of 3D points.\n";
            return false;}
    }
    for (int i = 0; i < points_2d.size(); i++) {
        if (points_2d[i].size() != 2) {
            std::cout << "Input file contains invalid size of 2D points.\n";
            return false;
        }
    }
    if (points_3d.size() != points_2d.size()) {
        std::cout << "Input file must contain (pairs) the same number of points for 2D and 3D.\n";
        return false;
    }
    // check if input is valid (e.g., number of correspondences >= 6, sizes of 2D/3D points must match)

#if 1
    //construct the P matrix (so P * m = 0).
    std::vector<double> array = {};
            for (int i = 0; i < points_3d.size(); i++) {
                for (int j = 0; j < 2; j++) {
                    if (j == 0) {
                        array.push_back(points_3d[i].x);
                        array.push_back(points_3d[i].y);
                        array.push_back(points_3d[i].z);
                        array.push_back(1);
                        for (int k = 0; k < 4; k++) { array.push_back(0); }
                        array.push_back(-points_2d[i].x * (points_3d[i].x));
                        array.push_back(-points_2d[i].x * (points_3d[i].y));
                        array.push_back(-points_2d[i].x * (points_3d[i].z));
                        array.push_back(-points_2d[i].x);
                    } else if (j == 1) {
                        for (int k = 0; k < 4; k++) { array.push_back(0); }
                            array.push_back(points_3d[i].x);
                            array.push_back(points_3d[i].y);
                            array.push_back(points_3d[i].z);
                            array.push_back(1);
                            array.push_back(-points_2d[i].y * (points_3d[i].x));
                            array.push_back(-points_2d[i].y * (points_3d[i].y));
                            array.push_back(-points_2d[i].y * (points_3d[i].z));
                            array.push_back(-points_2d[i].y);
                    }
                }
            }

    int r = 18, c = 12;
    Matrix<double> P(r, c, array.data());
    //std::cout << "P: \n" << P << std::endl;

    // Compute the SVD decomposition of P
    Matrix<double> U(r, r, 0.0);   // initialized with 0s
    Matrix<double> S(r, c, 0.0);   // initialized with 0s
    Matrix<double> V(c, c, 0.0);   // initialized with 0s


    svd_decompose(P, U, S, V);
    // Compute the SVD decomposition of P


    //Intermediate validation checks~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //Check validity of V
    Matrix<double> I(V.cols(),V.rows(), 0.0);
    I.load_identity(1);
    auto VVt=V*transpose(V);
    for (int i = 0; i < VVt.cols(); i++)
        for (int j = 0; j < VVt.rows(); j++)
            if (VVt[i][j] - I[i][j] > 0.00000001) {
                std::cout << "Invalid decomposition result of V: V*VT must give Identity Matrix (camera_calibration_method.cpp-line127)" << std::endl;
                std::cout << VVt << I;
                return false;
            }


    //Check Validity of U
    I.resize(U.rows(),U.cols()); //adjust Matrix to U's size
            I.load_zero(); //reset Matrix
            I.load_identity(1); //Build Identity Matrix
    auto UUt=U*transpose(U);
    for (int i = 0; i < UUt.cols(); i++)
        for (int j = 0; j < UUt.rows(); j++)
            if (UUt[i][j] - I[i][j] > 0.00000001) {
                std::cout << "Invalid decomposition result of U: U*UT must give Identity Matrix (camera_calibration_method.cpp-line141)" << std::endl;
                std::cout << UUt << I;
                return false;
            }

    //Check Validity of S
    for (int i = 0; i < S.cols(); i++)
        for (int j = 0; j < S.rows(); j++)
            if ((i != j) && (S[i][j]-S[i][j] > 0.00000001)) {

                std::cout << "Invalid decomposition result of S: S must be Diagonal Matrix (camera_calibration_method.cpp-line151)" << std::endl;
                std::cout<<S;
                return false;
            }
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //get m matrix
    Matrix<double> m(12,1, 0.0);
    for (int i = 0; i <12; i++) {
        m[i][0] = V.get_column(11)[i];
    }
    //get m matrix

    //Check Validity of P and m with the equation P*m = 0
    auto Czero=P*m;
    for (int i =0; i < Czero.rows(); i++){
        for (int j = 0; j < Czero.cols(); j++){
            if (abs(Czero[i][j])>0.001){
                std::cout<<"Invalid P (or m) as elements of their product may be far further from zero: not satisfying P*m=0 (camera_calibration_method.cpp-line169)"<<std::endl;
                std::cout<<P*m;
                return false;
            }
        }
    }


    //get M matrix
    mat34 M(1.0f);
    M.set_row(0, vec4(m[0][0],m[0][1],m[0][2],m[0][3]));
    M.set_row(1, vec4(m[0][4],m[0][5],m[0][6],m[0][7]));
    M.set_row(2, vec4(m[0][8],m[0][9],m[0][10],m[0][11]));
    //std::cout << M <<std::endl;
    //get M matrix

    //set A matrix from M
    mat3 A;
    A.set_row(0,vec3(M.row(0)[0],M.row(0)[1],M.row(0)[2]));
    A.set_row(1,vec3(M.row(1)[0],M.row(1)[1],M.row(1)[2]));
    A.set_row(2,vec3(M.row(2)[0],M.row(2)[1],M.row(2)[2]));
    //set A matrix from M


    //set b matrix from M
    Mat<3,1,float> b;
    b[0] = M.col(3)[0];
    b[1] = M.col(3)[1];
    b[2] = M.col(3)[2];
    //set b matrix from M

    //extract intrinsic parameters from M =[A,b]
    double rho = 1/sqrt(A.row(2).x * A.row(2).x + A.row(2).y*A.row(2).y+A.row(2).z*A.row(2).z);
    cx= rho*rho*(dot(A.row(0),(A.row(2))));
    cy = rho*rho*(dot(A.row(1),(A.row(2))));
    vec3 a1a3 = cross(A.row(0),(A.row(2)));
    vec3 a2a3 = cross(A.row(1),(A.row(2)));

    double mag13 = sqrt(a1a3.x*a1a3.x + a1a3.y*a1a3.y + a1a3.z*a1a3.z);
    double mag23 = sqrt(a2a3.x*a2a3.x + a2a3.y*a2a3.y + a2a3.z*a2a3.z);
    double theta = acos(-dot(a1a3,a2a3) / mag13 * mag23) ;
    fx = rho*rho*mag13*sin(theta);
    fy = rho*rho*mag23*sin(theta);

    //extract intrinsic parameters from M =[A,b]


    //extract extrinsic parameters from M =[A,b]
    vec3 r1 = a2a3 / mag23;
    vec3 r3 = rho*A.row(2);
    vec3 r2 = cross(r3,r1);
    R.set_row(0, r1);
    R.set_row(1, r2);
    R.set_row(2, r3);

    mat3 K;
    skew = -fx*1/tan(theta);
    K.set_row (0, vec3(fx, skew, cx));
    K.set_row(1, vec3(0, fy/sin(theta),cy));
    K.set_row(2, vec3(0, 0, 1));

    mat3 InvK = inverse(K);
    auto T =  InvK*rho*b;
    t = T.col(0);

    std::cout<<rho<<std::endl;
    //extract extrinsic parameters from M =[A,b]
    std::cout << "The intrinsic parameters:\n" << "\ncx: "<< cx << "\ncy: "<<cy<< "\nfx: "<< fx << "\nfy: " << fy <<"\nskew: "<< skew << "\n"<<std::endl;
    std::cout << "The extrinsic parameters:\n" <<"\nRotation Matrix: \n"<< R << "\nTranslation Matrix: \n"<< t << std::endl;
    return true;
#endif
}

















