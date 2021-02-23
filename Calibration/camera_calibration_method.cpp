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
 * TODO: Finish this function for calibrating a camera from the corresponding 3D-2D point pairs.
 *       You may define a few functions for some sub-tasks.
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
    std::cout << "TODO: I am going to implement the calibration() function in the following file:" << std::endl
              << "\t" << __FILE__ << std::endl;
    std::cout << "TODO: After implementing the calibration() function, I will disable all unrelated output ...\n\n";

    // TODO: check if input is valid (e.g., number of correspondences >= 6, sizes of 2D/3D points must match)
    //if (points_3d.size() < 6 || points_2d.size() < 6) std::cout << "Input file contains invalid number of 2D/3D points.\n"; return false;
    //for (int i = 0; i < points_3d.size(); i++) if (points_3d[i].length()!=3) std::cout << "Input file contains invalid size of 3D points.\n"; return false;
    //for (int i = 0; i < points_2d.size(); i++) if (points_3d[i].length()!=2) std::cout << "Input file contains invalid size of 3D points.\n"; return false;




    // TODO: solve for M (the whole projection matrix, i.e., M = K * [R, t]) using SVD decomposition.
    //   Optional: you can check if your M is correct by applying M on the 3D points. If correct, the projected point
    //             should be very close to your input images points.

    // TODO: extract intrinsic parameters from M.

    // TODO: extract extrinsic parameters from M.

    // TODO: uncomment the line below to return true when testing your algorithm and in you final submission.
    //return false;



    // TODO: The following code is just an example showing you SVD decomposition, matrix inversion, and some related.
    // TODO: Delete the code below (or change "#if 1" in the first line to "#if 0") in you final submission.
#if 1
    std::cout << "[Liangliang:] Camera calibration requires computing the SVD and inverse of matrices.\n"
                 "\tIn this assignment, I provide you with a Matrix data structure for storing matrices of arbitrary\n"
                 "\tsizes (see matrix.h). I also wrote the example code to show you how to:\n"
                 "\t\t- use the dynamic 1D array data structure 'std::vector' from the standard C++ library;\n"
                 "\t\t  The points (both 3D and 2D) are stored in such arrays;\n"
                 "\t\t- use the template matrix class (which can have an arbitrary size);\n"
                 "\t\t- compute the SVD of a matrix;\n"
                 "\t\t- compute the inverse of a matrix;\n"
                 "\t\t- compute the transpose of a matrix.\n"
                 "\tThe following are just the output of these examples. You should delete ALL unrelated code and\n"
                 "\tavoid unnecessary output in you final submission.\n\n";


    // TODO: construct the P matrix (so P * m = 0).

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

    const int r = 18, c = 12;
    Matrix<double> P(r, c, array.data());    // 'array.data()' returns a pointer to the array.
    std::cout << "P: \n" << P << std::endl;






    Matrix<double> U(r, r, 0.0);   // initialized with 0s
    Matrix<double> S(r, c, 0.0);   // initialized with 0s
    Matrix<double> V(c, c, 0.0);   // initialized with 0s


    // Compute the SVD decomposition of A
    svd_decompose(P, U, S, V);

    std::cout << "V: \n" << V << std::endl;

    //std::vector<double> m;
    Matrix<double> m(12,1, 0.0);
    for (int i = 0; i <12; i++) {
        m[i][0] = V.get_column(11)[i];
    }
    std::cout << m << std::endl;
    mat34 M(1.0f);
    M.set_row(0, vec4(m[0][0],m[0][1],m[0][2],m[0][3]));
    M.set_row(1, vec4(m[0][4],m[0][5],m[0][6],m[0][7]));
    M.set_row(2, vec4(m[0][8],m[0][9],m[0][10],m[0][11]));
    std::cout << M <<std::endl;

    mat3 A;
    A.set_row(0,vec3(M.row(0)[0],M.row(0)[1],M.row(0)[2]));
    A.set_row(1,vec3(M.row(1)[0],M.row(1)[1],M.row(1)[2]));
    A.set_row(2,vec3(M.row(2)[0],M.row(2)[1],M.row(2)[2]));


    Mat<3,1,float> b;
    b[0] = M.col(3)[0];
    b[1] = M.col(3)[1];
    b[2] = M.col(3)[2];

    std::cout << A <<std::endl << b << std::endl;

    double rho = 1/sqrt(A.row(2).x * A.row(2).x + A.row(2).y*A.row(2).y+A.row(2).z*A.row(2).z);
    double c_x = rho*rho*(dot(A.row(0),(A.row(2))));
    double c_y = rho*rho*(dot(A.row(1),(A.row(2))));
    vec3 a1a3 = cross(A.row(0),(A.row(2)));
    vec3 a2a3 = cross(A.row(1),(A.row(2)));
    double mag13 = sqrt(a1a3.x*a1a3.x + a1a3.y*a1a3.y + a1a3.z*a1a3.z);
    double mag23 = sqrt(a2a3.x*a2a3.x + a2a3.y*a2a3.y + a2a3.z*a2a3.z);
    double theta = 1/cos(-dot(a1a3,a2a3) / mag13 * mag23) ;
    double f_x = rho*rho*mag13*sin(theta);
    double f_y = rho*rho*mag23*sin(theta);

    std::cout << "The intrinsics:\nρ: "<< rho << "\nc_x: "<< c_x << "\nc_y: "<<c_y<<"\nθ: " << theta << "\nf_x: "<< f_x << "\nf_y: " << f_y << std::endl;

    vec3 r1 = a2a3 / mag23;
    vec3 r3 = rho*A.row(2);
    vec3 r2 = cross(r3,r1);
    R.set_row(0, r1);
    R.set_row(1, r2);
    R.set_row(2, r3);


    mat3 K;
    K.set_row (0, vec3(f_x,-fx*1/tan(theta), c_x));
    K.set_row(1, vec3(0, f_y/sin(theta),c_y));
    K.set_row(2, vec3(0, 0, 1));


    mat3 InvK = inverse(K);
    auto T =  InvK*rho*b;
    std::cout << "The extrinsic:\n Rotation Matrix: \n"<< R << "\nTranslation Matrix: \n"<< T << std::endl;

    return true;
    // TODO: delete the above code in you final submission (which are just examples).
#endif
}

















