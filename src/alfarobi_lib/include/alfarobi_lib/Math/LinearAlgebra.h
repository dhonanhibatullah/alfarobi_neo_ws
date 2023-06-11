/* LinearAlgebra.h 
    * Created by Dhonan Nabil Hibatullah, Alfarobi v12
    * dhonan.hibatullah@gmail.com, open for any questions
*/   

#ifndef LINEAR_ALGEBRA_H
#define LINEAR_ALGEBRA_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <alfarobi_lib/Eigen/Dense>




namespace alfarobi {



    /* normedMatError() function
        * This function is used for finding the absolute normalized error between two matrices
    */
    double normedMatError(Eigen::MatrixXd A0, Eigen::MatrixXd A1);



    /* solveDare() function
        * This function is used for evaluating the P in Discrete Algebraic Riccati Equation (DARE):
        *   P = (A^T)PA - ((A^T)PB)(R + (B^T)PB)^(-1)((B^T)PA) + Q
    */
    void solveDare(Eigen::MatrixXd& P, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R);
}


#endif