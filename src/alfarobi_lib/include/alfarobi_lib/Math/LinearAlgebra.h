/* LinearAlgebra.h 
    * Created by Dhonan Nabil Hibatullah, Alfarobi v12
    * dhonan.hibatullah@gmail.com, open for any questions
*/   

#define _USE_MATH_DEFINES
#include <cmath>
#include <alfarobi_lib/Eigen/Dense>




namespace alfarobi {



    /* normedMatError() function
        * This function is used for finding the absolute normalized error between two matrices
    */
    double normedMatError(Eigen::MatrixXd A0, Eigen::MatrixXd A1) {
        Eigen::MatrixXd diff        = A1 - A0;
        double          entry_sum   = 0.,
                        diff_sum    = 0.;

        for(uint16_t i = 0; i < A1.rows(); ++i) {
            for(uint16_t j = 0; j < A1.cols(); ++j) {
                entry_sum += fabs(A1(i, j));
                diff_sum  += fabs(diff(i, j));
            }
        }

        return diff_sum/entry_sum;
    }



    /* solveDare() function
        * This function is used for evaluating the P in Discrete Algebraic Riccati Equation (DARE):
        *   P = (A^T)PA - ((A^T)PB)(R + (B^T)PB)^(-1)((B^T)PA) + Q
    */
    void solveDare(Eigen::MatrixXd& P, Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R) {
        double stoppage_err = 0.0001;
        Eigen::MatrixXd A_0 = A,
                        G_0 = B*R.inverse()*B.transpose(),
                        H_0 = Q,
                        A_1 = A_0,
                        G_1 = G_0,
                        H_1 = H_0,
                        I   = Eigen::MatrixXd::Identity(Q.rows(), Q.cols());

        do {
            /* Update previous value */
            A_0 = A_1;
            G_0 = G_1;
            H_0 = H_1;

            /* Calculate repetitive term */
            Eigen::MatrixXd C_inv = (I + G_0*H_0).inverse();

            /* Determine the next term */
            A_1     = A_0*C_inv*A_0;
            G_1     = G_0 + A_0*C_inv*G_0*A_0.transpose();
            H_1     = H_0 + A_0.transpose()*H_0*C_inv*A_0;

        } while(normedMatError(H_1, H_0) > stoppage_err);

        /* Return the result to P */
        P = H_1;
    }
}