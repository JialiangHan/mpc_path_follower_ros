#pragma once
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>
#include <Eigen/SparseMatrixBase.h>
// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"
#include <algorithm>

namespace mpc_path_follower
{

    typedef Eigen::Matrix<double, n, n> MatrixA;
    typedef Eigen::Matrix<double, n, m> MatrixB;
    typedef Eigen::Vector4d VectorG;
    typedef Eigen::Vector4d VectorX;
    typedef Eigen::Vector2d VectorU;

    class MPC
    {
    public:
    private:
        void findPoint(const double &distance, double &x, double &y, double &heading, double &speed, double &steering_angle);

        void updateAdBdgd(const double &arc_length, double &x, double &y, double &last_phi);
        /**
         * @brief find trajectory length start from first of ref trajectory to current_location
         *
         * @param ref_trajectory
         * @param location
         * @return double
         */
        double findtrajetorylength(const std::vector<VectorX> &ref_trajectory, const VectorX &current_location);

        int findClosestIndex(const std::vector<VectorX> &ref_trajectory, const VectorX &current_location);
        int findClosestIndex(const std::vector<VectorX> &ref_trajectory, const double &distance);

        double findLength(const std::vector<VectorX> &ref_trajectory, const int &closest_index);

        double findDistance(const VectorX &p1, const VectorX &p2);
        /**
         * @brief calculate matrix A,B,g for x_k+1=Ad_*x_k+Bd_*u_k+gd_;
         *
         * @param phi
         * @param v
         * @param delta
         */
        void linearization(const double &phi, const double &v, const double &delta);
        /**
         * @brief consider a delay for state x0
         *
         * @param x0
         * @return VectorX
         */
        VectorX compensateDelay(const VectorX &x0);

        int solveMPC(const VectorX &x0_observe);
        /**
         * @brief just solve QP problem
         *
         * @param hessian
         * @param gradient
         * @param linearMatrix
         * @param lowerBound
         * @param upperBound
         * @return int 0 failed, 1 success
         */
        int solveQP(const Eigen::MatrixXd &hessian, const Eigen::MatrixXd &gradient, const Eigen::MatrixXd &linearMatrix, const Eigen::MatrixXd &lowerBound, const Eigen::MatrixXd &upperBound);

        Eigen::MatrixXd setupHessian();

        Eigen::MatrixXd setupGradient(const VectorX &x0_observe, const Eigen::SparseMatrix<double> &qx);
        //
        /**
         * @brief Set the State Constrain object.
         * -v_max <= v <= v_max, for x,y, theta ,there is no constrain
         *size: (N,1) <= (N,4N)*(4N,1)<=(N,1)
         *               /  x1  \
         *               |  x2  |
         *  lx_ <=  Cx_  |  x3  |  <= ux_
         *               | ...  |
         *               \  xN  /
         *
         * @return std::vector<Eigen::MatrixXd> : first is Cx_, second is lx_, last is ux_
         */
        std::vector<Eigen::MatrixXd> setStateConstrain(const double &v_max);
        /**
         * @brief Set the Control Constrain object

               *                  /  u0  \
               *                  |  u1  |
               *       lx <=  Cx  |  u2  |  <= ux
               *                  | ...  |
               *                  \ uN-1 /

         *
         * @return * std::vector<Eigen::MatrixXd>: first is Cx, second is lx, last is ux
         */
        std::vector<Eigen::MatrixXd> setControlConstrain(const double &a_max, const double &steering_angle_max, const double &steering_angle_rate_max);
        /**
         * @brief combine state constrain and control constrain, convert to QSQP format
         *
         * @return std::vector<Eigen::MatrixXd> first is Cx, second is lx, last is ux
         */
        std::vector<Eigen::MatrixXd> combineConstrain(const std::vector<Eigen::MatrixXd> &state_constrain, const std::vector<Eigen::MatrixXd> &control_constrain);
        /**
         * @brief set BB, AA, gg matrix
         *                 BB                AA
         * x1    /       B    0  ... 0 \    /   A \
         * x2    |      AB    B  ... 0 |    |  A2 |
         * x3  = |    A^2B   AB  ... 0 |u + | ... |x0 + gg
         * ...   |     ...  ...  ... 0 |    | ... |
         * xN    \A^(n-1)B  ...  ... B /    \ A^N /
         *
         *     X = BB * U + AA * x0 + gg
         * first element is BB
         * second is AA
         * last is gg
         *         @return std::vector<Eigen::MatrixXd> first is BB, second is AA, last is gg
         */
        std::vector<Eigen::MatrixXd>
        setupBBAAggmatrix(const int &predicted_length);
        /**
         * @brief      // cost function should be represented as follows:

             *           /  x1  \T       /  x1  \         /  x1  \
             *           |  x2  |        |  x2  |         |  x2  |
             *  J =  0.5 |  x3  |   Qx_  |  x3  | + qx^T  |  x3  | + const.
             *           | ...  |        | ...  |         | ...  |
             *           \  xN  /        \  xN  /         \  xN  /

         *
         * @return Eigen::SparseMatrix<double>
         */
        Eigen::SparseMatrix<double> setupqx(const VectorX &x0);

    private:
        int number_of_state_ = 4;   // state x y phi v
        int number_of_control_ = 2; // input a delta
        //    x_k+1=Ad_*x_k+Bd_*u_k+gd_;
        MatrixA Ad_;
        MatrixB Bd_;
        VectorG gd_;
        // vehicle length
        double ll_;
        // delta time
        double dt_;
        // time delay for state x0
        double delay_;
        // history control vector u
        std::vector<Eigen::MatrixXd> historyInput_;
        std::vector<Eigen::MatrixXd> predictInput_;
        // std::vector<Eigen::MatrixXd> predictState_;

        // weighting factor for cost function,
        Eigen::MatrixXd Qx_;
        // should be predicted length
        int N_;
        // should be reference trajectory
        std::vector<VectorX> ref_trajectory_;
        double desired_v_;

        Eigen::MatrixXd BB_;
        Eigen::MatrixXd AA_;
        Eigen::MatrixXd gg_;
    }
}
