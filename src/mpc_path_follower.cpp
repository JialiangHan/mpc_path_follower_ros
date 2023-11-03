/*********************************************************************
 * @file:   mpc_path_follower.cpp
 * @author: Lianchuan Zhang
 * @date:   2019.01.16
 * @version:1.0.1
 * @brief:  using mpc method to do path tracking
 *********************************************************************/

#include <mpc_path_follower/mpc_path_follower.h>

namespace mpc_path_follower
{

    FG_eval::FG_eval(Eigen::VectorXd coeffs, const int &predicted_length, const double &Lf, const double &dt)
    {
        this->coeffs = coeffs;
        predicted_length_ = predicted_length;
        Lf_ = Lf;
        dt_ = dt;
        DLOG(INFO) << "out of FG_eval.";
    }

    MPC_Path_Follower::MPC_Path_Follower(const int &predicted_length, const double &Lf, const double &dt)
    {
        predicted_length_ = predicted_length;
        Lf_ = Lf;
        dt_ = dt;
    }
    std::vector<double> MPC_Path_Follower::solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
    {
        DLOG(INFO) << "in solve.";
        // state: x,y,vehicle orientation angle, velocity,cross-track error. orientation error.
        x = state[0];
        y = state[1];
        psi = state[2];
        v = state[3];
        cte = state[4];
        epsi = state[5];
        DLOG(INFO) << "in line 39.";
        // number of independent variables
        // N timesteps == N - 1 actuation
        n_vars = predicted_length_ * 6 + (predicted_length_ - 1) * 2;
        // Number of constraints
        n_constraints = predicted_length_ * 6;
        // Initial value of the independent variables.
        // Should be 0 except for the initial values.
        // x_start to y_start-1 is x position
        // y_start to psi_start -1 is y position
        // psi_start to v_start -1 is vehicle orientation
        // v_start to cte_start -1 is velocity
        // cte_start to epsi_start -1 is tracking error
        // epsi_start to delta_start -1 is ????
        // delta_start to a_start-1 is ????
        // a_start to end is ???
        Dvector vars(n_vars);
        // reset to zero.
        for (int i = 0; i < n_vars; i++)
        {
            vars[i] = 0.0;
        }
        DLOG(INFO) << "size of vars is " << vars.size();
        // Set the initial variable values
        vars[x_start] = x;
        vars[y_start] = y;
        vars[psi_start] = psi;
        vars[v_start] = v;
        vars[cte_start] = cte;
        vars[epsi_start] = epsi;

        // Lower and upper limits for x
        Dvector vars_lowerbound(n_vars);
        Dvector vars_upperbound(n_vars);
        // TODO constraints are needed to be revised
        //  Set all non-actuators upper and lower limits
        //  to the max negative and positive values.
        //  constraints for [0,y_start -1] x position
        for (int i = 0; i < delta_start; i++)
        {
            vars_lowerbound[i] = -1.0e19;
            vars_upperbound[i] = 1.0e19;
        }

        // The upper and lower limits of delta are set to -25 and 25
        // degrees (values in radians).
        // NOTE: Feel free to change this to something else.
        for (int i = delta_start; i < a_start; i++)
        {
            vars_lowerbound[i] = -0.436332;
            vars_upperbound[i] = 0.436332;
        }

        // Acceleration/deceleration upper and lower limits.
        // NOTE: Feel free to change this to something else.
        for (int i = a_start; i < n_vars; i++)
        {
            vars_lowerbound[i] = -1.0;
            vars_upperbound[i] = 1.0;
        }

        // Lower and upper limits for constraints
        // All of these should be 0 except the initial
        // state indices.
        Dvector constraints_lowerbound(n_constraints);
        Dvector constraints_upperbound(n_constraints);
        for (int i = 0; i < n_constraints; i++)
        {
            constraints_lowerbound[i] = 0;
            constraints_upperbound[i] = 0;
        }
        constraints_lowerbound[x_start] = x;
        constraints_lowerbound[y_start] = y;
        constraints_lowerbound[psi_start] = psi;
        constraints_lowerbound[v_start] = v;
        constraints_lowerbound[cte_start] = cte;
        constraints_lowerbound[epsi_start] = epsi;

        constraints_upperbound[x_start] = x;
        constraints_upperbound[y_start] = y;
        constraints_upperbound[psi_start] = psi;
        constraints_upperbound[v_start] = v;
        constraints_upperbound[cte_start] = cte;
        constraints_upperbound[epsi_start] = epsi;

        DLOG(INFO) << "122th row.";
        // Object that computes objective and constraints
        FG_eval fg_eval(coeffs, predicted_length_, Lf_, dt_);
        // options
        std::string options;
        options += "Integer print_level  0\n";
        options += "Sparse  true        forward\n";
        options += "Sparse  true        reverse\n";
        // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
        // Change this as you see fit.
        options += "Numeric max_cpu_time          0.5\n";

        // place to return solution
        CppAD::ipopt::solve_result<Dvector> solution;
        // vars	initial argument value to start optimization procedure at.
        // vars_lowerbound	lower limit for argument during optimization
        // vars_upperbound	upper limit for argument during optimization
        // constraints_lowerbound	lower limit for g(x) during optimization.
        // constraints_upperbound	upper limit for g(x) during optimization.
        // fg_eval	function that evaluates the objective and constraints using the syntax
        // fg_eval(fg, x)
        // solution	structure that holds the solution of the optimization
        DLOG(INFO) << "before solve.";
        CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);
        DLOG(INFO) << "after solve";
        ok = true;
        // Check some of the solution values
        ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

        auto cost = solution.obj_value;
        DLOG(INFO) << "Cost is " << cost;
        // std::cout << "Cost " << cost << std::endl;

        result.push_back(solution.x[delta_start]);
        result.push_back(solution.x[a_start]);

        for (int i = 0; i < predicted_length_ - 1; i++)
        {
            result.push_back(solution.x[x_start + i + 1]);
            result.push_back(solution.x[y_start + i + 1]);
        }

        return result;
    }

}