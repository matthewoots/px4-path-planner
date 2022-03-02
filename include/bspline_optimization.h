/*
 * bspline_optimization.h
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2021 Matthew (matthewoots at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 * 
 * 
 */

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <LBFGSB.h>

using namespace Eigen;
using namespace std;
using Eigen::VectorXd;
using namespace LBFGSpp;

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

namespace bs
{
    class spline_opt_function
    {
    private:
        int col, row, n;

        double weight_smooth, weight_feas, weight_term; 
        double weight_static, weight_reci;
        double max_acc;
        double dt;

        double fx_smooth, fx_feas, fx_term;
        VectorXd grad_smooth, grad_feas, grad_term;

        MatrixXd reference_cp;

    public:
        spline_opt_function(int n_) : n(n_) {}

        void set_params(int _col, int _row, double _dt, double _max_acc)
        {
            col = _col; row = _row; dt = _dt; max_acc = _max_acc;
        }

        void load_data(MatrixXd _reference_cp)
        {
            reference_cp = MatrixXd::Zero(3,col);
            reference_cp = _reference_cp;
        }

        void set_weights(double _weight_smooth, double _weight_feas, double _weight_term, 
            double _weight_static, double _weight_reci)
        {
            weight_smooth = _weight_smooth; 
            weight_feas = _weight_feas;
            weight_term = _weight_term;
        }

        double operator()(const VectorXd& x, VectorXd& grad)
        {
            double fx = 0.0;
             
            double fx_static, fx_reci;
            
            VectorXd grad_static, grad_reci;
            
            MatrixXd cp = MatrixXd::Zero(3,col);
            for (int i = 0; i < row; i++)
            {
                for (int j = 0; j < col; j++)
                {
                    cp(i,j) = x[col * i + j];
                }
            }
            // std::cout << "cp : \n" << cp << std::endl;
            
            smoothnessCost(cp);
            feasibilityCost(cp);
            terminalCost(cp, reference_cp);
            // staticCollisionCost(x, &fx_static, &grad_static);
            // reciprocalAvoidanceCost(x, &fx_reci, &grad_reci);

            fx = weight_smooth * fx_smooth +
                weight_feas * fx_feas +
                weight_term * fx_term;
                // weight_static * fx_static +
                // weight_reci * fx_reci;

            for (int i = 0; i < col*row; i++)
                grad[i] = weight_smooth * grad_smooth(i) +
                   weight_feas * grad_feas(i) + 
                   weight_term * grad_term(i);
                // weight_static * grad_static +
                // weight_reci * grad_reci;

            // printf("%s[bspline_optimization.h] TM Cost %lf!\n", KBLU, weight_term * fx_term);
            // printf("%s[bspline_optimization.h] FS Cost %lf!\n", KBLU, weight_feas * fx_feas);
            // printf("%s[bspline_optimization.h] SM Cost %lf!\n", KBLU, weight_smooth * fx_smooth);
            // printf("%s[bspline_optimization.h] Total Cost %lf!\n", KCYN, fx);
            return fx;
        }

        void smoothnessCost(MatrixXd cp)
        {
            /*
            * Minimizing the integral over the squared derivatives 
            * (smoothness) such as acceleration, jerk, snap.
            */
            double cost = 0; 
            grad_smooth = VectorXd::Zero(col * row);

            // col*0 + (0 to col-1) = x 
            // col*1 + (0 to col-1) = y
            // col*2 + (0 to col-1) = z 

            MatrixXd gradient = MatrixXd::Zero(3,col);
            VectorXd grad_single = VectorXd::Zero(col * row);

            /* 3rd derivative, Jerk */
            for (int i = 0; i < cp.cols() - 3; i++)
            {
                Vector3d jerk = cp.col(i + 3) - 
                    3 * cp.col(i + 2) + 
                    3 * cp.col(i + 1) - 
                    cp.col(i);
                // printf("%s[bspline_optimization.h] Jerk [%lf %lf %lf]!\n", KBLU, jerk.x(), jerk.y(), jerk.z());

                cost = cost + pow(jerk.norm(), 2);
                Vector3d tmp_j = 2.0 * jerk;

                gradient.col(i + 0) += (-tmp_j);
                gradient.col(i + 1) += (3.0 * tmp_j);
                gradient.col(i + 2) += (-3.0 * tmp_j);
                gradient.col(i + 3) += (tmp_j);
            }
            
            /* 2nd derivative, Acceleration */
            for (int i = 0; i < cp.cols() - 2; i++)
            {   
                Vector3d acc = cp.col(i + 2) - 
                    2 * cp.col(i + 1) + 
                    cp.col(i);

                cost = cost + pow(acc.norm(), 2);
                Vector3d tmp_a = 2.0 * acc;
                
                gradient.col(i + 0) = (tmp_a);
                gradient.col(i + 1) = (-2.0 * tmp_a);
                gradient.col(i + 2) = (tmp_a);
            }

            for (int i = 0; i < row; i++)
                for (int j = 0; j < col; j++)
                    grad_smooth(col * i + j) = gradient(i,j);
            
            fx_smooth = cost;
        }

        void feasibilityCost(MatrixXd cp)
        {
            /*
            * Soft limit on the norm of time derivatives 
            * such as velocity, acceleration, jerk and snap.
            */
            double ts_inv2 = 1 / dt / dt;
            double cost = 0;
            grad_feas = VectorXd::Zero(col * row);

            MatrixXd gradient = MatrixXd::Zero(3,col);
            VectorXd grad_single = VectorXd::Zero(col * row);

            /* Check all instances with acceleration limit */
            for (int i = 0; i < cp.cols() - 2; i++)
            {
                Vector3d ai = (cp.col(i + 2) - 2 * cp.col(i+1) + cp.col(i)) * ts_inv2;
                for (int j = 0; j < 3; j++)
                {
                    if (ai(j) > max_acc)
                    {
                        cost += pow(ai(j) - max_acc, 2);

                        gradient(j, i + 0) += 2 * (ai(j) - max_acc) * ts_inv2;
                        gradient(j, i + 1) += (-4 * (ai(j) - max_acc) * ts_inv2);
                        gradient(j, i + 2) += 2 * (ai(j) - max_acc) * ts_inv2;
                    }
                    else if (ai(j) < -max_acc)
                    {
                        cost += pow(ai(j) + max_acc, 2);

                        gradient(j, i + 0) += 2 * (ai(j) + max_acc) * ts_inv2;
                        gradient(j, i + 1) += (-4 * (ai(j) + max_acc) * ts_inv2);
                        gradient(j, i + 2) += 2 * (ai(j) + max_acc) * ts_inv2;
                    }
                    else {}
                }
            }
            for (int i = 0; i < row; i++)
                for (int j = 0; j < col; j++)
                    grad_feas(col * i + j) = gradient(i,j);

            fx_feas = cost;
        }

        void terminalCost(MatrixXd cp, MatrixXd ref)
        {
            /*
            * Terminal cost function penalizes both position and 
            * velocity deviations from the desired values which comes from the global trajectory
            */
            double cost = 0;
            grad_term = VectorXd::Zero(col * row);

            MatrixXd gradient = MatrixXd::Zero(3,col);
            VectorXd grad_single = VectorXd::Zero(col * row);

            for (int i = 0; i < cp.cols(); i++)
            { 
                Vector3d diff = cp.col(i) - ref.col(i);
                double sq_diff = pow(diff.norm(), 2);
                cost = cost + sq_diff;
                gradient.col(i) = diff * 2;
            }

            for (int i = 0; i < row; i++)
                for (int j = 0; j < col; j++)
                    grad_term(col * i + j) = gradient(i,j);

            fx_term = cost;


        }

        void staticCollisionCost(VectorXd x, 
            double *fx, VectorXd *grad)
        {
            
        }

        void reciprocalAvoidanceCost(VectorXd x, 
            double *fx, VectorXd *grad)
        {
            /*
            * Reciprocal avoidance function that handles 
            * a collision-free path between agents
            */
        }

    };

    class bspline_optimization
    {
        private:

        VectorXd single_array;
        VectorXd lb;
        VectorXd ub;
        double min_height = 0.5, max_height = 2.7;

        double _weight_smooth, _weight_feas, _weight_term; 
        double _weight_static, _weight_reci;

        public:

        void load(double weight_smooth, double weight_feas, double weight_term, 
            double weight_static, double weight_reci)
        {
            _weight_smooth = weight_smooth; 
            _weight_feas = weight_feas;
            _weight_term = weight_term;
            _weight_static = weight_static;
            _weight_reci = weight_reci;
        }

        // Solver will solve with the given partial g_spline 
        // We should 
        MatrixXd solver(MatrixXd g_spline, double dt, MatrixXd fixed_cp, double max_acc)
        {
            // Set up parameters
            LBFGSBParam<double> param;  // New parameter class
            param.epsilon = 1e-6;
            // param.delta = 1e-3;
            param.max_iterations = 20;
            param.max_step = 0.07; // default is 1e-4
            param.max_linesearch = 20;
            // param.linesearch = LBFGS_LINESEARCH_BACKTRACKING_STRONG_WOLFE;


            int number_of_col = g_spline.cols();
            int number_of_row = g_spline.rows(); // Should be 3

            single_array = VectorXd::Zero(number_of_col * number_of_row);
            lb = VectorXd::Zero(number_of_col * number_of_row);
            ub = VectorXd::Zero(number_of_col * number_of_row);

            // We need to compress g_spline to a single array
            // g_spline -> g_single_array
            // MatrixXd to vector<double>
            // col*0 + (0 to col-1) = x 
            // col*1 + (0 to col-1) = y
            // col*2 + (0 to col-1) = z 
            for (int i = 0; i < number_of_row; i++)
                for (int j = 0; j < number_of_col; j++)
                    single_array(number_of_col*i + j) = g_spline(i,j);


            printf("%s[bspline_optimization.h] Single array size %lu! \n", KBLU, single_array.size());

            // Create solver and function object
            LBFGSBSolver<double> solver(param);  // New solver class
            spline_opt_function opt(number_of_col * number_of_row);


            printf("%s[bspline_optimization.h] Setup Bounds! \n", KBLU);
            // Bounds
            // Setup Lower Bound and upper bound for now we take z to be clamped
            for (int i = 0; i < number_of_row; i++)
            {
                for (int j = 0; j < number_of_col; j++)
                {
                    if (i != 2)
                    {
                        // Load in Lower and Upper bound
                        lb(number_of_col*i + j) = -5000;
                        ub(number_of_col*i + j) = 5000;
                    }
                    else
                    {
                        // If representing z
                        // Load in Lower and Upper bound
                        lb(number_of_col*i + j) = min_height;
                        ub(number_of_col*i + j) = max_height;
                    }

                }
            }

            // Initial guess = g_single_array which is x
            VectorXd x = single_array;

            

            // x will be overwritten to be the best point found
            double fx = 0; // Cost
            opt.set_params(number_of_col, number_of_row, dt, max_acc);
            opt.load_data(fixed_cp);

            // double _weight_smooth, double _weight_feas, double _weight_term, 
            // double _weight_static, double _weight_reci
            opt.set_weights(_weight_smooth,
                    _weight_feas,
                    _weight_term,
                    _weight_static,
                    _weight_reci);
            printf("%s[bspline_optimization.h] Set weights!\n", KBLU);
            
            int iter = solver.minimize(opt, x, fx, lb, ub);

            // g_single_array -> g_spline
            // vector<double> to MatrixXd

            printf("%s[bspline_optimization.h] Iterations %d!\n", KMAG, iter);
            printf("%s[bspline_optimization.h] F(x) %lf!\n", KGRN, fx);
            
            MatrixXd cp = MatrixXd::Zero(3,number_of_col);
            for (int i = 0; i < number_of_row; i++)
            {
                for (int j = 0; j < number_of_col; j++)
                {
                    cp(i,j) = x[number_of_col * i + j];
                }
            }
            return cp;
        }
                
    };
    
}