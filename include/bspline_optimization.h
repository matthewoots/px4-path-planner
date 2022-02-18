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
#include <LBFGSB.h>

using namespace Eigen;
using namespace std;
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
        int col;

        double weight_smooth, weight_feas, weight_term; 
        double weight_static, weight_reci, weight_keyp;
        double max_acc;
        double dt;

        MatrixXd global_control_points;

    public:
        spline_opt_function(int col,
            MatrixXd global_control_points, double dt,
            double weight_smooth, double weight_feas, double weight_term, 
            double weight_static, double weight_reci, double weight_keyp) 
        {}

        double operator()(const MatrixXd& x, MatrixXd& grad)
        {
            double fx = 0.0;
            double fx_smooth, fx_feas, fx_term, fx_static, fx_reci, fx_keyp;
            MatrixXd grad_smooth, grad_feas, grad_term, grad_static, grad_reci, grad_keyp;
            // for(int i = 0; i < n; i += 2)
            // {
            //     double t1 = 1.0 - x[i];
            //     double t2 = 10 * (x[i + 1] - x[i] * x[i]);
            //     grad[i + 1] = 20 * t2;
            //     grad[i]     = -2.0 * (x[i] * grad[i + 1] + t1);
            //     fx += t1 * t1 + t2 * t2;
            // }
            smoothnessCost(x, &fx_smooth, &grad_smooth);
            feasibilityCost(x, &fx_feas, &grad_feas);
            terminalCost(x, &fx_term, &grad_term);
            staticCollisionCost(x, &fx_static, &grad_static);
            reciprocalAvoidanceCost(x, &fx_reci, &grad_reci);
            keypointPenaltyCost(x, &fx_keyp, &grad_keyp);

            fx = weight_smooth * fx_smooth +
                weight_feas * fx_feas +
                weight_term * fx_term +
                weight_static * fx_static +
                weight_reci * fx_reci +
                weight_keyp * fx_keyp;

            return fx;
        }

        void smoothnessCost(MatrixXd x, 
            double *fx, MatrixXd *grad)
        {
            /*
            * Minimizing the integral over the squared derivatives 
            * (smoothness) such as acceleration, jerk, snap.
            */
            double cost = 0; 
            *grad = MatrixXd::Zero(3,col);
            MatrixXd gradient = MatrixXd::Zero(3,col);
            MatrixXd cp = x;

            /* 3rd derivative, Jerk */
            for (int i = 0; i < cp.cols() - 3; i++)
            {
                Vector3d jerk = cp.col(i + 3) - 
                    3 * cp.col(i + 2) + 
                    3 * cp.col(i + 1) - 
                    cp.col(i);

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

            *fx = cost;
            *grad = gradient;
        }

        void feasibilityCost(MatrixXd x, 
            double *fx, MatrixXd *grad)
        {
            /*
            * Soft limit on the norm of time derivatives 
            * such as velocity, acceleration, jerk and snap.
            */
            double ts_inv2 = 1 / dt / dt;
            double cost = 0;
            *grad = MatrixXd::Zero(3,col);
            MatrixXd gradient = MatrixXd::Zero(3,col);
            MatrixXd cp = x;

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

            *fx = cost;
            *grad = gradient;
        }

        void terminalCost(MatrixXd x, 
            double *fx, MatrixXd *grad)
        {
            /*
            * Terminal cost function penalizes both position and 
            * velocity deviations from the desired values which comes from the global trajectory
            */
        }

        void staticCollisionCost(MatrixXd x, 
            double *fx, MatrixXd *grad)
        {
            
        }

        void reciprocalAvoidanceCost(MatrixXd x, 
            double *fx, MatrixXd *grad)
        {
            /*
            * Reciprocal avoidance function that handles 
            * a collision-free path between agents
            */
        }

        void keypointPenaltyCost(MatrixXd x, 
            double *fx, MatrixXd *grad)
        {
            /*
            * Keypoint penalty function which evaluates way-points 
            * at the specific time in the local trajectory
            */
        }

    };

    class bspline_optimization
    {
        private:
        double dt;

        public:

        void solver(MatrixXd spline, MatrixXd g_spline, double dt)
        {
            // Set up parameters
            LBFGSBParam<double> param;  // New parameter class
            param.epsilon = 1e-6;
            param.max_iterations = 60;

            // Create solver and function object
            LBFGSBSolver<double> solver(param);  // New solver class
            spline_opt_function opt(spline.cols(), g_spline, dt,
                0.1, 0.1, 0.1, 0.1, 0.1, 0.1);

            // To get the bound we have to pass through the search algorithm
            // Get the safe corridor for each control point
            // Add the bound contrains to the optimizer
            // In this case we may not need (static avoidance in optimizer = More efficient)

            // Bounds
            // VectorXd lb = VectorXd::Constant(n, 2.0);
            // VectorXd ub = VectorXd::Constant(n, 4.0);

            // Initial guess
            // VectorXd x = VectorXd::Constant(n, 3.0);

            // x will be overwritten to be the best point found
            double fx;
            // int niter = solver.minimize(fun, x, fx, lb, ub);

            // std::cout << niter << " iterations" << std::endl;
            // std::cout << "x = \n" << x.transpose() << std::endl;
            // std::cout << "f(x) = " << fx << std::endl;
        }
                
    };
    
}