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
        MatrixXd global_control_points;


    public:
        spline_opt_function(int col_,
            MatrixXd global_control_points,
            double weight_smooth, double weight_feas, double weight_term, 
            double weight_static, double weight_reci, double weight_keyp) 
        {

        }

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
            feasibilityCost(x, &fx_smooth, &grad_smooth);
            terminalCost(x, &fx_smooth, &grad_smooth);
            staticCollisionCost(x, &fx_smooth, &grad_smooth);
            reciprocalAvoidanceCost(x, &fx_smooth, &grad_smooth);
            keypointPenaltyCost(x, &fx_smooth, &grad_smooth);

            return fx;
        }

        void smoothnessCost(MatrixXd x, 
            double *fx, MatrixXd *grad)
        {
            /*
            * Minimizing the integral over the squared derivatives 
            * (smoothness) such as acceleration, jerk, snap.
            */
        }

        void feasibilityCost(MatrixXd x, 
            double *fx, MatrixXd *grad)
        {
            /*
            * Soft limit on the norm of time derivatives 
            * such as velocity, acceleration, jerk and snap.
            */
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
        public:

        void solver(MatrixXd spline, MatrixXd g_spline)
        {
            // Set up parameters
            LBFGSBParam<double> param;  // New parameter class
            param.epsilon = 1e-6;
            param.max_iterations = 60;

            // Create solver and function object
            LBFGSBSolver<double> solver(param);  // New solver class
            spline_opt_function opt(spline.cols(), g_spline,
                0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
        }
                
    };
    
}