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

#ifndef BSP_OPT_H
#define BSP_OPT_H

#include <helper.h>

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

// ============ Global variables ============
struct bspline_assembly 
{
    double msg_time;
    vector<double> knot_vector;
    vector<Vector3d> control_points;
};
std::map<int, bspline_assembly> uavsTraj; 
int uav_id;

// Used in Task.h and main.cpp

namespace bs
{
    class spline_opt_function
    {
    private:
        int col, row, n;

        double weight_smooth, weight_feas, weight_term; 
        double weight_static, weight_reci;
        double max_acc;
        double protected_zone;
        double dt;

        double fx_smooth, fx_feas, fx_term, fx_static, fx_reci;
        VectorXd grad_smooth, grad_feas, grad_term, grad_static, grad_reci;

        MatrixXd reference_cp;

        VectorXd current_knots;

        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obs_pcl;

    public:
        spline_opt_function(int n_) : n(n_) {}

        void set_params(int _col, int _row, double _dt, 
            double _max_acc, double _protected_zone)
        {
            col = _col; row = _row; dt = _dt; max_acc = _max_acc;
            protected_zone = _protected_zone;
        }

        void load_data(MatrixXd g_spline, MatrixXd _reference_cp, pcl::PointCloud<pcl::PointXYZ>::Ptr _query_pcl,
            VectorXd knots)
        {
            // Reset values
            reference_cp = MatrixXd::Zero(3,col);
            obs_pcl.clear();
            double factor = 4.0;

            for (int i = 0; i < g_spline.cols(); i++)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_obs(new pcl::PointCloud<pcl::PointXYZ>);
                tmp_obs = pcl2_filter_ptr(_query_pcl, g_spline.col(i), 
                    Vector3d(factor*protected_zone, factor*protected_zone, 
                    factor*protected_zone));
                obs_pcl.push_back(tmp_obs);
            }

            reference_cp = _reference_cp;

            current_knots = knots;
        }

        void set_weights(double _weight_smooth, double _weight_feas, double _weight_term, 
            double _weight_static, double _weight_reci)
        {
            weight_smooth = _weight_smooth; 
            weight_feas = _weight_feas;
            weight_term = _weight_term;
            weight_static = _weight_static;
            weight_reci = _weight_reci;
        }

        double operator()(const VectorXd& x, VectorXd& grad)
        {
            double fx = 0.0;            
            
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
            // printf("%s[bspline_optimization.h] SM Cost %lf!\n", KBLU, weight_smooth * fx_smooth);
            feasibilityCost(cp);
            // printf("%s[bspline_optimization.h] FS Cost %lf!\n", KBLU, weight_feas * fx_feas);
            terminalCost(cp, reference_cp);
            // printf("%s[bspline_optimization.h] TM Cost %lf!\n", KBLU, weight_term * fx_term);
            // double prev_SA = ros::Time::now().toSec();
            staticCollisionCost(cp);
            // printf("%s[bspline_optimization.h] SA Cost %lf!\n", KBLU, weight_static * fx_static);
            // printf("%s[bspline_optimization.h] SA Run Time %lf!\n", KBLU, ros::Time::now().toSec() - prev_SA);
            reciprocalAvoidanceCost(cp, current_knots);
            // printf("%s[bspline_optimization.h] RA Cost %lf!\n", KBLU, weight_reci * fx_reci);
            // printf("%s[bspline_optimization.h] ----------------------------\n", KYEL);

            // printf("%s[bspline_optimization.h] SM %lf | FS %lf | TM %lf | SA %lf | RA %lf\n", KYEL,
            //     weight_smooth * fx_smooth, weight_feas * fx_feas, weight_term * fx_term, 
            //     weight_static * fx_static, weight_reci * fx_reci);

            
            // printf("%s[bspline_optimization.h] Total Cost %lf!\n", KCYN, fx);

            fx = weight_smooth * fx_smooth +
                weight_feas * fx_feas +
                weight_term * fx_term +
                weight_static * fx_static +
                weight_reci * fx_reci;

            for (int i = 0; i < col*row; i++)
                grad[i] = weight_smooth * grad_smooth(i) +
                    weight_feas * grad_feas(i) + 
                    weight_term * grad_term(i) +
                    weight_static * grad_static(i) +
                    weight_reci * grad_reci(i);
            // std::cout << KYEL << "[bspline_optimization.h] T " << grad.transpose() << std::endl;


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

        void staticCollisionCost(MatrixXd cp)
        {
            double cost = 0;
            grad_static = VectorXd::Zero(col * row);

            MatrixXd gradient = MatrixXd::Zero(3,col);
            VectorXd grad_single = VectorXd::Zero(col * row);

            double pz_expansion_factor = 2.0;

            // printf("%s[bspline_optimization.h] SA Cost %lf!\n", KBLU, weight_static * fx_static);

            for (int i = 0; i < cp.cols(); i++)
            { 
                // For each iteration we should calc this before hand to reduce the load
                // double prev_filter = ros::Time::now().toSec();
                // pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_obs(new pcl::PointCloud<pcl::PointXYZ>);
                // tmp_obs = pcl2_filter_ptr(obs_pcl, cp.col(i), 
                //     Vector3d(1.2*protected_zone, 1.2*protected_zone, 1.2*protected_zone));
                // printf("%s[bspline_optimization.h] Filtered!\n", KBLU);
                // printf("%s[bspline_optimization.h] SA : filter Run Time %lf!\n", KCYN, ros::Time::now().toSec() - prev_filter);
                pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_obs(new pcl::PointCloud<pcl::PointXYZ>);
                tmp_obs = obs_pcl[i];

                if (tmp_obs->points.size() == 0)
                    continue;
                // printf("%s[bspline_optimization.h] tmp_obs size %d!\n", KBLU, tmp_obs->points.size());

                double prev_kd_tree = ros::Time::now().toSec();
                vector<Vector3d> kd_points = kdtree_find_points_pcl(cp.col(i), tmp_obs, 
                    pz_expansion_factor * protected_zone, 10);
                // printf("%s[bspline_optimization.h] SA : KDtree Run Time %lf!\n", KCYN, ros::Time::now().toSec() - prev_kd_tree);
                // printf("%s[bspline_optimization.h] kd tree search!\n", KBLU);

                if (kd_points.size() == 0)
                    continue;
                // printf("%s[bspline_optimization.h] kd point size %d!\n", KBLU, kd_points.size());
                
                double prev_indi_points = ros::Time::now().toSec();

                // ** Size is too big and optimization may take a few seconds
                for (int j = 0; j < kd_points.size(); j++)
                {
                    // Vector3d diff = cp.col(i) - kd_points[j];
                    Vector3d diff = kd_points[j] - cp.col(i);
                    Vector3d diff_dir = diff / diff.norm();
                    double sq_diff = pow(diff.norm(), 2);
                    double sq_protected = pow(pz_expansion_factor * protected_zone, 2);
                    gradient.col(i) = gradient.col(i) + diff_dir * (sq_protected - sq_diff);
                    cost = cost + (sq_protected - sq_diff);
                    // printf("%s[bspline_optimization.h] cost %lf gradient [%lf %lf %lf]!\n", KBLU, cost, 
                        // gradient.col(i).x(), gradient.col(i).y(), gradient.col(i).z());
                }               
                // printf("%s[bspline_optimization.h] updated cost and gradient!\n", KBLU);
                // printf("%s[bspline_optimization.h] SA : single point Run Time %lf!\n", KCYN, ros::Time::now().toSec() - prev_indi_points);
            }

            for (int i = 0; i < row; i++)
                for (int j = 0; j < col; j++)
                    grad_static(col * i + j) = gradient(i,j);

            // std::cout << KYEL << "[bspline_optimization.h] S " << grad_static.transpose() << std::endl;

            fx_static = cost;

        }

        void reciprocalAvoidanceCost(MatrixXd cp, VectorXd knots)
        {
            /*
            * Reciprocal avoidance function that handles 
            * a collision-free path between agents
            */
            double cost = 0;
            double tolerance = knots(1) - knots(0);
            // std::cout << KGRN << "[bspline_optimization.h] tolerance " << tolerance << std::endl;

            grad_reci = VectorXd::Zero(col * row);

            MatrixXd gradient = MatrixXd::Zero(3,col);
            VectorXd grad_single = VectorXd::Zero(col * row);

            const double CLEARANCE = protected_zone * 4.0;
            constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

            // std::cout << KYEL << "[bspline_optimization.h] cp_cols " << cp.cols() << 
            //     " knots_size " << knots.size() << std::endl;
            for (int i = 0; i < cp.cols(); i++)
            {
                // std::cout << KYEL << "[bspline_optimization.h] knots " << i << " " << knots(i) << std::endl;
                double current_knot = knots(i);
                std::map<int, bspline_assembly>::iterator itr;
                for (itr = uavsTraj.begin(); itr != uavsTraj.end(); ++itr)
                {   
                    // std::cout << KYEL << "[bspline_optimization.h] idx " << itr->first << 
                    //     " knot_vector_size() " << itr->second.knot_vector.size() << std::endl;
                    // If id is same we skip
                    if (itr->first == uav_id)
                        continue;
                    
                    // Since there is no knots, it means the drone is stationary (hovering etc)
                    if (itr->second.knot_vector.size() == 0)
                    {
                        if (current_knot - knots(0) > 2.0)
                            continue; 
                        Vector3d dist_vec = cp.col(i) - itr->second.control_points[0];
                        double ellip_dist = sqrt(dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2);

                        double dist_err = CLEARANCE - ellip_dist;

                        Vector3d Coeff = Vector3d(
                            -2 * (CLEARANCE / ellip_dist - 1) * inv_b2,
                            -2 * (CLEARANCE / ellip_dist - 1) * inv_b2,
                            -2 * (CLEARANCE / ellip_dist - 1) * inv_a2
                        );

                        if (dist_err < 0 || dist_vec.norm() == 0)
                        {
                            // Empty
                        }
                        else
                        {
                            // std::cout << KYEL << "[bspline_optimization.h] dist_err " << dist_err 
                            //     << " magnitude " << magnitude << " dist_vec " << dist_vec.norm() << std::endl;
                            cost = cost + pow(dist_err, 2);
                            gradient.col(i) = gradient.col(i) + (Coeff.array() * dist_vec.array()).matrix();
                        }
                        continue;
                    }

                    int knot_idx = -1;
                    double magnitude = 1;
                    
                    // Match the timestamp of the knots
                    for (int j = 0; j < itr->second.knot_vector.size(); j++)
                    {
                        // std::cout << fixed;                        
                        if(abs(itr->second.knot_vector[j] - current_knot) < tolerance)
                        {
                            // std::cout << KBLU << "[bspline_optimization.h] " << itr->second.knot_vector[j] << 
                            //     " " << current_knot << " knot_difference " << abs(itr->second.knot_vector[j] - current_knot) << std::endl;
                            
                            // magnitude = (tolerance - abs(itr->second.knot_vector[j] - current_knot)) / tolerance; 
                            knot_idx = j;
                            break;
                        }
                    }
                    // std::cout << KYEL << "[bspline_optimization.h] knot_idx " << knot_idx << std::endl;

                    if (knot_idx < 0)
                        continue;

                    Vector3d dist_vec = cp.col(i) - itr->second.control_points[knot_idx];
                    double ellip_dist = sqrt(dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2);
                    
                    double dist_err = CLEARANCE - ellip_dist;

                    Vector3d Coeff = Vector3d(
                        -2 * (CLEARANCE / ellip_dist - 1) * inv_b2,
                        -2 * (CLEARANCE / ellip_dist - 1) * inv_b2,
                        -2 * (CLEARANCE / ellip_dist - 1) * inv_a2
                    );

                    if (dist_err < 0 || dist_vec.norm() == 0)
                    {
                        // Empty
                    }
                    else
                    {
                        // std::cout << KYEL << "[bspline_optimization.h] dist_err " << dist_err 
                        //     << " magnitude " << magnitude << " dist_vec " << dist_vec.norm() << std::endl;
                        cost = cost + magnitude * pow(dist_err, 2);
                        gradient.col(i) = gradient.col(i) + magnitude * (Coeff.array() * dist_vec.array()).matrix();
                    }
                }
            }

            for (int i = 0; i < row; i++)
                for (int j = 0; j < col; j++)
                    grad_reci(col * i + j) = gradient(i,j);

            fx_reci = cost;

        }

        /* 
        * @brief Filter point cloud with the dimensions given
        */
        pcl::PointCloud<pcl::PointXYZ>::Ptr 
            pcl2_filter_ptr(pcl::PointCloud<pcl::PointXYZ>::Ptr _pc, 
            Vector3d centroid, Vector3d dimension)
        {   
            pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

            float minX = centroid.x() - dimension.x()/2;
            float maxX = centroid.x() + dimension.x()/2;

            float minY = centroid.y() - dimension.y()/2;
            float maxY = centroid.y() + dimension.y()/2;

            float minZ = centroid.z() - dimension.z()/2;
            float maxZ = centroid.z() + dimension.z()/2;

            pcl::CropBox<pcl::PointXYZ> box_filter;
            box_filter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
            box_filter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));

            box_filter.setInputCloud(_pc);
            box_filter.filter(*output);

            // printf("%s[rrt_standalone.h] return fromPCLPointCloud2! \n", KBLU);
            return output;
        }

    };

    class bspline_optimization
    {
        private:

        VectorXd single_array;
        VectorXd lb;
        VectorXd ub;
        double min_height, max_height;

        double _weight_smooth, _weight_feas, _weight_term; 
        double _weight_static, _weight_reci;
        VectorXd _knots;

        public:

        void load(double weight_smooth, double weight_feas, double weight_term, 
            double weight_static, double weight_reci, VectorXd knots)
        {
            _weight_smooth = weight_smooth; 
            _weight_feas = weight_feas;
            _weight_term = weight_term;
            _weight_static = weight_static;
            _weight_reci = weight_reci;
            _knots = knots;
        }

        // Solver will solve with the given partial g_spline 
        // We should 
        MatrixXd solver(MatrixXd g_spline, double dt, 
            MatrixXd fixed_cp, double max_acc, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr _obs, double solo_protected_zone,
            double solo_min_height, double solo_max_height)
        {
            // Seems like LBFGS-B is using "More Thuente" for line search by default

            // Set up parameters
            LBFGSBParam<double> param;  // New parameter class
            param.epsilon = 1e-6;
            // param.delta = 1e-3;
            param.max_iterations = 20; // Stable is 20
            param.max_step = 0.20; // default is 1e-4
            param.max_linesearch = 20; // Stable is 20
            // param.linesearch = LBFGS_LINESEARCH_BACKTRACKING_STRONG_WOLFE;

            min_height = solo_min_height;
            max_height = solo_max_height;

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


            // printf("%s[bspline_optimization.h] Setup Bounds! \n", KBLU);

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
            opt.set_params(number_of_col, number_of_row, dt, max_acc, solo_protected_zone);
            opt.load_data(g_spline, fixed_cp, _obs, _knots);

            // std::cout << KBLU << "[bspline_optimization.h] knots \n" << _knots.transpose() << std::endl;

            // double _weight_smooth, double _weight_feas, double _weight_term, 
            // double _weight_static, double _weight_reci
            opt.set_weights(_weight_smooth,
                    _weight_feas,
                    _weight_term,
                    _weight_static,
                    _weight_reci);
            // printf("%s[bspline_optimization.h] Set weights!\n", KBLU);
            
            int iter = solver.minimize(opt, x, fx, lb, ub);

            // g_single_array -> g_spline
            // vector<double> to MatrixXd

            printf("%s[bspline_optimization.h] Iterations %s%d! %sF(x) %s%lf!\n", 
                KGRN, KNRM, iter, KGRN, KNRM, fx);
            
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

#endif