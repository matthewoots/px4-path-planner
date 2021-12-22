/*
 * Bspline.h
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

using namespace Eigen;
using namespace std;

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
    class bspline
    {
        public:

        /* 
        * Uniform Distribution
        */
        void UniformDistribution(Vector3d start_pose, MatrixXd wp, double max_vel, double knot_span, 
            VectorXd *time_waypoint, MatrixXd *cp_raw)
        {
            MatrixXd keypoints = MatrixXd::Zero(3,wp.cols() + 1);
            keypoints.col(0) = start_pose;
            for (int j = 0; j < wp.cols(); j++)
            {
                keypoints.col(j+1) = wp.col(j);
            }
            std::cout << KRED << "[Keypoints]:\n" << KNRM << keypoints << std::endl;
            
            VectorXd diff = VectorXd::Zero(wp.cols());
            // Until cp_tmp.cols() - order - 1 since that is the last change in a clamped spline
            // But will be different for non-clamped splines 
            for (int j = 0; j < wp.cols(); j++)
            {
                Vector3d diff_tmp;
                diff_tmp = keypoints.col(j+1) - keypoints.col(j);
                diff(j) = sqrt(pow(diff_tmp[0],2) + pow(diff_tmp[1],2) + pow(diff_tmp[2],2));
            }
            std::cout << KRED << "[Length of Segment]:\n" << KNRM << diff << std::endl;

            double total_dist = diff.sum();
            double est_dist_knot = max_vel * knot_span;

            double total_segment = 0;
            VectorXd segment = VectorXd::Zero(diff.size());
            *time_waypoint = VectorXd::Zero(diff.size());
            VectorXd time_waypoint_tmp = VectorXd::Zero(diff.size());
            for (int j = 0; j < diff.size(); j++)
            {
                // ceil helps to push values above 0 to 1 or more
                // or else segment count is 0 and causes an error
                double knot_in_seg_count = ceil(diff(j)/est_dist_knot);
                segment(j) = knot_in_seg_count;
                total_segment += knot_in_seg_count;
                time_waypoint_tmp(j) = total_segment * knot_span;
            }
            *time_waypoint = time_waypoint_tmp;
            std::cout << KRED << "[Time to Waypoint]:\n" << KNRM << *time_waypoint << std::endl;
            std::cout << KRED << "[Segment Count]:\n" << KNRM << segment << std::endl;

            // Raw control points
            // total_segment + 1 is the total count since + 1 refers to the last missing point after linspace
            *cp_raw = MatrixXd::Zero(3,total_segment + 1);
            MatrixXd cp_raw_tmp = MatrixXd::Zero(3,total_segment + 1);
            for (int i = 0; i < 3; i++)
            {
                double segment_counter = 0;
                for (int j = 0; j < wp.cols(); j++)
                {
                    double div = segment(j) + 1.0;
                    MatrixXd sub_cp_tmp = linspace(keypoints(i,j), keypoints(i,j+1), div);
                    std::cout << KRED << "[sub_cp_tmp] " << j << ":\n" << KNRM << sub_cp_tmp << std::endl;
                    for (int k = 0; k < segment(j); k++)
                    {
                        cp_raw_tmp(i,k + segment_counter) = sub_cp_tmp(0,k);
                    }
                    segment_counter += segment(j);
                }
                cp_raw_tmp(i,segment_counter) = keypoints(i, keypoints.cols()-1);
            }
            *cp_raw = cp_raw_tmp;
            std::cout << KRED << "[cp_raw]:\n" << KNRM << *cp_raw << std::endl;
            std::cout << KRED << "[Size of cp_raw]: " << KNRM << cp_raw_tmp.cols() << std::endl;
            return;
        }

        /* 
        * Clamp the Bspline
        */
        MatrixXd ClampBspline(int order, MatrixXd cp_raw)
        {
            // Back and front will be kept empty till we clamp it
            // p of the first and last knots must be identical
            MatrixXd cp = MatrixXd::Zero(3,cp_raw.cols() + 2*(order-1));
            for (int i = 0; i < 3; i++)
            {
                // We use (order - 1, cols - order + 1), then we get the first and last values and clamp them
                // this will follow p in front and at the back
                for (int j = 0; j < cp_raw.cols(); j++)
                {
                    // Get value from cp_raw
                    cp(i,order - 1 + j) = cp_raw(i,j);
                }

                // Making it clamped (front)
                for (int j = 0; j < order - 1; j++)
                {
                    cp(i,j) = cp(i,order - 1);
                }
                
                // Making it clamped (back)
                for (int j = cp.cols() - order + 1; j < cp.cols(); j++)
                {
                    cp(i,j) = cp(i,cp.cols() - order);
                }
                // std::cout << "[range]:\n" << cp.cols() - order - 1 << " - " << cp.cols() << std::endl;
            }
            // std::cout << KRED << "[control_points]:\n" << KNRM << cp << std::endl;
            // std::cout << KRED << "[Size of control_points]: " << KNRM << cp.cols() << std::endl;
            return cp;
        }

        /*
        * Get Bspline in 3D while calling GetBspline
        */
        void GetBspline3(int order, MatrixXd cp, double start, double end, int knotdiv, 
            MatrixXd *pos, MatrixXd *vel, MatrixXd *acc, VectorXd *time)
        {
            if (cp.rows() != 3)
            {
                printf("Not a 3xn matrixXd format\n");
                return;
            }

            // Time instance of each knot
            MatrixXd t = linspace(start, end, (double)(cp.cols() - (order-1)));
            // std::cout << KRED << "[Size of t.cols]: " << KNRM << t.cols() << std::endl;
            // std::cout << KRED << "[Knots]: \n" << KNRM << t << std::endl;
            int size_span = (t.cols()-1) * knotdiv;
            // std::cout << KRED << "[Size of size_span]: " << KNRM << size_span << std::endl;

            // Resize matrix is called inside here
            *pos = MatrixXd::Zero(3,size_span); 
            *vel = MatrixXd::Zero(3,size_span); 
            *acc = MatrixXd::Zero(3,size_span); 
            *time = VectorXd::Zero(size_span);
            for (int i = 0; i < cp.rows(); i++)
            {
                VectorXd pos_tmp(size_span); VectorXd vel_tmp(size_span); VectorXd acc_tmp(size_span); 
                VectorXd time_tmp(size_span);
                GetBspline(order, cp.row(i), start, end, knotdiv, &pos_tmp, &vel_tmp, &acc_tmp, &time_tmp);
                pos->row(i) = pos_tmp; 
                vel->row(i) = vel_tmp; 
                acc->row(i) = acc_tmp;
                *time = time_tmp;
            }
            return;
        }
        
        /*
        * Get Bspline in 1D
        */
        void GetBspline(int order, MatrixXd cp, double start, double end, int knotdiv, 
            VectorXd *pos, VectorXd *vel, VectorXd *acc, VectorXd *time)
        {
            if (cp.rows() != 1)
            {
                printf("Not a 1xn matrixXd format\n");
                return;
            }
                
            int k = order + 1;
            int n = cp.cols() - 1;
            MatrixXd M = createM(order);
            MatrixXd u = MatrixXd::Zero(1,k); // Position Row Vector
            MatrixXd du = MatrixXd::Zero(1,k); // Velocity Row Vector
            MatrixXd ddu = MatrixXd::Zero(1,k); // Acceleration Row Vector
            MatrixXd dddu = MatrixXd::Zero(1,k); // Snap Row Vector
            MatrixXd p = MatrixXd::Zero(k,1); // Control Points in a Span Column vector
            // std::cout << "u:\n" << u << std::endl;
            // std::cout << "p:\n" << p << std::endl;

            int r = cp.cols() - (order-1);
            // Range is the amount of knots since we can expand from [order : end] 
            // since we require order number of cp to calculate the first segment 
            MatrixXd range = MatrixXd::Zero(1,r);
            
            int count = 0;
            for (int i = order-1; i < cp.cols(); i++)
            {
                range(i - (order-1)) = i;
            }
            // std::cout << KRED << "[range]: " << KNRM << range << std::endl;
            
            // dt would be the span of 1 knot which is the same length as range
            double dt = (end - start) / (double)(range.cols());
            // std::cout << KRED << "[dt]: " << KNRM << dt << std::endl;
            // std::cout << "dt:\n" << dt << std::endl;

            // time instance of each knot
            MatrixXd t = linspace(start, end, (double)(range.cols()));
            // std::cout << KRED << "[Size of t.cols]: " << KNRM << t.cols() << std::endl;
            // std::cout << KRED << "[Knots]: " << KNRM << t << std::endl;

            VectorXd pos_tmp = VectorXd::Zero((t.cols()-1) * (knotdiv)); 
            VectorXd vel_tmp = VectorXd::Zero((t.cols()-1) * (knotdiv));  
            VectorXd acc_tmp = VectorXd::Zero((t.cols()-1) * (knotdiv));  
            VectorXd time_tmp = VectorXd::Zero((t.cols()-1) * (knotdiv));
            // std::cout << KRED << "[Size of time_tmp]: " << KNRM << time_tmp.size() << std::endl;
            // MatrixXd jrk; 

            int idx_multiplier = 0;
            // total number of span will be t.cols()-1
            for (int l = 0; l < t.cols()-1; l++)
            {
                int idx = (int)range(l) - order + 1;
                // std::cout << KRED << "[Index]: " << KNRM << idx << std::endl;

                int nxt_idx = idx + 1; 
                MatrixXd tmpp = MatrixXd::Zero(1,knotdiv);
                MatrixXd tmpv = MatrixXd::Zero(1,knotdiv);
                MatrixXd tmpa = MatrixXd::Zero(1,knotdiv);
                // MatrixXd tmpj = MatrixXd::Zero(1,knotdiv-1);

                MatrixXd span = linspace(idx, nxt_idx, (double)knotdiv + 1); // relative to the start time as 0 regardless of the time 
                MatrixXd actualspan = linspace(t(l), t(l+1), (double)knotdiv + 1); // time in abs (simulation time / given time)
                // std::cout << KRED << "[Actual Span]: " << KNRM << actualspan << std::endl;

                if (idx < 0)
                {
                    printf("idx is below suggested idx for control points\n");
                    return;
                }

                if (idx + 1 >= cp.cols())
                {
                    printf("idx is out of bounds compared to control points\n");
                    return;
                }
                
                // Span column - 1 is because [0,1)
                for (int m = 0; m < span.cols()-1; m++)
                {
                    double time = span(m); // current time in index form, of course we dont like to play with conversion
                    double u_t = (time - (double)idx) / (double)((idx + 1) - idx); // using index is the same as using time since u_t is a factor
                    // std::cout << KRED << "[Actual Time]: " << KNRM << actualspan(m) << std::endl;

                    // p have several conventions according to SO many papers but i
                    // would use the convention under (1) and it is the correct one
                    // etc if order is 5
                    // (1) p = [P(idx-5) P(idx-4) P(idx-3) P(idx-2) P(idx-1) P(idx)]'

                    // Make the u, du, ddu and p matrix
                    for (int j = 0; j < k; j++)
                    {
                        u(j) = pow(u_t, j);
                        p(j) = cp(idx + j);
                        // std::cout << KRED << "[cp]: " << KNRM << cp(idx + j) << std::endl;
                        if (j >= 1)
                            du(j) = (j) * pow(u_t, j-1);
                        if (j >= 2)
                            ddu(j) = (j) * (j-1) * pow(u_t, j-2);
                        // if (j >= 3)
                        //     dddu(j) = (j) * (j-1) * (j-2) * pow(u_t, j-3);
                    }
                    // std::cout << KRED << "[p]: \n" << KNRM << p << std::endl;

                    double inv_dt = 1/dt;

                    // Matrix multiplication to attain the pos, vel and acc
                    tmpp(m) = (u * M * p)(0,0);
                    tmpv(m) = (inv_dt * du * M * p)(0,0);
                    tmpa(m) = (pow(inv_dt,2) * ddu * M * p)(0,0);
                    // tmpj(m) = inv_dt^3 * dddu * M * p;
                    // std::cout << KRED << "[tmpp_matrix0]: \n" << KNRM << (u * M * p)(0,0) << std::endl;
                }
                // std::cout << KRED << "[tmpp_matrix]: \n" << KNRM << tmpp << std::endl;
                
                for (int m = 0; m < span.cols()-1; m++)
                {
                    pos_tmp(m+idx_multiplier*(knotdiv)) = tmpp(m);
                    vel_tmp(m+idx_multiplier*(knotdiv)) = tmpv(m);
                    acc_tmp(m+idx_multiplier*(knotdiv)) = tmpa(m);
                    time_tmp(m+idx_multiplier*(knotdiv)) = actualspan(m);
                }

                idx_multiplier++;
                // std::cout << KRED << "[Value Range]: " << KNRM << idx_multiplier*(knotdiv) << std::endl;
            }
            // std::cout << KRED << "[Time]: \n" << KNRM << time_tmp << std::endl;
            *pos = pos_tmp;
            *vel = vel_tmp;
            *acc = acc_tmp;
            *time = time_tmp;
            return;
        }

        /*
        * Creating Bspline recursive M matrix
        * General matrix representations for B-splines See Theorem 1 of page 182 for getting M
        * https://link.springer.com/article/10.1007/s003710050206
        */
        MatrixXd createM(int order)
        {
            int k = order+1;
            MatrixXd M = MatrixXd::Zero(k,k);
            double f = 1 / factorial(k - 1);

            for (int i = 0; i < M.rows(); ++i)
            {
                for (int j = 0; j < M.cols(); ++j)
                {
                    
                    double fac = 0;
                    for (int s = j; s <= (k-1) ; s++)
                    {
                        double p21 = (double)(k-s-1);
                        double p22 = (double)(k-i-1);
                        double p11 = (double)(s-j);
                        fac += (pow(-1.0, p11) * getC(s-j, k) * pow(p21, p22)); 
                    }  
                    double m = getC(k-i-1, k-1) * fac;
                    m = f * m;
                    M(i, j) = m;            
                }
            }
            return M;
        }

        MatrixXd linspace(double min, double max, double n)
        {
            MatrixXd linspaced = MatrixXd::Zero(1,n);
            double delta = (max - min) / (n - 1.0);
            linspaced(0,0) = min;
            
            for (int i = 1; i < (int)n; i++)
            {
                linspaced(0,i) = linspaced(0,i-1) + delta;
            }
            return linspaced;
        }

        private:
        double getC(int i, int n)
        {
            return factorial(n)/(factorial(i) * factorial(n-i));
        }

        double factorial(int n)
        {
            double factorial = 1.0;
            for (int i = n; i > 1 ; i--)
            {
                factorial = factorial * (double)i;
            }
            return factorial;
        }
                
    };
    
}