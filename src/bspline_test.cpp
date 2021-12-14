#include <bspline.h>
#include <cstdlib>
#include <string>
#include <Eigen/Dense>
#include <time.h>

#include "ros/ros.h"

using namespace Eigen;

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

bs::bspline bsp;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_path_planner");
    ros::NodeHandle n;
    
    int order = 3;
    // Row is like number of waypoints
    int row = 5; 
    int wp_count = 2;
    int range = 4;
    int knotdiv = 4;
    double start = 0.0; 
    double end = 10.0;
    double traj_pub_rate = 20.0; // hz 
    ros::Rate rate(1/traj_pub_rate);
    ros::Rate sleep_rate(0.1);
    double knot_span = (double)knotdiv * (1/traj_pub_rate);
    // std::cout << KBLU << "[knot_span]:\n" << KNRM << knot_span << std::endl;
    double max_vel = 1.0;
    Vector3d start_pose = Vector3d::Zero();

    while (ros::ok())
    {
        // We will try for clamped Bspline only for now

        /*
        * Set up waypoints
        */

        // Convention will be (start to last waypoint)
        MatrixXd wp = MatrixXd::Zero(3,wp_count);
        srand ( time(NULL) );
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < wp_count; j++)
            {
                wp(i,j) = (double)(rand() % range + 1) - (double)(range/2);
                // std::cout << KBLU << "[RNG]: " << KNRM << wp(i,j) << std::endl;
            }
        }
        MatrixXd keypoints = MatrixXd::Zero(3,wp_count + 1);
        keypoints.col(0) = start_pose;
        for (int j = 0; j < wp_count; j++)
        {
            keypoints.col(j+1) = wp.col(j);
        }
        std::cout << KBLU << "[Keypoints]:\n" << KNRM << keypoints << std::endl;

        /* 
        * Uniform Distribution
        */

        VectorXd diff = VectorXd::Zero(wp_count);
        // Until cp_tmp.cols() - order - 1 since that is the last change in a clamped spline
        // But will be different for non-clamped splines 
        for (int j = 0; j < wp.cols(); j++)
        {
            Vector3d diff_tmp;
            diff_tmp = keypoints.col(j+1) - keypoints.col(j);
            diff(j) = sqrt(pow(diff_tmp[0],2) + pow(diff_tmp[1],2) + pow(diff_tmp[2],2));
        }
        std::cout << KBLU << "[Length of Segment]:\n" << KNRM << diff << std::endl;

        double total_dist = diff.sum();
        double est_dist_knot = max_vel * knot_span;

        double total_segment = 0;
        VectorXd segment = VectorXd::Zero(diff.size());
        VectorXd time_waypoint = VectorXd::Zero(diff.size());
        for (int j = 0; j < diff.size(); j++)
        {
            // ceil helps to push values above 0 to 1 or more
            // or else segment count is 0 and causes an error
            double knot_in_seg_count = ceil(diff(j)/est_dist_knot);
            segment(j) = knot_in_seg_count;
            total_segment += knot_in_seg_count;
            time_waypoint(j) = total_segment * knot_span;
        }
        std::cout << KBLU << "[Time to Waypoint]:\n" << KNRM << time_waypoint << std::endl;
        std::cout << KBLU << "[Segment Count]:\n" << KNRM << segment << std::endl;

        // Raw control points
        // total_segment + 1 is the total count since + 1 refers to the last missing point after linspace
        MatrixXd cp_raw = MatrixXd::Zero(3,total_segment + 1);
        for (int i = 0; i < 3; i++)
        {
            double segment_counter = 0;
            for (int j = 0; j < wp_count; j++)
            {
                double div = segment(j) + 1.0;
                MatrixXd sub_cp_tmp = bsp.linspace(keypoints(i,j), keypoints(i,j+1), div);
                // std::cout << KBLU << "[sub_cp_tmp] " << j << ":\n" << KNRM << sub_cp_tmp << std::endl;
                for (int k = 0; k < segment(j); k++)
                {
                    cp_raw(i,k + segment_counter) = sub_cp_tmp(0,k);
                }
                segment_counter += segment(j);
            }
            cp_raw(i,segment_counter) = keypoints(i, keypoints.rows()-1);
        }
        // std::cout << KBLU << "[cp_raw]:\n" << KNRM << cp_raw << std::endl;
        // std::cout << KBLU << "[Size of cp_raw]: " << KNRM << cp_raw.cols() << std::endl;

        /* 
        * Clamp the Bspline
        */

        // std::cout << "M("<< order << "):\n" << bsp.createM(order) << std::endl;
        ros::Time tstart = ros::Time::now();
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
        std::cout << KBLU << "[control_points]:\n" << KNRM << cp << std::endl;
        std::cout << KBLU << "[Size of control_points]: " << KNRM << cp.cols() << std::endl;

        double end = (cp.cols() - (order)) * knot_span;
        std::cout << KBLU << "[End time]: " << KNRM << end << std::endl;

        /* 
        * Bspline Creation Function
        */
        MatrixXd t = bsp.linspace(start, end, (double)(cp.cols() - (order-1)));
        std::cout << KBLU << "[Size of t.cols]: " << KNRM << t.cols() << std::endl;
        std::cout << KBLU << "[Knots]: \n" << KNRM << t << std::endl;
        int size_span = (t.cols()-1) * knotdiv;
        std::cout << KBLU << "[Size of size_span]: " << KNRM << size_span << std::endl;

        MatrixXd pos = MatrixXd::Zero(3,size_span); MatrixXd vel = MatrixXd::Zero(3,size_span); MatrixXd acc = MatrixXd::Zero(3,size_span); 
        VectorXd time = VectorXd::Zero(size_span);
        for (int i = 0; i < cp.rows(); i++)
        {
            VectorXd pos_tmp(size_span); VectorXd vel_tmp(size_span); VectorXd acc_tmp(size_span); 
            VectorXd time_tmp(size_span);
            bsp.GetBspline(order, cp.row(i), start, end, knotdiv, &pos_tmp, &vel_tmp, &acc_tmp, &time_tmp);
            // std::cout << "[pos_tmp]:" << pos_tmp << std::endl;
            pos.row(i) = pos_tmp; vel.row(i) = vel_tmp; acc.row(i) = acc_tmp;
            time = time_tmp;
        }

        /* 
        * Results
        */
        double duration = (ros::Time::now() - tstart).toSec();
        std::cout << KBLU << "[Recorded Time]:\n" << KNRM << duration << std::endl;
        std::cout << KBLU << "[Pos]:\n" << KNRM << pos << std::endl;
        // std::cout << KBLU << "[Vel]:\n" << KNRM << vel << std::endl;
        // std::cout << KBLU << "[Acc]:\n" << KNRM << acc << std::endl;
        // std::cout << KBLU << "[Time]:\n" << KNRM << time << std::endl;
        order++;
        ros::spinOnce();
        sleep_rate.sleep();
    }
    
    return 0;
}