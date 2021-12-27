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
    int wp_count = 1;
    int range = 4;
    int knotdiv = 4;
    double start = 0.0; 
    double end = 10.0;
    double traj_pub_rate = 20.0; // hz 
    ros::Rate rate(1/traj_pub_rate);
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
            }
        }
        std::cout << KBLU << "[Waypoint]:\n" << KNRM << wp << std::endl;

        /* 
        * Uniform Distribution
        */
        MatrixXd cp_raw(3,1); VectorXd time_waypoint(1);
        bsp.UniformDistribution(start_pose, wp, max_vel, knot_span, 
            &time_waypoint, &cp_raw);

        /* 
        * Clamp the Bspline
        */
        // std::cout << "M("<< order << "):\n" << bsp.createM(order) << std::endl;
        ros::Time tstart = ros::Time::now();
        MatrixXd cp = bsp.ClampBspline(order, cp_raw);

        double end = (cp.cols() - (order)) * knot_span;
        std::cout << KBLU << "[End time]: " << KNRM << end << std::endl;

        /* 
        * Bspline Creation using Header Function
        */
        MatrixXd pos(3,1); MatrixXd vel(3,1); MatrixXd acc(3,1); 
        VectorXd time(1);
        bsp.GetBspline3(order, cp, start, end, knotdiv, 
            &pos, &vel, &acc, &time);

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
        rate.sleep();
    }
    
    return 0;
}