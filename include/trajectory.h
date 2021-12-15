#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <bspline.h>
#include <csv.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;

class trajectory
{
private:
    int _order, _knotdiv;
    ros::Time _start, _end;
    double _traj_pub_rate;

    MatrixXd _fixed_cp; // Does not change once set
    MatrixXd _global_cp; // Able to change for replanning
    VectorXd _knots;
    bs::bspline _bsp;

public:

    trajectory();
    ~trajectory();

    void InitialiseParam(int order, int knotdiv, ros::Time start, 
        ros::Time end, double traj_pub_rate)
    {
        _order = order;
        _knotdiv = knotdiv;
        _start = start; _end = end;
        _traj_pub_rate = traj_pub_rate;
    }

    void SetClampedPath(
        MatrixXd wp, double max_vel, Vector3d start_pose);

    MatrixXd GetLocalControlPoints(
        ros::Time start, double _finite_time_horizon);

    MatrixXd GetFixedControlPoints(){return _fixed_cp;}
    MatrixXd GetGlobalControlPoints(){return _global_cp;}

};
