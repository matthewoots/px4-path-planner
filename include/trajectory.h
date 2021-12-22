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
    double _start, _end;
    double _traj_pub_rate;
    double _knot_span;

    MatrixXd _fixed_cp; // Does not change once set  
    MatrixXd _global_cp; // Able to change for replanning
    MatrixXd _local_cp;

    VectorXd _fixed_knots;
    bs::bspline _bsp;

    MatrixXd _pos; MatrixXd _vel; MatrixXd _acc; 
    VectorXd _yaw; VectorXd _time;

public:

    // trajectory();
    // ~trajectory();

    void Initialise(int order, int knotdiv, double traj_pub_rate);

    void SetClampedPath(
        MatrixXd wp, double max_vel, Vector3d start_pose);
    
    /** 
    * @brief Update Partial Path with Local Control Points
    * According to Finite Time Horizon
    */
    bool UpdatePartialPath(
        double now, double _finite_time_horizon);

    /** 
    * @brief Update Full Path
    */
    bool UpdateFullPath();

    /** 
    * @brief Get Desired State
    */
    bool GetDesiredState(double now, 
        Vector3d *pos, Vector3d *vel, Vector3d *acc);
    
    /** 
    * @brief Get Local Control Points
    */
    MatrixXd GetLocalControlPoints(
        double now, double _finite_time_horizon);

    /** 
    * @brief Check if trajectory is completed
    */
    bool isCompleted(double now);

    /** 
    * @brief Return control point pose according to idx
    */
    bool returnControlPointPose(int idx, Vector3d *pos);

    int returnFixedCPColumn(){return _fixed_cp.cols();};

    double GetStartTime(){return _start;};

    double GetEndTime(){return _end;};

    MatrixXd GetFixedControlPoints(){return _fixed_cp;};

    MatrixXd GetGlobalControlPoints(){return _global_cp;};


};
