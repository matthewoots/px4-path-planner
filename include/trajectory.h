/*
 * trajectory.h
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
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <bspline.h>
#include <bspline_optimization.h>
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
    bs::bspline_optimization _bsp_opt;

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
    * @brief Get Desired Yaw
    */
    double GetDesiredYaw(double now, double default_yaw);

    /** 
    * @brief Return control point pose according to idx
    */
    bool returnControlPointPose(int idx, Vector3d *pos);
    
    double GetStartTime(){return _start;};
    double GetEndTime(){return _end;};

    MatrixXd GetFixedControlPoints(){return _fixed_cp;};
    MatrixXd GetGlobalControlPoints(){return _global_cp;};

    VectorXd GetKnots(){return _fixed_knots;};

    int GetOrder(){return _order;};
    int GetKnotDivision(){return _knotdiv;};
    int returnFixedCPColumn(){return _fixed_cp.cols();};

};
