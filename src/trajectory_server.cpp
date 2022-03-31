/*
 * trajectory_server.cpp
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
#include <trajectory.h>


void trajectory::Initialise(int order, int knotdiv, double traj_pub_rate)
{
    _order = order;
    _knotdiv = knotdiv;
    _traj_pub_rate = traj_pub_rate;
    _fixed_cp = MatrixXd::Zero(3,1); 
    _global_cp = MatrixXd::Zero(3,1);
    _fixed_knots = VectorXd::Zero(1);
    _knot_span = (double)knotdiv * (1/traj_pub_rate);
    std::cout << KYEL << "[trajectory_server.cpp] " << "Initialise Complete" << KNRM << std::endl;
}

void trajectory::SetClampedPath(MatrixXd wp, 
    double max_vel, Vector3d start_pose) 
{
    /* 
    * Uniform Distribution
    */
    MatrixXd cp_raw = MatrixXd::Zero(3,1); VectorXd time_waypoint = VectorXd::Zero(1);

    _bsp.UniformDistribution(start_pose, wp, max_vel, _knot_span, 
        &time_waypoint, &cp_raw);
    std::cout << KYEL << "[trajectory_server.cpp] " << "Uniform Distribution Complete" << KNRM << std::endl;

    /* 
    * Clamp the Bspline
    */
    // Update global control points and start and end time 
    _global_cp = _bsp.ClampBspline(_order, cp_raw);
    std::cout << KYEL << "[trajectory_server.cpp] " << "Clamping Bspline" << KNRM << std::endl;

    _fixed_cp = MatrixXd::Zero(3,_global_cp.cols());
    _fixed_cp = _global_cp;
    _start = ros::Time::now().toSec();
    ros::Time _start_ros = ros::Time::now();
    std::cout << KYEL << "[trajectory_server.cpp] " << "Start time in sec : " << KNRM << _start << std::endl;
    
    int knots_size = (_global_cp.cols() - (_order-1));
    _fixed_knots = VectorXd::Zero(knots_size);
    for(int i = 0; i < knots_size; i++)
    {
        ros::Time time_tmp = _start_ros + ros::Duration(_knot_span * i);
        _fixed_knots(i) = time_tmp.toSec();
    }

    std::cout << KYEL << "[trajectory_server.cpp] " << "Clamping Complete" << KNRM << std::endl;
}

/** 
* @brief Update Local Control Points
* According to Finite Time Horizon
*/
// A requirement is that now must be an interval of the knots 
bool trajectory::UpdatePartialPath(
    double now, double _finite_time_horizon)
{
    // Now to _finite_time_horizon
    start_to_end_idx = ceil(_finite_time_horizon / _knot_span);

    start_idx = -1;

    // Check if anything to evaluate in knots
    for (int i = 0; i < _fixed_knots.size(); i++)
    {
        if (now - _fixed_knots(i) <= 0)
        {
            start_idx = i;

            // We need to move the cp back since order number before the start of the trajectory is needed
            // We should take the immediate point after since we do not want a jerk at the start
            if (start_idx - _order < 0)
                start_idx = _order + 1;

            if (start_idx + start_to_end_idx > (_fixed_knots.size()-1))
            {
                int end_idx = (_fixed_knots.size()-1);
                start_to_end_idx = end_idx - start_idx;
            }
            // else
            //     int end_idx = i + start_to_end_idx;

            break;
        }    
    }
    if (start_to_end_idx < _order)
        return false;

    // if no early break, return update path as false
    // This will give the 1st check that the agent is approximately close to agent
    if (start_idx < 0)
        return false; 

    // Truncate control points from global to local
    _local_cp = MatrixXd::Zero(3,start_to_end_idx);
    _local_fixed_cp = MatrixXd::Zero(3,start_to_end_idx);
    _local_knots = VectorXd::Zero(start_to_end_idx);

    for(int i = 0; i < _local_cp.cols(); i++)
    {
        _local_cp.col(i) = _global_cp.col(start_idx + i);
        _local_knots(i) = _fixed_knots(start_idx + i);
        _local_fixed_cp.col(i) = _fixed_cp.col(start_idx + i);
    }

    // std::cout << KYEL << "[trajectory_server.cpp] " << "Partial Path Update Complete " 
    //     << KNRM << _local_cp.cols() << " and " << _local_knots.size() << std::endl;
    return true;
}

/** 
* @brief Update Global Path with Local Control Points
* According to Finite Time Horizon
*/
void trajectory::UploadPartialtoGlobal(MatrixXd opt_cp, int id)
{
    std::lock_guard<std::mutex> t_lock(BsplineTrajMutex);
    
    int end_cols;
    // Make sure opt cp does not go over global_cp size
    if (start_idx + _local_cp.cols() > _global_cp.cols())
        end_cols = _global_cp.cols() - start_idx;
    else 
        end_cols = _local_cp.cols();

    for(int i = 0; i < end_cols; i++) 
        _global_cp.col(start_idx + i) = opt_cp.col(i);

}

/** 
* @brief Update Full Path
*/
bool trajectory::UpdateFullPath()
{
    std::lock_guard<std::mutex> t_lock(BsplineTrajMutex);
    // Reset pos, vel, acc and time
    _pos = MatrixXd::Zero(3,1); _vel = MatrixXd::Zero(3,1); _acc = MatrixXd::Zero(3,1); 
    _time = VectorXd::Zero(1);

    double start = _fixed_knots(0); double end = _fixed_knots(_fixed_knots.size()-1);

    // Bspline Creation using Function
    _bsp.GetBspline3(_order, _global_cp, start, end, _knotdiv, 
        &_pos, &_vel, &_acc, &_time);

    std::cout << KYEL << "[trajectory_server.cpp] " << "Full Path Update Complete" << KNRM << std::endl;
    return true;
}

/** 
* @brief Get Desired State
*/
// A requirement is that now must be an rounded to division of the knots
bool trajectory::GetDesiredState(double now, 
    Vector3d *pos, Vector3d *vel, Vector3d *acc)
{
    bool early_break = false;
    int idx;
    // Check the time now and compare to knots
    for(int i = 0; i < _time.size(); i++)
    {
        if(abs(_time(i) - now) < 0.001)
        {
            idx = i;
            // std::cout << KYEL << "[trajectory_server.cpp] " << "idx " << idx << KNRM << std::endl;
            early_break = true;
            break;
        }
    }
    // if no early break, return update path as false
    // This will give the 2nd check that the agent is approximately close to agent
    if (!early_break)
        return false;

    // Send back Vector3 pos vel and acc
    *pos = _pos.col(idx); *vel = _vel.col(idx); *acc = _acc.col(idx); 

    // std::cout << KYEL << "[trajectory_server.cpp] " << "Desired State Update Complete" << KNRM << std::endl;
    // std::cout << KYEL << "[trajectory_server.cpp] \n" << *pos << KNRM << std::endl;
    // std::cout << KYEL << "[trajectory_server.cpp] \n" << *vel << KNRM << std::endl;
    return true;
}

/** 
* @brief Get Desired Yaw
*/
// A requirement is that now must be an rounded to division of the knots
double trajectory::GetDesiredYaw(double now, double default_yaw)
{
    double current_yaw;
    bool early_break = false;
    int idx;
    // Check the time now and compare to knots
    for(int i = 0; i < _time.size(); i++)
    {
        if(abs(_time(i) - now) < 0.001)
        {
            idx = i;
            // std::cout << KYEL << "[trajectory_server.cpp] " << "idx " << idx << KNRM << std::endl;
            early_break = true;
            break;
        }
    }
    // if no early break, return update path as false
    // This will give the 2nd check that the agent is approximately close to agent
    if (!early_break || idx - 1 < 0)
        return default_yaw; 

    Vector3d _pos_diff = _pos.col(idx) - _pos.col(idx-1);
    double _norm = sqrt(pow(_pos_diff.x(),2) + pow(_pos_diff.y(),2));
    double _norm_x = _pos_diff.x() / _norm;
    double _norm_y = _pos_diff.y() / _norm;

    current_yaw = atan2(_norm_y,_norm_x);
    return current_yaw;
}

/** 
* @brief Check if trajectory is completed
*/
bool trajectory::isCompleted(double now)
{
    if (now - _fixed_knots(_fixed_knots.size()-1) <= 0)
        return false;
    else
        return true;
}

/** 
* @brief Return control point pose according to idx
*/
bool trajectory::returnControlPointPose(int idx, Vector3d *pos)
{
    if (idx >= 0 && idx < _fixed_cp.cols())
    {
        *pos = _fixed_cp.col(idx);
        return true;
    }
    else         
        return false;
}