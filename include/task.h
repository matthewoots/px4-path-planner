/*
 * task.h
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

// Mavlink  POSITION_TARGET_TYPEMASK
// 1	POSITION_TARGET_TYPEMASK_X_IGNORE	Ignore position x
// 2	POSITION_TARGET_TYPEMASK_Y_IGNORE	Ignore position y
// 4	POSITION_TARGET_TYPEMASK_Z_IGNORE	Ignore position z
// 8	POSITION_TARGET_TYPEMASK_VX_IGNORE	Ignore velocity x
// 16	POSITION_TARGET_TYPEMASK_VY_IGNORE	Ignore velocity y
// 32	POSITION_TARGET_TYPEMASK_VZ_IGNORE	Ignore velocity z
// 64	POSITION_TARGET_TYPEMASK_AX_IGNORE	Ignore acceleration x
// 128	POSITION_TARGET_TYPEMASK_AY_IGNORE	Ignore acceleration y
// 256	POSITION_TARGET_TYPEMASK_AZ_IGNORE	Ignore acceleration z
// 512	POSITION_TARGET_TYPEMASK_FORCE_SET	Use force instead of acceleration
// 1024	POSITION_TARGET_TYPEMASK_YAW_IGNORE	Ignore yaw
// 2048	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE	Ignore yaw rate

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandTOL.h> 

#include <std_msgs/Byte.h>

#include "px4_path_planner/Bspline.h"

#include <tf/tf.h>

#include <trajectory.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace Eigen;
using namespace std;

#define IDLE 0
#define TAKEOFF 1
#define HOVER 2
#define MISSION 3
#define HOME 4
#define LAND 5

enum VehicleTask
{
    kIdle,
    kTakeOff,
    kHover,
    kMission,
    kHome,
    kLand
};

enum MissionMode
{
    bypass,
    bspline,
    bspline_avoid,
    bspline_avoid_opt
};

class taskmaster
{
private:
    ros::NodeHandle _nh;

    ros::Subscriber state_sub;
    ros::Subscriber uav_pose_sub;
    ros::Subscriber uav_vel_sub;
    ros::Subscriber uav_cmd_sub;

    ros::Publisher local_pos_pub; // Only publishes position
    ros::Publisher local_pos_raw_pub; // Publish setpoint_local_raw, either P,V or A

    ros::Publisher bspline_pub;

    ros::ServiceClient arming_client; 
    ros::ServiceClient set_mode_client; 

    ros::Timer mission_timer;

    bool _initialised;
    bool _setpoint_raw_mode;
    bool takeoff_flag; // Whether the agent has taken off
    bool task_complete; // When task is complete

    double _send_desired_interval; // Trajectory interval, for bspline planning
    double _trajectory_pub_rate;
    double _takeoff_height;
    double _common_max_vel;
    double _common_min_vel;
    double roll, pitch, yaw;
    double last_request_timer;
    double last_mission_yaw;
    double home_yaw;
    
    int retry = 0;
    int uav_task = 0;
    int uav_prev_task;
    int _unique_id_range;
    int _order;
    int _control_points_division;

    bool _bypass = false;

    vector<int> uav_id;
    
    std::string _wp_file_location;
    std::string _id;

    mavros_msgs::CommandBool arm_cmd;

    Vector3d takeoff_pos;

    Vector3d current_pos;
    Vector3d global_pos;
    Vector3d global_offset;

    Vector3d current_vel;
    Vector3d last_mission_pos;

    mavros_msgs::State uav_current_state;

    // In ENU without global offset just local position
    geometry_msgs::PoseStamped uav_pose;

    // In NWU and with offset
    geometry_msgs::PoseStamped uav_global_pose; 
    
    geometry_msgs::PoseStamped home;
    geometry_msgs::TwistStamped uav_vel;

    mavros_msgs::PositionTarget bypass_sp;

    trajectory traj;

    vector<int> mission_mode_vector;
    vector<int> mission_mode;  
    vector<MatrixXd> mission_wp;
    int mission_type_count;
    ros::Time bypass_previous_message_time;

public:
    taskmaster(ros::NodeHandle &nodeHandle);
    ~taskmaster();

    bool set_offboard();

    void initialisation();

    void missionTimer(const ros::TimerEvent &);

    const std::string TaskToString(int v)
    {
        switch (v)
        {
            case kIdle:   return "IDLE";
            case kTakeOff:   return "TAKEOFF";
            case kHover: return "HOVER";
            case kMission:   return "MISSION";
            case kHome:   return "HOME";
            case kLand: return "LAND";
            default:      return "[Unknown Task]";
        }
    }

    const std::string MissionModeToString(int v)
    {
        switch (v)
        {
            case bypass:   return "BYPASS";
            case bspline:   return "BSPLINE";
            case bspline_avoid: return "BSPLINE_AVOID";
            case bspline_avoid_opt:   return "BSPLINE_AVOID_OPT";
            default:      return "[Unknown Mode]";
        }
    }

    void uavCommandCallBack(const std_msgs::Byte::ConstPtr &msg);
    
    /** @brief Get current uav FCU state */
    void uavStateCallBack(const mavros_msgs::State::ConstPtr &msg)
    {
        uav_current_state = *msg;
        // Initialise state at the beginning
        if (!_initialised)
            taskmaster::initialisation();
    }

    /** @brief Get current uav velocity */
    void uavVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        uav_vel = *msg;
        current_vel.x() = (double)uav_vel.twist.linear.x;
        current_vel.y() = (double)uav_vel.twist.linear.y;
        current_vel.z() = (double)uav_vel.twist.linear.z;
    }

    /** @brief Get current uav pose */
    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // Local in ENU frame
        uav_pose = *msg;
        current_pos.x() = (double)uav_pose.pose.position.x;
        current_pos.y() = (double)uav_pose.pose.position.y;
        current_pos.z() = (double)uav_pose.pose.position.z;

        tf::Quaternion q(
            uav_pose.pose.orientation.x, uav_pose.pose.orientation.y,
            uav_pose.pose.orientation.z, uav_pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        // Convert from ENU to NWU
        geometry_msgs::PoseStamped nwu_tmp = transform_pose_stamped(uav_pose, Vector3d(0,0,90.0));
        nwu_tmp.pose.position.x += global_offset.x();
        nwu_tmp.pose.position.x += global_offset.y();
        nwu_tmp.pose.position.x += global_offset.z();

        uav_global_pose = nwu_tmp;

        // Global in NWU frame
        global_pos.x() = (double)nwu_tmp.pose.position.x;
        global_pos.y() = (double)nwu_tmp.pose.position.y;
        global_pos.z() = (double)nwu_tmp.pose.position.z;
    }

    /** @brief Get bypass command message */
    void bypassCommandCallback(const  mavros_msgs::PositionTarget::ConstPtr &msg)
    {
        bypass_sp = *msg;
    }

    /** @brief Send Desired Command to PX4 via Mavros (Using Vector3d)*/
    void uavDesiredControlHandler(Vector3d command_pose, 
        Vector3d command_vel, 
        Vector3d command_acc, 
        double command_yaw)
    {
        // When in position control mode, send only waypoints
        if (!_setpoint_raw_mode)
        {
            geometry_msgs::PoseStamped pos_sp;
            pos_sp.pose.position.x = command_pose.x();
            pos_sp.pose.position.y = command_pose.y();
            pos_sp.pose.position.z = command_pose.z();
            local_pos_pub.publish(pos_sp);
        }
        // Send to setpoint_raw, which gives more freedom to what settings to control
        else
        {
            mavros_msgs::PositionTarget pos_sp;
            pos_sp.position.x = command_pose.x();
            pos_sp.position.y = command_pose.y();
            pos_sp.position.z = command_pose.z();

            pos_sp.velocity.x = command_vel.x();
            pos_sp.velocity.y = command_vel.y();
            pos_sp.velocity.z = command_vel.z();

            pos_sp.acceleration_or_force.x = command_acc.x();
            pos_sp.acceleration_or_force.y = command_acc.y();
            pos_sp.acceleration_or_force.z = command_acc.z();

            pos_sp.yaw = (float)command_yaw;
            // For the type mask we have to ignore the rest (3520)
            // 64	POSITION_TARGET_TYPEMASK_AX_IGNORE	Ignore acceleration x
            // 128	POSITION_TARGET_TYPEMASK_AY_IGNORE	Ignore acceleration y
            // 256	POSITION_TARGET_TYPEMASK_AZ_IGNORE	Ignore acceleration z
            // 1024	POSITION_TARGET_TYPEMASK_YAW_IGNORE	Ignore yaw
            // 2048	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE	Ignore yaw rate
            pos_sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            // pos_sp.type_mask = 3576; // Ignore Velocity, Acceleration and Yaw
            pos_sp.type_mask = 2552; // Ignore Velocity, Acceleration
            // pos_sp.type_mask = 3520; // Ignore Acceleration and Yaw
            // pos_sp.type_mask = 3072; // Ignore Yaw
            // pos_sp.type_mask = 2048;
            local_pos_raw_pub.publish(pos_sp);
        }
    }

    bool UnpackWaypoint(vector<int> *_mission_mode, MatrixXd *_waypoint, string _file_location)
    {
        printf("%s[task.h] Trying to open %s \n", KYEL, _file_location.c_str());
        ifstream file(_file_location);
        
        if (!file)
        {
            printf("%s[task.h] File not present! \n", KRED);
            return false;
        }
        printf("%s[task.h] Success, found %s \n", KGRN, _file_location.c_str());

        io::CSVReader<4> in(_file_location);
        in.read_header(io::ignore_extra_column, "mode", "xpos", "ypos", "zpos");
        int mission_mode; double xpos; double ypos; double zpos;
        std::vector<int> mm; std::vector<double> x; std::vector<double> y; std::vector<double> z;
        int total_row = 0;
        // First pass is to get number of rows
        while (in.read_row(mission_mode, xpos, ypos, zpos)){
            total_row++;
            mm.push_back (mission_mode); 
            x.push_back (xpos); y.push_back (ypos); z.push_back (zpos);
            std::cout << KBLU << "[task.cpp] " << "[Unpack Waypoint] " << KNRM << " " << mission_mode << " " << xpos << " " << ypos << " " << zpos << std::endl;
        }
        MatrixXd waypoint = MatrixXd::Zero(3, total_row);
        *_waypoint = MatrixXd::Zero(3, total_row);
        for (int i = 0; i < total_row; i++)
        {
            waypoint(0,i) = x[i]; 
            waypoint(1,i) = y[i]; 
            waypoint(2,i) = z[i];
        }
        *_waypoint = waypoint;
        *_mission_mode = mm;

        return true;
    }

    void TrajectoryGeneration(Vector3d start_pose, MatrixXd wp,
        int _default, int _succeed)
    {
        double selected_speed;
        if (_succeed == kLand || _succeed == kTakeOff)
            selected_speed = taskmaster::_common_min_vel;
        else
            selected_speed = taskmaster::_common_max_vel;

        traj.Initialise(taskmaster::_order, 
            taskmaster::_control_points_division, 
            taskmaster::_trajectory_pub_rate);
        traj.SetClampedPath(wp, 
            selected_speed, 
            start_pose); 
        if (!traj.UpdateFullPath())
        {
            printf("%s[task.h] TrajectoryGeneration fail to set path \n", KRED);
            taskmaster::uav_task = _default;
            return;
        }

        taskmaster::uav_task = _succeed;
        return;
    }

    // Used for everything except mission
    void PublishDesiredControl(Vector3d final_pose, int _finished_task)
    {
        double last_timer = taskmaster::last_request_timer;
        double interval = taskmaster::_send_desired_interval;
        if (!taskmaster::task_complete && (ros::Time::now().toSec() - last_timer > interval))
        {
            // std::cout << KYEL << "[main.cpp] Trajectory_Time_Delay " << KNRM << (ros::Time::now().toSec() - last_timer) - (interval) << std::endl;
            
            double fact = floor((ros::Time::now().toSec() - traj.GetStartTime()) / interval);

            taskmaster::last_request_timer = traj.GetStartTime() + interval * fact;
            
            if (traj.isCompleted(ros::Time::now().toSec()))
            {
                printf("%s[task.h] %s Complete \n", KGRN, TaskToString(taskmaster::uav_task).c_str());
                taskmaster::task_complete = true;
                taskmaster::last_mission_pos = final_pose;
                taskmaster::last_mission_yaw = taskmaster::yaw;
                taskmaster::uav_task = _finished_task;
                return;
            }
        }
        else 
        {
            // Will not do anything since it is not time to publish the trajectory
            return;
        }

        Vector3d pos; Vector3d vel; Vector3d acc; 
        // double _calculated_yaw = taskmaster::home_yaw;
        double _calculated_yaw = yaw;
        if (!traj.GetDesiredState(taskmaster::last_request_timer, &pos, &vel, &acc))
        {
            printf("%s[task.h] %s Desired State Error \n", KRED, TaskToString(taskmaster::uav_task).c_str());
            return;
        }

        taskmaster::PublishBspline();
        taskmaster::uavDesiredControlHandler(pos, vel, acc, _calculated_yaw);
        return;
    }

    void PublishBspline()
    {
        // printf("%s[task.h] Publishing Bspline \n", KGRN);
        px4_path_planner::Bspline bspline;
        bspline.order = traj.GetOrder();
        bspline.knot_division = traj.GetKnotDivision();

        MatrixXd gcp = MatrixXd (traj.GetGlobalControlPoints());
        VectorXd knots = VectorXd (traj.GetKnots());

        // printf("%s[task.h] Publishing gcp \n", KGRN);
        bspline.global_control_points.reserve(gcp.cols());

        for (int i = 0; i < gcp.cols(); i++)
        {
            geometry_msgs::Point pt;
            pt.x = gcp(0,i);
            pt.y = gcp(1,i);
            pt.z = gcp(2,i);
            // std::cout << KGRN << "[main.cpp] CP \n" << KNRM << pt << std::endl;
            bspline.global_control_points.push_back(pt);
        }
        // std::cout << KGRN << "[main.cpp] cp_size \n" << KNRM << bspline.global_control_points.size() << std::endl;

        // printf("%s[main.cpp] Publishing knots \n", KGRN);
        bspline.knot.reserve(knots.rows());
        for (int j = 0; j < knots.size(); j++)
        {
            bspline.knot[j] = (float)knots[j];
        }

        bspline_pub.publish(bspline);
        return;
    }

    /* 
    * @brief Transform PoseStamped according to the translation and rpy given
    */
    geometry_msgs::PoseStamped transform_pose_stamped(geometry_msgs::PoseStamped _p, Vector3d _rpy)
    {
        geometry_msgs::PoseStamped poseStamped;

        geometry_msgs::TransformStamped transform;
        geometry_msgs::Quaternion q; geometry_msgs::Vector3 t;
        tf2::Quaternion quat_tf;

        t.x = 0; t.y = 0; t.z = 0; 

        double deg2rad = 1.0 / 180.0 * 3.1415926535;

        quat_tf.setRPY(_rpy.x() * deg2rad, 
            _rpy.y() * deg2rad, 
            _rpy.z() * deg2rad); // Create this quaternion from roll/pitch/yaw (in radians)
        q = tf2::toMsg(quat_tf);

        transform.transform.translation = t;
        transform.transform.rotation = q;
        transform.child_frame_id = "/base";
        transform.header.frame_id = "/map";

        tf2::doTransform(_p, poseStamped, transform);

        return poseStamped;
    }

};
