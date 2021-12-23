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

#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <std_msgs/Byte.h>

#include <tf/tf.h>

#include <trajectory.h>

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

class taskmaster
{
private:
    ros::NodeHandle _nh;

    ros::Subscriber state_sub;
    ros::Subscriber uav_gps_cur_sub;
    ros::Subscriber uav_gps_home_sub;
    ros::Subscriber uav_pose_sub;
    ros::Subscriber uav_vel_sub;
    ros::Subscriber uav_cmd_sub;

    ros::Publisher local_pos_pub; 
    ros::Publisher local_pos_raw_pub;

    ros::ServiceClient arming_client; 
    ros::ServiceClient set_mode_client; 

    ros::Timer mission_timer;

    bool _initialised;
    bool _setpoint_raw_mode;
    bool takeoff_flag;
    bool task_complete;

    double _send_desired_interval;
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

    vector<int> uav_id;
    
    std::string _wp_file_location;
    std::string _id;

    mavros_msgs::CommandBool arm_cmd;

    Vector3d takeoff_pos;
    Vector3d current_pos;
    Vector3d current_vel;
    Vector3d last_mission_pos;

    mavros_msgs::HomePosition uav_gps_home;
    mavros_msgs::State uav_current_state;
    sensor_msgs::NavSatFix uav_gps_cur;
    geometry_msgs::PoseStamped uav_pose;
    geometry_msgs::PoseStamped home;
    geometry_msgs::TwistStamped uav_vel;

    trajectory traj;

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

    void uavCommandCallBack(const std_msgs::Byte::ConstPtr &msg);

    void gpsHomeCallback(const mavros_msgs::HomePosition::ConstPtr &msg)
    {
        uav_gps_home = *msg;
    }
    void gpsCurrentCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        uav_gps_cur = *msg;
    }
    
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
        uav_pose = *msg;
        current_pos.x() = (double)uav_pose.pose.position.x;
        current_pos.y() = (double)uav_pose.pose.position.y;
        current_pos.z() = (double)uav_pose.pose.position.z;

        tf::Quaternion q(
            uav_pose.pose.orientation.x, uav_pose.pose.orientation.y,
            uav_pose.pose.orientation.z, uav_pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
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
            // pos_sp.type_mask = 3520; // Ignore Acceleration and Yaw
            // pos_sp.type_mask = 3072; // Ignore Yaw
            pos_sp.type_mask = 2048;
            local_pos_raw_pub.publish(pos_sp);
        }
    }

    bool UnpackWaypoint(MatrixXd *_waypoint, string _file_location)
    {
        printf("%s[main.h] Trying to open %s \n", KYEL, _file_location.c_str());
        ifstream file(_file_location);
        
        if (!file)
        {
            printf("%s[main.h] File not present! \n", KRED);
            return false;
        }
        printf("%s[main.h] Success, found %s \n", KGRN, _file_location.c_str());

        io::CSVReader<3> in(_file_location);
        in.read_header(io::ignore_extra_column, "xpos", "ypos", "zpos");
        double xpos; double ypos; double zpos;
        std::vector<double> x; std::vector<double> y; std::vector<double> z;
        int total_row = 0;
        // First pass is to get number of rows
        while (in.read_row(xpos, ypos, zpos)){
            total_row++;
            x.push_back (xpos); y.push_back (ypos); z.push_back (zpos);
            std::cout << KBLU << "[main.cpp] " << "[Unpack Waypoint] " << KNRM << xpos << " " << ypos << " " << zpos << std::endl;
        }
        MatrixXd waypoint = MatrixXd::Zero(3, total_row);
        *_waypoint = MatrixXd::Zero(3, total_row);
        for (int i = 0; i < x.size(); i++)
        {
            waypoint(0,i) = x[i]; 
            waypoint(1,i) = y[i]; 
            waypoint(2,i) = z[i];
        }
        *_waypoint = waypoint;

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
            printf("%s[main.cpp] TrajectoryGeneration fail to set path \n", KRED);
            taskmaster::uav_task = _default;
            return;
        }

        taskmaster::uav_task = _succeed;
        return;
    }

    void PublishDesiredControl(Vector3d final_pose, int _finished_task)
    {
        double last_timer = taskmaster::last_request_timer;
        double interval = taskmaster::_send_desired_interval;
        if (!taskmaster::task_complete && (ros::Time::now().toSec() - last_timer > interval))
        {
            std::cout << KYEL << "[main.cpp] Trajectory_Time_Delay " << KNRM << (ros::Time::now().toSec() - last_timer) - (interval) << std::endl;
            
            double fact = floor((ros::Time::now().toSec() - traj.GetStartTime()) / interval);

            taskmaster::last_request_timer = traj.GetStartTime() + interval * fact;
            
            if (traj.isCompleted(ros::Time::now().toSec()))
            {
                printf("%s[main.cpp] %s Complete \n", KGRN, TaskToString(taskmaster::uav_task).c_str());
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

        Vector3d pos; Vector3d vel; Vector3d acc; double _calculated_yaw = taskmaster::home_yaw;
        if (!traj.GetDesiredState(taskmaster::last_request_timer, &pos, &vel, &acc))
        {
            printf("%s[main.cpp] %s Desired State Error \n", KRED, TaskToString(taskmaster::uav_task).c_str());
            return;
        }
        
        taskmaster::uavDesiredControlHandler(pos, vel, acc, _calculated_yaw);
        return;
    }

};
