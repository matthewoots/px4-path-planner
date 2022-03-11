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
#ifndef TASK_H
#define TASK_H

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
#include <std_msgs/Float32MultiArray.h>

#include "px4_path_planner/Bspline.h"
#include "px4_path_planner/agent.h"

#include <tf/tf.h>

#include <trajectory.h>
#include <rrt.h>
#include <bspline_optimization.h>

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

    ros::Subscriber state_sub, uav_pose_sub, uav_vel_sub, uav_cmd_sub;
    ros::Subscriber mission_msg_sub, bypass_msg_sub, pcl2_msg_sub, no_fly_zone_sub;

    ros::Publisher local_pos_pub; // Only publishes position
    ros::Publisher local_pos_raw_pub; // Publish setpoint_local_raw, either P,V or A

    ros::Publisher bspline_pub, global_pos_pub, opt_bspline_pub;

    ros::ServiceClient arming_client; 
    ros::ServiceClient set_mode_client; 

    ros::Timer mission_timer, opt_timer, hover_timer;

    double _weight_smooth, _weight_feas, _weight_term; 
    double _weight_static, _weight_reci;
    double _max_acc;

    bool _initialised;
    bool _setpoint_raw_mode;
    bool takeoff_flag = false; // Whether the agent has taken off
    bool task_complete; // When task is complete

    double _send_desired_interval; // Trajectory interval, for bspline planning
    double _send_optimization_interval;
    double _optimization_rate;
    double _trajectory_pub_rate;
    double _takeoff_height;
    double _common_max_vel;
    double _common_min_vel;
    double roll, pitch, yaw;
    double roll_nwu, pitch_nwu, yaw_nwu;
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
    bool _unpack_from_local_file;
    bool loaded_pcl = false;

    bool formation_mission_mode = false;
    vector<VectorXd> no_fly_zone;

    bool bypass_initialize = false;
    double bypass_init_time = -1.0; // Not initialized condition for time

    vector<int> uav_id;
    
    std::string _wp_file_location;
    std::string _id;
    std::string _package_directory;
    std::string _params_directory;

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

    sensor_msgs::PointCloud2 pcl_pc2;
    int pcl_count = 0;

    vector<int> mission_mode_vector;
    vector<int> mission_mode;  
    vector<MatrixXd> mission_wp;
    int mission_type_count;

    ros::Time bypass_previous_message_time;
    ros::Time mission_previous_message_time;

    struct bspline_assembly 
    {
        int agent_id;
        vector<double> knot_vector;
        vector<Vector3d> control_points;
    };

    vector<bspline_assembly> bspline_vector;

public:
    taskmaster(ros::NodeHandle &nodeHandle);
    ~taskmaster();

    bool set_offboard();

    void initialisation();

    void optimization(const ros::TimerEvent &);
    void hoverTimer(const ros::TimerEvent &);
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
    
    /** @brief Get current uav FCU state */
    void uavStateCallBack(const mavros_msgs::State::ConstPtr &msg)
    {
        uav_current_state = *msg;
        // Initialise state at the beginning
        if (!_initialised)
            taskmaster::initialisation();
    }

    // We should reset the agent data at everytime we call it
    /** @brief Get Agent info with bspline callback */
    void agentInfoBsplineCallBack(const px4_path_planner::agent::ConstPtr &msg)
    {
        px4_path_planner::agent agent_bspline = *msg;
        px4_path_planner::Bspline bspline_msg = agent_bspline.bs;
        // int32 order
        // int32 knot_division
        // float64[] knot
        // geometry_msgs/Point[] global_control_points
        bspline_assembly tmp_bs;
        tmp_bs.agent_id = agent_bspline.id;
        
        for (int i = 0; i < bspline_msg.knot.size(); i++)
            tmp_bs.knot_vector.push_back(bspline_msg.knot[i]);

        for (int i = 0; i < bspline_msg.global_control_points.size(); i++)
        {
            Vector3d cp_tmp;
            cp_tmp.x() = bspline_msg.global_control_points[i].x;
            cp_tmp.y() = bspline_msg.global_control_points[i].y;
            cp_tmp.z() = bspline_msg.global_control_points[i].z;
            tmp_bs.control_points.push_back(cp_tmp);
        }

        bspline_vector.push_back(tmp_bs);
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

        tf::Quaternion q_enu(
            uav_pose.pose.orientation.x, uav_pose.pose.orientation.y,
            uav_pose.pose.orientation.z, uav_pose.pose.orientation.w);
        tf::Matrix3x3 m_enu(q_enu);
        m_enu.getRPY(roll, pitch, yaw);
    //     std::cout << KYEL << "[task.h] enu_yaw=" << KNRM <<
    // yaw << std::endl;

        // Convert from ENU to global NWU
        uav_global_pose = convert_enu_to_global_nwu(uav_pose);

        tf::Quaternion q_nwu(
            uav_global_pose.pose.orientation.x, uav_global_pose.pose.orientation.y,
            uav_global_pose.pose.orientation.z, uav_global_pose.pose.orientation.w);
        tf::Matrix3x3 m_nwu(q_nwu);
        m_nwu.getRPY(roll_nwu, pitch_nwu, yaw_nwu);
    //     std::cout << KGRN << "[task.h] nwu_yaw=" << KNRM <<
    // yaw_nwu << std::endl;

        // Global in NWU frame in Vector3d
        global_pos.x() = (double)uav_global_pose.pose.position.x;
        global_pos.y() = (double)uav_global_pose.pose.position.y;
        global_pos.z() = (double)uav_global_pose.pose.position.z;

        global_pos_pub.publish(uav_global_pose);
    }

    /** @brief Get bypass command message */
    void bypassCommandCallback(const  mavros_msgs::PositionTarget::ConstPtr &msg)
    {
        bypass_sp = *msg;
        bypass_previous_message_time = bypass_sp.header.stamp;
    }

    void uavCommandCallBack(int msg);

    /** @brief Handles no fly zone float64 array 1D x_min, x_max, y_min, y_max */
    void noFlyZoneMsgCallBack(const  std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        std_msgs::Float32MultiArray multi_array = *msg;
        int size = multi_array.data.size();
        int idx = size / 4;
        
        for (int i = 0; i < idx; i++)
        {
            VectorXd no_fly_zone_single = VectorXd::Zero(4);
            no_fly_zone_single[0] = multi_array.data[4*i+0];
            no_fly_zone_single[1] = multi_array.data[4*i+1];
            no_fly_zone_single[2] = multi_array.data[4*i+2];
            no_fly_zone_single[3] = multi_array.data[4*i+3];
            printf("%s[task.h] %d no_fly_zone_single %lf %lf %lf %lf! \n", KGRN, i, no_fly_zone_single[0],
                no_fly_zone_single[1], no_fly_zone_single[2], no_fly_zone_single[3]);
            no_fly_zone.push_back(no_fly_zone_single);
        }
    }

    void pcl2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        // Once load a few times we do not need to do a callback
        if (pcl_count > 2)
        {
            // loaded_pcl = true;
            return;
        }

        pcl_pc2 = *msg;
        pcl_count++;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr original_pcl_pc = 
        pcl2_converter(pcl_pc2);

        size_t num_points = original_pcl_pc->size();
        int total = static_cast<int>(num_points);
        printf("%s[task.h] Actual obstacle size %d! \n", KGRN, total);
        printf("%s[task.h] INSTANCE OF PCL IS LOADED!\n", KCYN);
    }

    /** @brief Handles mission from float64 array 1D (1) Mode (2-4) Waypoint */
    void uavMissionMsgCallBack(const  std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        mission_previous_message_time = ros::Time::now();
        std_msgs::Float32MultiArray multi_array = *msg;
        
        // Error when mission mode is sent while in another mission mode this is disastrous

        // We just hardcode it to 0 so it will only take 1 message
        int command_callback_type = (int)multi_array.data[0];
        
        if (command_callback_type == MISSION)
        {
            mission_wp.clear();
            mission_mode_vector.clear();
            mission_mode.clear();

            MatrixXd wp;
            
            int size = multi_array.data.size();
            int idx = size / 9;
            wp = MatrixXd::Zero(3, idx);
            for (int i = 0; i < idx; i++)
            {
                mission_mode_vector.push_back((int)multi_array.data[9*i+1]);

                wp(0,i) = multi_array.data[9*i+2];
                wp(1,i) = multi_array.data[9*i+3];
                wp(2,i) = multi_array.data[9*i+4];
            }
            std::cout << KBLU << "[task.h] " << "[Mission Waypoint] " << std::endl << KNRM << wp << std::endl;
            printf("%s[task.h] Total Mission size %lu with mode size %lu waypoints! \n", KBLU, 
                wp.size(), mission_mode_vector.size());
            
            // Mission changes counts how many waypoints belong to the mission mode
            // Counts in series/sequentially 
            vector<int> mission_changes; 
            int mission_mode_count = 0;
            // We check to see whether is there any changes in mission mode throughout our mission
            for (int j = 0; j < mission_mode_vector.size(); j++)
            {
                
                mission_mode.push_back(mission_mode_vector[j]); 
                mission_changes.push_back(1);
                

                // Use this for checking whether there are mixed missions in the vector

                // mission_mode_count++;
                // // Put the rest of the points at the last point
                // if (j == mission_mode_vector.size() - 1)
                // {
                //     mission_mode.push_back(mission_mode_vector[j]); 
                //     mission_changes.push_back(mission_mode_count);
                //     printf("%s[task.h] Mission %d with %d waypoints woth type %d! \n", KBLU, 
                //         mission_changes.size(), mission_mode_count, mission_mode_vector[j]);
                    
                //     continue;
                // }

                // // Mission Mode at j check to see whether is it the same as the previous
                // // if not we segment it
                // if (mission_mode_vector[j] != mission_mode_vector[j+1])
                // {
                //     mission_mode.push_back(mission_mode_vector[j]); 
                //     mission_changes.push_back(mission_mode_count);
                //     printf("%s[task.h] Mission %d with %d waypoints woth type %d! \n", KBLU, 
                //         mission_changes.size(), mission_mode_count, mission_mode_vector[j]);
                //     mission_mode_count = 0;
                //     continue;
                // }
                
            }            

            // Adder is just to push back the points that we start from when we do the Matrix block process
            // To fill in the mission wp
            int adder = 0;
            // Segment mission_wp into wp
            for (int j = 0; j < mission_changes.size(); j++)
            {
                mission_wp.push_back(wp.block(0,adder,3,mission_changes[j])); 
                adder += mission_changes[j];
            }
            printf("%s[task.h] Segment wp to wp vector, mission size %lu! \n", KBLU, mission_wp.size());

        }

        std::string leader_index = "S" + to_string((int)multi_array.data[5]);
        
        // If both are true then we know we are using formation parameters since we're the leader
        if ((int)multi_array.data[5] < 30 && !leader_index.compare(_id))
            formation_mission_mode = true;
        else 
            formation_mission_mode = false;

        mission_type_count = 0;

        // We need to take this out of the callback
        uavCommandCallBack(command_callback_type);


    }

    /** @brief Send Desired Command to PX4 via Mavros (Using Vector3d)*/
    void uavDesiredControlHandler(Vector3d command_pose, 
        Vector3d command_vel, 
        Vector3d command_acc, 
        double command_yaw)
    {
        // Do conversion from NWU_global to ENU using convert_global_nwu_to_enu
        // command_pose need to change to ENU with global conversion
        // command_vel and command_acc do not need
        // Calculated yaw need to change too
        geometry_msgs::PoseStamped pos_nwu_sp_tmp;
        pos_nwu_sp_tmp.pose.position.x = command_pose.x();
        pos_nwu_sp_tmp.pose.position.y = command_pose.y();
        pos_nwu_sp_tmp.pose.position.z = command_pose.z();

        geometry_msgs::PoseStamped vel_nwu_sp_tmp;
        vel_nwu_sp_tmp.pose.position.x = command_vel.x();
        vel_nwu_sp_tmp.pose.position.y = command_vel.y();
        vel_nwu_sp_tmp.pose.position.z = command_vel.z();

        // Convert from ENU to Global NWU
        geometry_msgs::PoseStamped vel_enu_sp = transform_pose_stamped(vel_nwu_sp_tmp, Vector3d(0,0,90.0));

        geometry_msgs::PoseStamped pos_enu_sp_tmp = convert_global_nwu_to_enu(pos_nwu_sp_tmp);

        command_yaw += 90.0/180.0 * 3.1415;
    //     std::cout << KBLU << "[task.h] enu_cmd_yaw=" << KNRM <<
    // command_yaw << std::endl;

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

            pos_sp.position = pos_enu_sp_tmp.pose.position;

            pos_sp.velocity.x = vel_enu_sp.pose.position.x;
            pos_sp.velocity.y = vel_enu_sp.pose.position.y;
            pos_sp.velocity.z = vel_enu_sp.pose.position.z;

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
            // pos_sp.type_mask = 2552; // Ignore Velocity, Acceleration
            pos_sp.type_mask = 2496; // Ignore Acceleration
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
        int _default, int _succeed, int _mode)
    {
        double selected_speed;
        if (_succeed == kLand || _succeed == kTakeOff)
            selected_speed = taskmaster::_common_min_vel;
        else
            selected_speed = taskmaster::_common_max_vel;

        traj.Initialise(taskmaster::_order, 
            taskmaster::_control_points_division, 
            taskmaster::_trajectory_pub_rate);

        MatrixXd key_wp;
        // Add RRT here before we set path
        if (_mode == bspline_avoid || _mode == bspline_avoid_opt)
        {
            rrt_sample rrt_node; std::string rrt_params;
            if (formation_mission_mode)
                rrt_params = _params_directory + "/formation.csv";
            else
                rrt_params = _params_directory + "/solo.csv";
            if (!rrt_node.unpack_rrt_params(rrt_params))
            {
                printf("%s[task.h] TrajectoryGeneration fail to set path \n", KRED);
                taskmaster::uav_task = _default;
                return;
            }
            for (int i = 0; i < no_fly_zone.size(); i++)
            {
                VectorXd no_fly_zone_single = VectorXd::Zero(4);
                no_fly_zone_single = no_fly_zone[i];
                printf("%s[task.h] %d no_fly_zone_single %lf %lf %lf %lf! \n", KGRN, i, no_fly_zone_single[0],
                    no_fly_zone_single[1], no_fly_zone_single[2], no_fly_zone_single[3]);
            }

            // We assume we are receiving only one waypoint
            if (rrt_node.RRT(pcl_pc2, start_pose, wp.col(0), 
                no_fly_zone))
            {
                
                rrt_node.rrt_bspline(3);
                key_wp.resize(3,rrt_node.bspline.size());

                for (int i = 0; i < rrt_node.bspline.size(); i++)
                {
                    key_wp(0,i) = rrt_node.bspline[i].x(); 
                    key_wp(1,i) = rrt_node.bspline[i].y(); 
                    key_wp(2,i) = rrt_node.bspline[i].z();
                }
                printf("%s[task.h] RRT Succeeded\n", KGRN);
            }
            else
            {
                printf("%s[task.h] TrajectoryGeneration fail to set path \n", KRED);
                taskmaster::uav_task = _default;
                return;
            }
        }
        else 
        {
            key_wp = wp;
        }

        traj.SetClampedPath(key_wp, 
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
                // *** This uses ENU, obsolete since we use NWU ***
                // taskmaster::last_mission_yaw = taskmaster::yaw;

                taskmaster::last_mission_yaw = taskmaster::yaw_nwu;
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

        // *** This uses ENU, obsolete since we use NWU ***
        // double _calculated_yaw = yaw;
        double _calculated_yaw = yaw_nwu;

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

    /* 
    * @brief Transform PoseStamped from ENU to Global NWU
    */
    geometry_msgs::PoseStamped convert_enu_to_global_nwu(geometry_msgs::PoseStamped enu_pose)
    {
        // Convert from ENU to Global NWU
        geometry_msgs::PoseStamped nwu_tmp = transform_pose_stamped(enu_pose, Vector3d(0,0,-90.0));
        nwu_tmp.header.stamp.sec = enu_pose.header.stamp.sec; 
        nwu_tmp.header.stamp.nsec = enu_pose.header.stamp.nsec; 
        nwu_tmp.pose.position.x += global_offset.x();
        nwu_tmp.pose.position.y += global_offset.y();
        nwu_tmp.pose.position.z += global_offset.z();

        return nwu_tmp;
    }

    /* 
    * @brief Transform PoseStamped from Global NWU to ENU 
    */
    geometry_msgs::PoseStamped convert_global_nwu_to_enu(geometry_msgs::PoseStamped global_nwu_pose)
    {
        global_nwu_pose.pose.position.x -= global_offset.x();
        global_nwu_pose.pose.position.y -= global_offset.y();
        global_nwu_pose.pose.position.z -= global_offset.z();
        // Convert from ENU to Global NWU
        geometry_msgs::PoseStamped enu_tmp = transform_pose_stamped(global_nwu_pose, Vector3d(0,0,90.0));
        enu_tmp.header.stamp.sec = global_nwu_pose.header.stamp.sec; 
        enu_tmp.header.stamp.nsec = global_nwu_pose.header.stamp.nsec; 
    
        return enu_tmp;
    }

};

#endif
