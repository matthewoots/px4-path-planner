/*
 * main.cpp
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

#include "task.h"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_path_planner");
    ros::NodeHandle nh("~");
    // The setpoint publishing rate MUST be faster than 2Hz
    taskmaster taskmaster(nh);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}

taskmaster::taskmaster(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
{
    _nh.param<int>("spline_order", _order, 3);
    _nh.param<int>("unique_id_range", _unique_id_range, 10);
    _nh.param<int>("control_points_division", _control_points_division, 1);
    _nh.param<bool>("setpoint_raw_mode", _setpoint_raw_mode, true);
    _nh.param<double>("trajectory_pub_rate", _trajectory_pub_rate, 1.0);_nh.param<double>("common_min_vel", _common_min_vel, 0.1);
    _nh.param<double>("takeoff_height", _takeoff_height, 1.0);
    _nh.param<double>("common_max_vel", _common_max_vel, 1.0);
    _nh.param<std::string>("wp_file_location", _wp_file_location, "~/wp.csv");
    _nh.param<std::string>("agent_id", _id, "S1");
    _nh.param<std::string>("package_directory", _package_directory, "~/");

    _params_directory = _package_directory + "/params";

    // Global offset from the origin in NWU frame
    _nh.param<double>("nwu_offset_x", global_offset.x(), 0.0);
    _nh.param<double>("nwu_offset_y", global_offset.y(), 0.0);
    _nh.param<double>("nwu_offset_z", global_offset.z(), 0.0);

    // Optimization parameters
    _nh.param<double>("optimization_rate", _optimization_rate, 2.0);
    _nh.param<double>("weight_smooth", _weight_smooth, 0.5);
    _nh.param<double>("weight_feas", _weight_feas, 0.5);
    _nh.param<double>("weight_term", _weight_term, 0.5);
    _nh.param<double>("weight_static", _weight_static, 0.5);
    _nh.param<double>("weight_reci", _weight_reci, 0.5);
    _nh.param<double>("max_acc", _max_acc, 0.5);

    _nh.param<bool>("unpack_from_local_file", _unpack_from_local_file, true);

    _send_desired_interval = 1 / _trajectory_pub_rate;
    _send_optimization_interval = 1 / _optimization_rate;

    std::string copy_id = _id; 
    string uav_id_char = copy_id.erase(0,1); // removes first character
    uav_id = stoi(uav_id_char);

    printf("%s------------- Planner Parameter ------------- \n", KYEL);
    printf("%s  _unpack_from_local_file =%s %s \n", KBLU, KNRM, _unpack_from_local_file ? "true" : "false");
    printf("%s  _setpoint_raw_mode =%s %s \n", KBLU, KNRM, _setpoint_raw_mode ? "true" : "false");
    printf("%s  _spline_order =%s %d \n", KBLU, KNRM, _order);
    printf("%s  _unique_id_range =%s %d \n", KBLU, KNRM, _unique_id_range);
    printf("%s  _send_desired_interval =%s %lf \n", KBLU, KNRM, _send_desired_interval);
    printf("%s  _control_points_division =%s %d \n", KBLU, KNRM, _control_points_division);
    printf("%s  _takeoff_height =%s %lf \n", KBLU, KNRM, _takeoff_height);
    printf("%s  _common_max_vel =%s %lf \n", KBLU, KNRM, _common_max_vel);
    printf("%s  _common_min_vel =%s %lf \n", KBLU, KNRM, _common_min_vel);
    printf("%s  _wp_file_location =%s %s \n", KBLU, KNRM, _wp_file_location.c_str());
    printf("%s  _package_directory =%s %s \n", KBLU, KNRM, _package_directory.c_str());
    printf("%s  _id =%s %s \n", KBLU, KNRM, _id.c_str());
    printf("%s  uav_id =%s %d \n", KBLU, KNRM, uav_id);
    printf("%s  global_offset =%s [%lf %lf %lf] \n\n", KBLU, KNRM, 
        global_offset.x(), global_offset.y(), global_offset.z());

    printf("\n%s  Using System Time =%s %s\n", KBLU, KNRM, 
       ros::Time::useSystemTime() ? "true" : "false");
    printf("%s------------------------------------- \n", KYEL);

    printf("%s------------- Optimization Parameter ------------- \n", KYEL);
    printf("%s  _send_optimization_interval =%s %lf \n", KBLU, KNRM, _send_optimization_interval);
    printf("%s  _weight_smooth =%s %lf \n", KBLU, KNRM, _weight_smooth);
    printf("%s  _weight_feas =%s %lf \n", KBLU, KNRM, _weight_feas);
    printf("%s  _weight_term =%s %lf \n", KBLU, KNRM, _weight_term);
    printf("%s  _weight_static =%s %lf \n", KBLU, KNRM, _weight_static);
    printf("%s  _weight_reci =%s %lf \n", KBLU, KNRM, _weight_reci);
    
    printf("%s------------------------------------- \n", KYEL);


    /* ------------ Subscribers ------------ */
    /** 
    * @brief Get Mavros State of PX4
    */
    state_sub = _nh.subscribe<mavros_msgs::State>(
        "/" + _id + "/mavros/state", 10, boost::bind(&taskmaster::uavStateCallBack, this, _1));
    /** 
    * @brief Get current agent pose
    */
    uav_pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
        "/" + _id + "/mavros/local_position/pose", 1, &taskmaster::uavPoseCallback, this);
    /** 
    * @brief Get current agent velocity
    */
    uav_vel_sub = _nh.subscribe<geometry_msgs::TwistStamped>(
        "/" + _id + "/mavros/local_position/velocity_local", 1, &taskmaster::uavVelCallback, this);
    /** 
    * @brief Get all uav agent info such as bspline with this callback
    */
     multi_uav_sub = _nh.subscribe<px4_path_planner::multi_agent>(
         "/multi_agent_trajectory", 50, &taskmaster::multiAgentInfoBsplineCallBack, this);
    
    /** 
    * @brief Handles bypass message from mavros_msgs::PositionTarget type
    */
    bypass_msg_sub = _nh.subscribe<mavros_msgs::PositionTarget>(
        "/" + _id + "/bypass", 1, &taskmaster::bypassCommandCallback, this);

    
    /* ------------ Subscribe from missionmanager ------------ */
    /** 
    * @brief Handles mission from float64 array 1D (1) Mode (2-4) Waypoint
    */
    mission_msg_sub = _nh.subscribe<std_msgs::Float32MultiArray>(
        "/" + _id + "/mission", 1, &taskmaster::uavMissionMsgCallBack, this);
    /** 
    * @brief Handles no fly zone float64 array 1D x_min, x_max, y_min, y_max
    */
    no_fly_zone_sub = _nh.subscribe<std_msgs::Float32MultiArray>(
        "/no_fly_zones", 1, &taskmaster::noFlyZoneMsgCallBack, this);
    /** 
    * @brief Handles mission from float64 array 1D (1) Mode (2-4) Waypoint
    */
    mission_msg_sub = _nh.subscribe<std_msgs::Float32MultiArray>(
        "/" + _id + "/mission", 1, &taskmaster::uavMissionMsgCallBack, this);
    

    /* ------------ Subscribe from swarm param ------------ */
    /** 
    * @brief Handles formation parameters from float64 array
    */
    formation_msg_sub = _nh.subscribe<std_msgs::Float32MultiArray>(
        "/param/formation_settings", 1, &taskmaster::formationParamMsgCallBack, this);
    /** 
    * @brief Handles solo parameters from float64 array
    */
    solo_msg_sub = _nh.subscribe<std_msgs::Float32MultiArray>(
        "/param/solo_settings", 1, &taskmaster::soloParamMsgCallBack, this);
    pcl2_msg_sub = _nh.subscribe<sensor_msgs::PointCloud2>(
        "/param/pcl", 1,  boost::bind(&taskmaster::pcl2Callback, this, _1));

    /* ------------ Subscribe from re-localization ------------ */
    /** 
    * @brief Get from Relocalization module, the corrected pose
    */
    relocalization_gp_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
        "/" + _id + "/relocalization/global_pose", 1, &taskmaster::rlGlobalPoseCallback, this);



    /* ------------ Publishers ------------ */
    /** 
    * @brief Publisher that publishes control position setpoints to Mavros
    */
    local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>(
        "/" + _id + "/mavros/setpoint_position/local", 10);
    /** 
    * @brief Publisher that publishes control raw setpoints to Mavros
    */
    local_pos_raw_pub = _nh.advertise<mavros_msgs::PositionTarget>(
        "/" + _id + "/mavros/setpoint_raw/local", 10);
    /** 
    * @brief Publisher that publishes Bspline
    */
    bspline_pub = _nh.advertise<px4_path_planner::Bspline>(
        "/" + _id + "/path/bspline", 10);
    /** 
    * @brief Publisher that publishes Global NWU Coordinates
    */
    global_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>(
        "/" + _id + "/global_pose", 10);
    /** 
    * @brief Publisher that publishes Bspline that is optimized
    */
    opt_bspline_pub = _nh.advertise<px4_path_planner::Bspline>(
        "/" + _id + "/path/opt_bspline", 10);
    

    multi_uav_pub = _nh.advertise<px4_path_planner::multi_agent>(
        "/multi_agent_trajectory", 10);


    /* ------------ Service Clients ------------ */
    /** 
    * @brief Service Client that handles arming in Mavros
    */
    arming_client = _nh.serviceClient<mavros_msgs::CommandBool>(
        "/" + _id + "/mavros/cmd/arming");
    /** 
    * @brief Service Client that handles mode switching in Mavros
    */
    set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>(
        "/" + _id + "/mavros/set_mode");



    /* ------------ Timers ------------ */
    /** 
    * @brief Mission Timer that handles output and publishing of setpoints to PX4
    */
    mission_timer = _nh.createTimer(ros::Duration(_send_desired_interval), &taskmaster::missionTimer, this, false, false);
    opt_timer = _nh.createTimer(ros::Duration(_send_optimization_interval), &taskmaster::optimization, this, false, false);
    hover_timer = _nh.createTimer(ros::Duration(_send_desired_interval), &taskmaster::hoverTimer, this, false, false);
    update_drone_traj_timer = _nh.createTimer(ros::Duration(_send_optimization_interval), &taskmaster::trajUpdate, this, false, false);

    printf("%s[main.cpp] taskmaster Setup Ready! \n", KGRN);
}

taskmaster::~taskmaster(){
}

void taskmaster::initialisation()
{
    ros::Rate rate(20.0);
    ros::Time last_request = ros::Time::now();

    // Make Sure FCU is connected, wait for 5s if not connected.
    printf("%s[main.cpp] FCU Connection is %s \n", uav_current_state.connected? KBLU : KRED, uav_current_state.connected? "up" : "down");
    while (ros::ok() && !uav_current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        if (ros::Time::now() - last_request > ros::Duration(5.0))
        {
            printf("%s[main.cpp] FCU not connected for 5s. Stop initialisation procedure \n", KRED);
            printf("%s[main.cpp] Check for FCU connection \n", KRED);
            last_request = ros::Time::now();
            return;
        }
    }

    printf("%s[main.cpp] FCU connected! \n", KBLU);
    _initialised = true;

    return;
}

void taskmaster::uavCommandCallBack(int msg)
{
    int idx = msg;
    printf("%s[main.cpp] User command %d received \n", KBLU, idx);
    int tries = 0;
    ros::Time last_request = ros::Time::now();

    switch (idx)
    {
    case TAKEOFF:
    {
        if (takeoff_flag)
        {
            printf("%s[main.cpp] Vehicle has already taken off, going back to hover \n", KRED);
            uav_task = kHover;
            break;
        }

        if (!uav_current_state.armed && retry < tries)
        {
            printf("%s[main.cpp] Vehicle is not armed, please ask safety pilot to arm the vehicle [Attempt %d] \n", KRED, retry + 1);
            printf("%s[main.cpp] Overwrite and arm in %d \n", KRED, tries - (retry));
            retry++;
            break;
        }
        printf("%s[main.cpp] Takeoff command received! \n", KYEL);
        arm_cmd.request.value = true;

        if (set_offboard())
        {
            printf("%s[main.cpp] Offboard mode activated going to run takeoff \n", KBLU);
            mission_timer.start();
            hover_timer.start();
            update_drone_traj_timer.start();
            printf("%s[main.cpp] Mission timer started! \n", KGRN);
            takeoff_flag = true;
        }

        // Set Takeoff Position
        // *** This uses ENU, obsolete since we use NWU ***
        // takeoff_pos.x() = uav_pose.pose.position.x;
        // takeoff_pos.y() = uav_pose.pose.position.y;
        // takeoff_pos.z() = uav_pose.pose.position.z + _takeoff_height;
        
        takeoff_pos.x() = uav_global_pose.pose.position.x;
        takeoff_pos.y() = uav_global_pose.pose.position.y;
        takeoff_pos.z() = uav_global_pose.pose.position.z + _takeoff_height;

        /** @brief Set up Takeoff Waypoints */
        MatrixXd wp = MatrixXd::Zero(3,1);
        wp.col(0) = takeoff_pos;
        std::cout << KBLU << "[main.cpp] " << "[Takeoff Waypoint] " << std::endl << KNRM << wp << std::endl;

        double clean_buffer = ros::Time::now().toSec();

        double buffer = 3.0;
        std::cout << KBLU << "[main.cpp] " << "Takeoff buffer of " << KNRM << buffer << KBLU << "s" << std::endl;
        double last_interval = ros::Time::now().toSec();
        // while loop to clean out buffer for command for 3s
        while (abs(clean_buffer - ros::Time::now().toSec()) < buffer)
        {
            // WARNING : Publishing too fast will result in the mavlink bandwidth to be clogged up hence we need to limit this rate
            if (ros::Time::now().toSec() - last_interval > _send_desired_interval) 
            {
                Vector3d home_pose = {home.pose.position.x, home.pose.position.y, home.pose.position.z};
                // uavDesiredControlHandler
                uavDesiredControlHandler(home_pose, 
                    Vector3d (0,0,0),
                    Vector3d (0,0,0),
                    home_yaw);
                //             std::cout << KGRN << "[task.h] home_yaw=" << KNRM <<
                // home_yaw << std::endl;
                // std::cout << KBLU << "[main.cpp] Publish buffer" << KNRM << home_pose << std::endl;
                last_interval = ros::Time::now().toSec();
            }
            
        }

        // *** This uses ENU, obsolete since we use NWU ***
        // TrajectoryGeneration(Vector3d (uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z), 
        //     wp, kIdle, kTakeOff);
        
        TrajectoryGeneration(Vector3d (uav_global_pose.pose.position.x, 
            uav_global_pose.pose.position.y, uav_global_pose.pose.position.z), 
            wp, kIdle, kTakeOff, 1);
        break;
    }

    case HOVER:
    {
        if (!takeoff_flag)
        {
            printf("%s[main.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
            break;
        }

        uav_task = kHover;
        last_mission_pos = global_pos;
        last_mission_yaw = yaw_nwu;
        printf("%s[main.cpp] UAV sent to hover! \n", KBLU);
        break;
    }

    case MISSION:
    {
        if (!takeoff_flag)
        {
            printf("%s[main.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
            break;
        }
        printf("%s[main.cpp] Mission command received! \n", KYEL);
        printf("%s[main.cpp] Loading Trajectory... \n", KBLU);

        // Error when mission mode is sent while in another mission mode this is disastrous
        // Throw the mission into hover while it calculates
        // if (!task_complete)
        // {
        //     uav_task = kHover;
        //     last_mission_pos = global_pos;
        //     last_mission_yaw = yaw_nwu;
        //     printf("%s[main.cpp] Task is not complete! \n", KRED);
        //     break;
        // }

        if (param_check < 2)
        {
            taskmaster::uav_task = kHover;
            last_mission_pos = global_pos;
            last_mission_yaw = yaw_nwu;
            printf("%s[main.cpp] Formation and Solo params are not received! Return to Hover! \n", KRED);
            break;
        }

        // Mission changes is the switch in missions
        if (mission_mode[mission_type_count] != bypass)
        {
            // We will start finding the first of the mission waypoints
            // *** This uses ENU, obsolete since we use NWU ***
            // TrajectoryGeneration(Vector3d (uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z), 
            //         mission_wp[0], kHover, kMission);

            TrajectoryGeneration(Vector3d (uav_global_pose.pose.position.x, 
                uav_global_pose.pose.position.y, uav_global_pose.pose.position.z), 
                mission_wp[mission_type_count], kHover, kMission, mission_mode[mission_type_count]);             
        }
        else
        {
            bypass_initialize = false;
            bypass_init_time = -1.0; // Not initialized condition for time
        }

        /** 
        * @brief Enable start of optimization mission timer after RRT
        */
        if (mission_mode[mission_type_count] == bspline_avoid_opt)
            opt_timer.start();


        else if (mission_mode.size() == 0)
        {
            printf("%s[main.cpp] Mission not Valid! Return to Hover! \n", KRED);
            taskmaster::uav_task = kHover;
        }
        else
            taskmaster::uav_task = kMission;

        break;
    }

    case HOME:
    {
        if (!takeoff_flag)
        {
            printf("%s[main.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
            break;
        }
        printf("%s[main.cpp] Home command received! \n", KYEL);
        printf("%s[main.cpp] Loading Trajectory... \n", KBLU);
        
        /** @brief Set up Home Waypoints */
        MatrixXd wp = MatrixXd::Zero(3,1);
        wp.col(0) = takeoff_pos;
        std::cout << KBLU << "[main.cpp] " << "[Home Waypoint] " << std::endl << KNRM << wp << std::endl;

        // *** This uses ENU, obsolete since we use NWU ***
        // TrajectoryGeneration(Vector3d (uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z), 
        //     wp, kHover, kHome);

        TrajectoryGeneration(Vector3d (uav_global_pose.pose.position.x, 
            uav_global_pose.pose.position.y, uav_global_pose.pose.position.z), 
            wp, kHover, kHome, 1);
        break;
    }

    case LAND:
    {
        if (!takeoff_flag)
        {
            printf("%s[main.cpp] Vehicle has not taken off, please issue takeoff command first \n", KRED);
            break;
        }
        // Check if its at XY of home position, if not we send it to HOME then to land
        double sqdist_hpos = pow(takeoff_pos.x(),2) + pow(takeoff_pos.y(),2);
        // *** This uses ENU, obsolete since we use NWU ***
        // double sqdist_pos = pow(uav_pose.pose.position.x,2) + pow(uav_pose.pose.position.y,2);

        double sqdist_pos = pow(uav_global_pose.pose.position.x,2) + pow(uav_global_pose.pose.position.y,2);

        // if (sqdist_pos - sqdist_hpos > 1.0)
        // {
        //     printf("%s[main.cpp] Call home first \n", KRED);
        //     printf("%s[main.cpp] Position not suitable for landing, no RTL enabled, at dist %lf \n", KRED, sqdist_pos - sqdist_hpos);
        //     break;
        // }
        printf("%s[main.cpp] Land command received! \n", KYEL);
        printf("%s[main.cpp] Loading Trajectory... \n", KBLU);

        /** @brief Set up Land Waypoints */
        MatrixXd wp = MatrixXd::Zero(3,1);

        // wp.col(0) = Vector3d (home.pose.position.x, home.pose.position.y, home.pose.position.z);
        // *** This uses ENU, obsolete since we use NWU ***
        // wp.col(0) = Vector3d (uav_pose.pose.position.x, 
        //     uav_pose.pose.position.y, home.pose.position.z);
        
        wp.col(0) = Vector3d (uav_global_pose.pose.position.x, 
            uav_global_pose.pose.position.y, home.pose.position.z);
        std::cout << KBLU << "[main.cpp] " << "[Land Waypoint] " << std::endl << KNRM << wp << std::endl;

        // *** This uses ENU, obsolete since we use NWU ***
        // TrajectoryGeneration(Vector3d (uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z), 
        //     wp, kHover, kLand);
        
        TrajectoryGeneration(Vector3d (uav_global_pose.pose.position.x, 
            uav_global_pose.pose.position.y, uav_global_pose.pose.position.z), 
            wp, kHover, kLand, 1);
        printf("%s[main.cpp] UAV is landing! \n", KBLU);
        break;
    }

    default:
        break;
    }
}

bool taskmaster::set_offboard()
{
    ros::Rate rate(20.0);
    ros::Time last_request = ros::Time::now();

    mission_previous_message_time = ros::Time::now();

    // *** This uses ENU, obsolete since we use NWU ***
    // home.pose.position.x = uav_pose.pose.position.x;
    // home.pose.position.y = uav_pose.pose.position.y;
    // home.pose.position.z = uav_pose.pose.position.z;
    // home_yaw = yaw;

    // *** If takeoff is immediately sent, this will help to stream the correct data to the program first
    // *** Will give a 5sec buffer ***
    while (ros::Time::now() - last_request > ros::Duration(5.0))
    {

    }

    home.pose.position.x = uav_global_pose.pose.position.x;
    home.pose.position.y = uav_global_pose.pose.position.y;
    home.pose.position.z = uav_global_pose.pose.position.z;

    geometry_msgs::PoseStamped home_enu;
    //send a few setpoints before starting
    for (int i = 20; ros::ok() && i > 0; --i)
    {
        // *** This uses ENU, obsolete since we use NWU ***
        // local_pos_pub.publish(home);

        // We must convert home from NWU to ENU
        // home_enu = convert_global_nwu_to_enu(home);
        // local_pos_pub.publish(home_enu);

        uavDesiredControlHandler(Vector3d(home.pose.position.x,home.pose.position.y,home.pose.position.z), 
        Vector3d(0,0,0), Vector3d(0,0,0), yaw_nwu);

        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    bool is_mode_ready = false;
    last_request = ros::Time::now();

    while (!is_mode_ready)
    {
        if (uav_current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            printf("%s[main.cpp] Try set offboard \n", KYEL);
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                printf("%s[main.cpp] Offboard Enabled \n", KGRN);
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!uav_current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(2.0)))
            {
                printf("%s[main.cpp] Try arm \n", KYEL);
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    printf("%s[main.cpp] Vehicle armed \n", KGRN);  
                }
                last_request = ros::Time::now();
            }
        }

        // *** This uses ENU, obsolete since we use NWU ***
        // local_pos_pub.publish(home);

        // We must convert home from NWU to ENU
        // local_pos_pub.publish(home_enu);

        uavDesiredControlHandler(Vector3d(home.pose.position.x,home.pose.position.y,home.pose.position.z), 
        Vector3d(0,0,0), Vector3d(0,0,0), yaw_nwu);

        is_mode_ready = (uav_current_state.mode == "OFFBOARD") && uav_current_state.armed;
        ros::spinOnce();
        rate.sleep();

        // std::cout << KGRN << "[task.h] home_yaw_nwu_yaw=" << KNRM <<
        //     yaw_nwu << std::endl;
        home_yaw = yaw_nwu;
        // std::cout << KYEL << "[main.cpp] Home Position : " << KNRM <<
        //     home.pose.position.x << " " << home.pose.position.y << " " <<
        //     home.pose.position.z << " " << home_yaw << std::endl;
    }

    if (is_mode_ready)
    {
        printf("%s[main.cpp] Offboard mode activated! \n", KBLU);
    }

    return is_mode_ready;
}

void taskmaster::optimization(const ros::TimerEvent &)
{
    // Will optimize and then add the values to the trajectory
    // Find finite horizon
    // Get global cp for [initial guess] = global cp changes in every iteration
    // Get fixed trajectory for [terminal cost]
    // Get knots + global cp for [reciprocal cost]

    // Update the new [pos vel acc and yaw] so that uav can call on the desired states 
    bs::bspline_optimization bsp_opt;
    // MatrixXd opt = bsp_opt.solver(traj.GetGlobalControlPoints(), traj.GetKnotSpan());

    ros::Time _start = ros::Time::now();
    bsp_opt.load(_weight_smooth, _weight_feas, _weight_term, _weight_static, _weight_reci);

    MatrixXd opt_cp = bsp_opt.solver(traj.GetGlobalControlPoints(), traj.GetKnotSpan(), traj.GetFixedControlPoints(), _max_acc);

    ros::Duration diff = ros::Time::now() - _start;

    printf("%s[main.cpp] Optimization Time %lf!\n", KGRN, diff.toSec());

    // Publish New Optimized Spline Message
    px4_path_planner::Bspline opt_bspline;
    opt_bspline.order = traj.GetOrder();
    opt_bspline.knot_division = traj.GetKnotDivision();
    VectorXd knots = VectorXd (traj.GetKnots());

    opt_bspline.global_control_points.reserve(opt_cp.cols());
    for (int i = 0; i < opt_cp.cols(); i++)
    {
        geometry_msgs::Point pt;
        pt.x = opt_cp(0,i);
        pt.y = opt_cp(1,i);
        pt.z = opt_cp(2,i);
        // std::cout << KGRN << "[main.cpp] CP \n" << KNRM << pt << std::endl;
        opt_bspline.global_control_points.push_back(pt);
    }
    // std::cout << KGRN << "[main.cpp] cp_size \n" << KNRM << bspline.global_control_points.size() << std::endl;

    // printf("%s[main.cpp] Publishing knots \n", KGRN);
    opt_bspline.knot.reserve(knots.rows());
    for (int j = 0; j < knots.size(); j++)
    {
        opt_bspline.knot[j] = (float)knots[j];
    }

    opt_bspline_pub.publish(opt_bspline);

}

void taskmaster::trajUpdate(const ros::TimerEvent &)
{
    std::lock_guard<std::mutex> _lock(uavsTrajMutex);
    
    // Initialization if id is not found
    if (traj_msg_idx < 0)
    {
        px4_path_planner::agent agent_msg;
        // std_msgs/Header header
        // int32 id
        // int32 task
        // px4_path_planner/Bspline bs
        agent_msg.header.stamp = ros::Time::now();
        agent_msg.id = uav_id;
        agent_msg.task = uav_task;

        if (uav_task != kMission)
            agent_msg.bs.global_control_points.push_back(uav_global_pose.pose.position);

        multi_uav_msg.agent.push_back(agent_msg);
        multi_uav_pub.publish(multi_uav_msg);
        return;
    }

    px4_path_planner::Bspline bspline;

    // If there is no rows in knots, then there is not trajectory to publish
    if (traj.GetKnots().rows() <= 1)
        bspline.global_control_points.push_back(uav_global_pose.pose.position);

    // Else there may be a trajectory to publish but we have to check whether the data is outdated
    else 
    {
        MatrixXd gcp = MatrixXd (traj.GetGlobalControlPoints());
        VectorXd knots = VectorXd (traj.GetKnots());
        if (ros::Time::now().toSec() - knots[0] < 0 || ros::Time::now().toSec() - knots[knots.rows()-1] > 0)
            bspline.global_control_points.push_back(uav_global_pose.pose.position);
        else
        {
            // If all is good we can update the message
            bspline.order = traj.GetOrder();
            bspline.knot_division = traj.GetKnotDivision();

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
                bspline.knot.push_back((float)knots[j]);
            }
        } 
    }
    multi_uav_msg.agent[traj_msg_idx].header.seq = traj_seq;

    traj_seq++;

    multi_uav_msg.agent[traj_msg_idx].bs = bspline;
    multi_uav_msg.agent[traj_msg_idx].header.stamp = ros::Time::now();
    multi_uav_msg.agent[traj_msg_idx].task = uav_task;
    multi_uav_pub.publish(multi_uav_msg);
}

void taskmaster::hoverTimer(const ros::TimerEvent &)
{
    switch (uav_task)
    {
    case kHover:
    {
        if (ros::Time::now().toSec() - last_request_timer > _send_desired_interval)
        {
            uavDesiredControlHandler(last_mission_pos, 
            Vector3d (0,0,0),
            Vector3d (0,0,0),
            last_mission_yaw);
            last_request_timer = ros::Time::now().toSec();
        }

        break;
    }
    default:
        break;
    }
}


void taskmaster::missionTimer(const ros::TimerEvent &)
{
    if (uav_task != uav_prev_task)
    {
        task_complete = false;
        last_request_timer = traj.GetStartTime();
        printf("%s[main.cpp] Starting New Task %d @ time(%lf)! \n", KBLU, uav_task, last_request_timer);
        uav_prev_task = uav_task;
    }

    switch (uav_task)
    {
    case kTakeOff:
    {
        PublishDesiredControl(takeoff_pos, kHover);        
        break;
    }

    case kHome:
    {
        PublishDesiredControl(takeoff_pos, kHover);        
        break;
    }

    case kMission:
    {
        // Does not use PublishDesiredControl(takeoff_pos, kHover);
        // Need to do replanning
        if (!task_complete && (ros::Time::now().toSec() - last_request_timer > _send_desired_interval))
        {
            // Time Based Trajectory
            if (mission_mode[mission_type_count] != bypass)
            {
                // If Mission is not using bypass means we are using time based trajectory
                // std::cout << KYEL << "[main.cpp] Time Delay " << KNRM << (ros::Time::now().toSec() - last_request_timer) - (_send_desired_interval) << std::endl;
                double fact = floor((ros::Time::now().toSec() - traj.GetStartTime())/_send_desired_interval);
                
                last_request_timer = traj.GetStartTime() + _send_desired_interval * fact;

                if (traj.isCompleted(ros::Time::now().toSec()))
                {
                    printf("%s[main.cpp] Completed Trajectory %d\n", KGRN, mission_type_count);
                    printf("%s[main.cpp] Check for next mission, %d / %d, %s\n", KGRN, mission_type_count, (int)(mission_wp.size()-1),
                        mission_type_count < (mission_wp.size()-1) ? "true" : "false");
                    
                    opt_timer.stop();
                    
                    // Check if there are still any more missions
                    if (mission_type_count < mission_wp.size()-1)
                    {
                        mission_type_count++;
                        printf("%s[main.cpp] Moving on to next mission, idx %d\n", KGRN, mission_type_count);
                        
                        // *** This uses ENU, obsolete since we use NWU ***
                        // TrajectoryGeneration(Vector3d (uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z), 
                        //     mission_wp[mission_type_count], kHover, kMission);
                        
                        // TrajectoryGeneration(Vector3d (uav_global_pose.pose.position.x, 
                        //     uav_global_pose.pose.position.y, uav_global_pose.pose.position.z), 
                        //     mission_wp[mission_type_count], kHover, kMission, mission_mode[mission_type_count]);
                        
                        // if (mission_mode[mission_type_count] == bspline_avoid_opt)
                        //     opt_timer.start();

                        uavCommandCallBack(MISSION);

                        // Don't break here, we have to return or else it will go through an empty bspline update

                        return;
                    }

                    printf("%s[main.cpp] Mission Complete \n", KGRN);
                    task_complete = true;
                    Vector3d last_pos;
                    if (!traj.returnControlPointPose(
                        traj.returnFixedCPColumn()-1, &last_pos))
                    {
                        printf("%s[main.cpp] Reject Last Position\n", KRED);
                        // printf("%s[main.cpp] Still need to figure what to do from here\n", KRED);
                        break;
                    }
                    last_mission_pos = last_pos;

                    // *** This uses ENU, obsolete since we use NWU ***
                    // last_mission_yaw = yaw;

                    last_mission_yaw = yaw_nwu;

                    uav_task = kHover;
                    break;
                }
            }
            
            // Bypass Trajectory
            else
            {
                // If Mission is in bypass we just use time
                last_request_timer = ros::Time::now().toSec();

                if (!bypass_initialize)
                {
                    if (bypass_init_time < 0)
                        bypass_init_time = ros::Time::now().toSec();
                    // To facilitate a buffer so that a non-zero value will not be sent to px4 from the bypass message 
                    // when the bypass message is calculating a proper message
                    // approx 0.25s will suffice
                    if (ros::Time::now().toSec() - bypass_init_time > 0.25)
                        bypass_initialize = true;
                    return;
                }
                
                // printf("%s[main.cpp] Bypass Mode activated!\n", KRED);
                // If bypass_message callback shows long delay, we will return to hover
                // The vehicle will exit the mode if target setpoints are not received at a rate of > 2Hz
                if (ros::Time::now().toSec() - bypass_previous_message_time.toSec() > 2.0)
                {
                    printf("%s[main.cpp] Reject Bypass Mode and move to Hover!\n", KRED);
                    printf("%s[main.cpp] Latency %lf!\n", KRED, ros::Time::now().toSec() - bypass_previous_message_time.toSec());
                    
                    // *** This uses ENU, obsolete since we use NWU ***
                    // last_mission_pos = current_pos;
                    // last_mission_yaw = yaw;

                    last_mission_pos = global_pos;
                    last_mission_yaw = yaw_nwu;

                    uav_task = kHover;
                    break;
                }
                else
                {
                    // Use the bypass message in mavros_msgs::PositionTarget NWU frame
                    Vector3d bypass_pos = Vector3d(bypass_sp.position.x,
                        bypass_sp.position.y, bypass_sp.position.z);
                    Vector3d bypass_vel = Vector3d(bypass_sp.velocity.x,
                        bypass_sp.velocity.y, bypass_sp.velocity.z);
                    double bypass_yaw = (double)bypass_sp.yaw;
                    
                    uavDesiredControlHandler(bypass_pos, bypass_vel, 
                        Vector3d(0,0,0), bypass_yaw);
                }
            }
        
        }
        else
        {
            break;
        }

        

        /** @brief Recalculate trajectory during mission = Handled by optimization timer **/
        if (mission_mode[mission_type_count] != bypass)
        {
            Vector3d pos; Vector3d vel; Vector3d acc; 
            double _calculated_yaw = traj.GetDesiredYaw(last_request_timer, yaw);

            PublishBspline();
            traj.GetDesiredState(last_request_timer, &pos, &vel, &acc);
            
            // Get the desired pose from the trajectory handler
            uavDesiredControlHandler(pos, vel, acc, _calculated_yaw);
        
            switch (mission_mode[mission_type_count])
            {
                // Do not need to do anything for case bspline and bypass
                case bspline_avoid:
                {
                    // Add any constant search avoidance here
                    break;
                }
                case bspline_avoid_opt:
                {
                    // Add optimization code here
                    break;
                }
            }
        }
        
        break;
    }

    case kLand:
    {
        // Somehow not sending any setpoint disarms the drone
        // We killing it with idleness (kindness)
        PublishDesiredControl(Vector3d (home.pose.position.x, home.pose.position.y, home.pose.position.z), 
            kIdle);
        
        // At the last point we reset it to not takeoff
        if (uav_task == kIdle)
        {
            takeoff_flag = false;
            printf("%s[main.cpp] Takeoff flag return to false, you can issue takeoff again\n", KRED);
        }
        
        break;
    }

    default:
        break;
    }
}