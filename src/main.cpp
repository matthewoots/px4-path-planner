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
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    taskmaster taskmaster(nh);
    ros::spin();
    return 0;
}

taskmaster::taskmaster(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
{
    // Interval
    _nh.param<bool>("debug", _debug, false);
    _nh.param<bool>("setpoint_raw_mode", _setpoint_raw_mode, true);
    _nh.param<int>("unique_id_range", _unique_id_range, 10);
    _nh.param<double>("send_desired_interval", _send_desired_interval, 0.1);
    _nh.param<double>("control_points_interval", _control_points_interval, 0.1);
    _nh.param<double>("trajectory_calc_interval", _trajectory_calc_interval, 1.5);
    _nh.param<double>("takeoff_height", _takeoff_height, 1.2);
    _nh.param<double>("takeoff_velocity", _takeoff_velocity, 1.0);
    _nh.param<std::string>("wp_file_location", _wp_file_location, "~/wp.csv");

    printf("%s[main.cpp] Parameter (_debug = %s) \n", KBLU, _debug ? "true" : "false");
    printf("%s[main.cpp] Parameter (_setpoint_raw_mode = %s) \n", KBLU, _setpoint_raw_mode ? "true" : "false");
    printf("%s[main.cpp] Parameter (_unique_id_range = %d) \n", KBLU, _unique_id_range);
    printf("%s[main.cpp] Parameter (_send_desired_interval = %lf) \n", KBLU, _send_desired_interval);
    printf("%s[main.cpp] Parameter (_control_points_interval = %lf) \n", KBLU, _control_points_interval);
    printf("%s[main.cpp] Parameter (_trajectory_calc_interval = %lf) \n", KBLU, _trajectory_calc_interval);
    printf("%s[main.cpp] Parameter (_takeoff_height = %lf) \n", KBLU, _takeoff_height);
    printf("%s[main.cpp] Parameter (_takeoff_velocity = %lf) \n", KBLU, _takeoff_velocity);
    printf("%s[main.cpp] Parameter (_wp_file_location = %s) \n", KBLU, _wp_file_location.c_str());
    

    state_sub = _nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, boost::bind(&taskmaster::uavStateCallBack, this, _1));
    // Get current uav pose
    uav_pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 1, &taskmaster::uavPoseCallback, this);
    // Get current uav velocity
    uav_vel_sub = _nh.subscribe<geometry_msgs::TwistStamped>(
        "/mavros/local_position/velocity_local", 1, &taskmaster::uavVelCallback, this);
    uav_gps_cur_sub = _nh.subscribe<sensor_msgs::NavSatFix>(
        "/mavros/global_position/global", 1, &taskmaster::gpsCurrentCallback, this);
    uav_gps_home_sub = _nh.subscribe<mavros_msgs::HomePosition>(
        "/mavros/home_position/home", 1, &taskmaster::gpsHomeCallback, this);
    uav_cmd_sub = _nh.subscribe<std_msgs::Byte>(
        "/user", 1, &taskmaster::uavCommandCallBack, this);

    // Publisher that publishes control position setpoints to mavros
    local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);
    // Publisher that publishes control raw setpoints to mavros
    local_pos_raw_pub = _nh.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);
    
    arming_client = _nh.serviceClient<mavros_msgs::CommandBool>(
        "/mavros/cmd/arming");
    set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>(
        "/mavros/set_mode");

    mission_timer = _nh.createTimer(ros::Duration(0.05), &taskmaster::missionTimer, this, false, false);

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

    // Testing and debug purposes concerning the trajectory node
    if (_debug && !_vel_initialised)
    {
        traj.setTrajectory(_wp_file_location, _send_desired_interval, _control_points_interval,
            current_vel, Vector3d (0.0, 0.0, 0.0));
        _vel_initialised = true;
    }

    printf("%s[main.cpp] FCU connected! \n", KBLU);
    _initialised = true;
    
    // Set Takeoff Position
    takeoff_pos.x() = uav_pose.pose.position.x;
    takeoff_pos.y() = uav_pose.pose.position.y;
    takeoff_pos.z() = uav_pose.pose.position.z + _takeoff_height;

    return;
}

void taskmaster::uavCommandCallBack(const std_msgs::Byte::ConstPtr &msg)
{
    int idx = msg->data;
    printf("%s[main.cpp] User command %d received \n", KBLU, idx);
    int tries = 0;
    ros::Time last_request = ros::Time::now();

    switch (idx)
    {
    case TAKEOFF:
    {
        if (!uav_current_state.armed && retry < tries)
        {
            printf("%s[main.cpp] Vehicle is not armed, please ask safety pilot to arm the vehicle [Attempt %d] \n", KRED, retry + 1);
            printf("%s[main.cpp] Overwrite and arm in %d \n", KRED, tries - (retry));
            retry++;
            break;
        }
        printf("%s[main.cpp] Takeoff command received! \n", KYEL);
        arm_cmd.request.value = true;
        uav_task = kTakeOff;

        if (set_offboard())
        {
            printf("%s[main.cpp] Offboard mode activated going to run takeoff \n", KBLU);
            mission_timer.start();
            printf("%s[main.cpp] Mission timer started! \n", KGRN);
            takeoff_flag = true;
        }
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

        // initial trajectory
        if (!traj.setTrajectory(_wp_file_location, _send_desired_interval, _control_points_interval,
            current_vel, Vector3d (0.0, 0.0, 0.0)))
        {
            printf("%s[main.cpp] Not able to load and set trajectory! \n", KRED);
            break;
        }
        uav_task = kMission;
        mission_start_time = ros::Time::now().toSec();
        prev_traj_replan = ros::Time::now().toSec();

        break;
    }

    case LAND:
    {
        printf("%s[main.cpp] Land command received! \n", KYEL);
        uav_task = kLand;
        // Mission timer will handle it for us
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

    home.pose.position.x = uav_pose.pose.position.x;
    home.pose.position.y = uav_pose.pose.position.y;
    home.pose.position.z = uav_pose.pose.position.z;

    //send a few setpoints before starting
    for (int i = 20; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(home);
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
        local_pos_pub.publish(home);
        is_mode_ready = (uav_current_state.mode == "OFFBOARD") && uav_current_state.armed;
        ros::spinOnce();
        rate.sleep();
    }

    if (is_mode_ready)
    {
        printf("%s[main.cpp] Offboard mode activated! \n", KBLU);
    }

    return is_mode_ready;
}

void taskmaster::missionTimer(const ros::TimerEvent &)
{
    if (uav_task != uav_prev_task)
    {
        printf("%s[main.cpp] Starting New Task %d! \n", KBLU, uav_task);
        task_complete = false;
        last_request_timer = ros::Time::now();
        uav_prev_task = uav_task;
    }

    switch (uav_task)
    {
    case kTakeOff:
    {
        if (!task_complete && (ros::Time::now() - last_request_timer > ros::Duration(1.0)))
        {
            last_request_timer = ros::Time::now();
            if (abs(uav_pose.pose.position.z - takeoff_pos.z()) < 0.1)
            {
                printf("%s[main.cpp] Current Altitude is: %lf/%lf \n", KBLU, uav_pose.pose.position.z, takeoff_pos.z());
                printf("%s[main.cpp] Takeoff Complete \n", KGRN);
                task_complete = true;
                last_mission_pos = current_pos;
                uav_task = kHover;
            }
        }
        uavDesiredControlHandler(takeoff_pos, Vector3d (0,0,_takeoff_velocity));
        
        break;
    }
    case kHover:
    {
        if (!task_complete && (ros::Time::now() - last_request_timer > ros::Duration(5.0)))
        {
            last_request_timer = ros::Time::now();
 
            printf("%s[main.cpp] Hovering @ ENU pos (%.2lf, %.2lf, %.2lf) last_mission_pos (%.2lf, %.2lf, %.2lf) \n", KBLU, 
                uav_pose.pose.position.x, 
                uav_pose.pose.position.y,
                uav_pose.pose.position.z,
                last_mission_pos.x(),
                last_mission_pos.y(),
                last_mission_pos.z());
        }
        uavDesiredControlHandler(last_mission_pos, Vector3d (0,0,0));

        break;
    }

    case kHome:
    {
        break;
    }

    case kMission:
    {
        if (!task_complete && (ros::Time::now() - last_request_timer > ros::Duration(_send_desired_interval)))
        {
            last_request_timer = ros::Time::now();
            break;
        }

        /** @brief recalculate trajectory during mission **/
        // if (ros::Time::now().toSec() - prev_traj_replan > _trajectory_calc_interval)
        // {
        //     printf("%s[main.cpp] Recalculating trajectory! \n", KGRN);
        //     traj.recalculateTrajectory(current_pos, current_vel, Vector3d (0,0,0),
        //         ros::Time::now().toSec() - prev_traj_replan);
        //     prev_traj_replan = ros::Time::now().toSec();
        //     break;
        // }

        traj.calcDesired(ros::Time::now().toSec() - prev_traj_replan);
        if (traj.isCompleted())
        {
            printf("%s[main.cpp] Mission Complete \n", KGRN);
            task_complete = true;
            last_mission_pos = traj.returnPose();
            uav_task = kHover;
        }
        
        // Get the desired pose from the trajectory handler
        uavDesiredControlHandler(traj.returnPose(), traj.returnVel());
        
        break;
    }

    case kLand:
    {
        // Check if its at XY of home position, if not we send it to HOME then to land
        double sqdist_hpos = pow(home.pose.position.x,2) + pow(home.pose.position.y,2);
        double sqdist_pos = pow(uav_pose.pose.position.x,2) + pow(uav_pose.pose.position.y,2);

        if (sqdist_pos - sqdist_hpos > 4.0)
        {
            printf("%s[main.cpp] Position not suitable for landing, RTL at dist %lf \n", KRED, sqdist_pos - sqdist_hpos);
            uav_task = kHome;
            break;
        }

        if (!task_complete && (ros::Time::now() - last_request_timer > ros::Duration(1.0)))
        {
            arm_cmd.request.value = false;
            last_request_timer = ros::Time::now();
            if (abs(uav_pose.pose.position.z - home.pose.position.z) < 0.1)
            {
                printf("%s[main.cpp] Current Altitude is: %lf/%lf \n", KBLU, uav_pose.pose.position.z, takeoff_pos.z());
                printf("%s[main.cpp] Land Complete \n", KGRN);
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    printf("%s[main.cpp] Vehicle disarmed \n", KGRN);  
                    task_complete = true;
                    takeoff_flag = false;
                } 
            }
        }
        uavDesiredControlHandler(Vector3d (home.pose.position.x,home.pose.position.y,home.pose.position.z), 
            Vector3d (0,0,0));

        /** @brief Currently switching to AUTO.LAND does not work so we will use offboard **/
        // ros::Rate rate(20.0);
        // mavros_msgs::SetMode offb_set_mode;
        // offb_set_mode.request.custom_mode = "AUTO.LAND";

        // bool is_mode_ready = false;
        // ros::Time last_request_inner = ros::Time::now();

        // while (!is_mode_ready)
        // {
        //     if (uav_current_state.mode != "AUTO.LAND" &&
        //         (ros::Time::now() - last_request_inner > ros::Duration(2.0)))
        //     {
        //         printf("%s[main.cpp] Try set AUTO.LAND \n", KYEL);
        //         if (set_mode_client.call(offb_set_mode) &&
        //             offb_set_mode.response.mode_sent)
        //         {
        //             printf("%s[main.cpp] AUTO.LAND Enabled \n", KGRN);
        //             is_mode_ready = true;
        //         }
        //         last_request_inner = ros::Time::now();
        //     }
        //     ros::spinOnce();
        //     rate.sleep();
        // }
        
        break;
    }

    default:
        break;
    }
}