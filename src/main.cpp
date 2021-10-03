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
    _nh.param<double>("trajectory_interval", _interval, 0.2);
    _nh.param<double>("takeoff_height", _takeoff_height, 1.2);
    _nh.param<std::string>("wp_file_location", _wp_file_location, "~/wp.csv");

    printf("%s[main.cpp] taskmaster _interval = %lf \n", KBLU, _interval);
    printf("%s[main.cpp] taskmaster _takeoff_height = %lf \n", KBLU, _takeoff_height);
    printf("%s[main.cpp] taskmaster _wp_file_location = %s \n", KBLU, _wp_file_location.c_str());

    state_sub = _nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, boost::bind(&taskmaster::uavStateCallBack, this, _1));
    // Get current uav pose
    uav_pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 1, &taskmaster::uavPoseCallback, this);
    uav_gps_cur_sub = _nh.subscribe<sensor_msgs::NavSatFix>(
        "/mavros/global_position/global", 1, &taskmaster::gpsCurrentCallback, this);
    uav_gps_home_sub = _nh.subscribe<mavros_msgs::HomePosition>(
        "/mavros/home_position/home", 1, &taskmaster::gpsHomeCallback, this);
    uav_cmd_sub = _nh.subscribe<std_msgs::Byte>(
        "/user", 1, &taskmaster::uavCommandCallBack, this);

    // Publisher that publishes control setpoints to mavros
    local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);
    
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

    traj.start(_wp_file_location,_interval);

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
        printf("%s[main.cpp] Takeoff command received! \n", KBLU);
        arm_cmd.request.value = true;
        uav_task = kTakeOff;

        if (set_offboard())
        {
            printf("%s[main.cpp] Offboard mode activated going to run takeoff \n", KBLU);
            mission_timer.start();
            printf("%s[main.cpp] Mission timer started! \n", KYEL);
            takeoff_flag = true;
        }
        break;
    }

    // case MISSION:
    // {
    //     if (!takeoff_flag)
    //     {
    //         ROS_ERROR("Vehilce has not taken off, please issue takeoff command first.");
    //         break;
    //     }
    //     ROS_INFO("command received!");
    //     ROS_INFO("Loading Trajectory...");
    //     if (loadTrajectory())
    //     {

    //         ROS_INFO("trajectory is loaded.");
    //         uavTask = kMission;
    //     }
    //     else
    //     {
    //         break;
    //     }

    //     break;
    // }

    case LAND:
    {
        printf("%s[main.cpp] Land command received! \n", KBLU);
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
            if (abs(uav_pose.pose.position.z - takeoff_pos.z()) < 0.05)
            {
                printf("%s[main.cpp] Current Altitude is: %lf/%lf \n", KBLU, uav_pose.pose.position.z, takeoff_pos.z());
                printf("%s[main.cpp] Takeoff Complete \n", KGRN);
                task_complete = true;
            }
        }

        geometry_msgs::PoseStamped pos_sp;
        pos_sp.pose.position.x = takeoff_pos.x();
        pos_sp.pose.position.y = takeoff_pos.y();
        pos_sp.pose.position.z = takeoff_pos.z();

        // pos_sp.type_mask = 3576;
        local_pos_pub.publish(pos_sp);
        break;
    }

    // case kMission:
    // {
    //     mavros_msgs::PositionTarget pos_sp;
    //     ROS_INFO("Mission timer Doing mission!");
    //     if (_points_id <= _traj_list.size() - 1)
    //     {
    //         std::cout << "Go to next ref: " << _points_id << std::endl;
    //         std::cout << "Position: " << _traj_list[_points_id].pos.x << " " << _traj_list[_points_id].pos.y << " " << _traj_list[_points_id].pos.z << std::endl;
    //         std::cout << "Velocity: " << _traj_list[_points_id].vel.x << " " << _traj_list[_points_id].vel.y << " " << _traj_list[_points_id].vel.z << std::endl;
    //         std::cout << "Aceleration: " << _traj_list[_points_id].acc.x << " " << _traj_list[_points_id].acc.y << " " << _traj_list[_points_id].acc.z << std::endl;
    //         pos_sp.position.x = _traj_list[_points_id].pos.x;
    //         pos_sp.position.y = _traj_list[_points_id].pos.y;
    //         pos_sp.position.z = _traj_list[_points_id].pos.z;
    //         pos_sp.velocity.x = _traj_list[_points_id].vel.x;
    //         pos_sp.velocity.y = _traj_list[_points_id].vel.y;
    //         pos_sp.velocity.z = _traj_list[_points_id].vel.z;
    //         pos_sp.acceleration_or_force.x = _traj_list[_points_id].acc.x;
    //         pos_sp.acceleration_or_force.y = _traj_list[_points_id].acc.y;
    //         pos_sp.acceleration_or_force.z = _traj_list[_points_id].acc.z;
    //         //pos_sp.yaw = atan2(pos_sp.velocity.y, pos_sp.velocity.x); // yaw control
    //         pos_sp.yaw = 0; //fixed yaw

    //         _points_id++;
    //     }
    //     else if (_points_id == _traj_list.size())
    //     {
    //         _points_id--;
    //         std::cout << "Hold last ref: " << _points_id << std::endl;
    //         std::cout << "Position: " << _traj_list[_points_id].pos.x << " " << _traj_list[_points_id].pos.y << " " << _traj_list[_points_id].pos.z << std::endl;
    //         std::cout << "Velocity: " << _traj_list[_points_id].vel.x << " " << _traj_list[_points_id].vel.y << " " << _traj_list[_points_id].vel.z << std::endl;
    //         std::cout << "Aceleration: " << _traj_list[_points_id].acc.x << " " << _traj_list[_points_id].acc.y << " " << _traj_list[_points_id].acc.z << std::endl;
    //         pos_sp.position.x = _traj_list[_points_id].pos.x;
    //         pos_sp.position.y = _traj_list[_points_id].pos.y;
    //         pos_sp.position.z = _traj_list[_points_id].pos.z;
    //         pos_sp.velocity.x = _traj_list[_points_id].vel.x;
    //         pos_sp.velocity.y = _traj_list[_points_id].vel.y;
    //         pos_sp.velocity.z = _traj_list[_points_id].vel.z;
    //         pos_sp.acceleration_or_force.x = _traj_list[_points_id].acc.x;
    //         pos_sp.acceleration_or_force.y = _traj_list[_points_id].acc.y;
    //         pos_sp.acceleration_or_force.z = _traj_list[_points_id].acc.z;
    //         _points_id++;
    //     }

    //     pos_sp.type_mask = 32768;
    //     std::cout << "Yaw: " << pos_sp.yaw << std::endl;
    //     Position_Setpoint_Pub.publish(pos_sp);
    // }
    case kLand:
    {
        if (!task_complete && (ros::Time::now() - last_request_timer > ros::Duration(1.0)))
        {
            last_request_timer = ros::Time::now();
            if (abs(uav_pose.pose.position.z - home.pose.position.z) < 0.05)
            {
                printf("%s[main.cpp] Current Altitude is: %lf/%lf \n", KBLU, uav_pose.pose.position.z, takeoff_pos.z());
                printf("%s[main.cpp] Land Complete \n", KGRN);
                task_complete = true;
            }
        }

        ros::Rate rate(20.0);
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "AUTO.LAND";

        bool is_mode_ready = false;
        ros::Time last_request_inner = ros::Time::now();

        while (!is_mode_ready)
        {
            if (uav_current_state.mode != "AUTO.LAND" &&
                (ros::Time::now() - last_request_inner > ros::Duration(2.0)))
            {
                printf("%s[main.cpp] Try set AUTO.LAND \n", KYEL);
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                    printf("%s[main.cpp] AUTO.LAND Enabled \n", KGRN);
                    is_mode_ready = true;
                }
                last_request_inner = ros::Time::now();
            }
            ros::spinOnce();
            rate.sleep();
        }

        takeoff_flag = false;
        break;
    }

    default:
        break;
    }
}