#include "task.h"

taskmaster::taskmaster(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
{
    _points_id = 0;

    uavTask = kIdle;

    takeoff_flag = false;

    takeoff_announced = false;

    if !_initialised
        taskmaster::gpsHomeCallback

    // publish file timer
    _nh.param("interval", _interval, 0.2);
    std::cout << "[taskmaster] Trajectory_Interval =" << _interval << std::endl;

    cmd_sub = _nh.subscribe<std_msgs::Byte>("/user_cmd", 1, &taskmaster::cmd_cb, this);

    uav_state_sub = _nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &Task::uavStateCallBack, this);
    local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming"); // Not used

    takeoff_client = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    land_client = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

    set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    Position_Setpoint_Pub = _nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10); // Publisher that publishes control setpoints to mavros

    mission_timer = _nh.createTimer(ros::Duration(0.05), &taskmaster::missionTimer, this, false, false);
    // get current uav pose
    uav_pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(
        "/mavros/local_position/pose", 1, &taskmaster::uavPoseCallback, this);

    uav_gps_cur_sub = _nh.subscribe<sensor_msgs::NavSatFix>(
        "/mavros/global_position/global", 1, &taskmaster::gpsCurrentCallback, this);

    uav_gps_home_sub = _nh.subscribe<mavros_msgs::HomePosition>(
        "/mavros/home_position/home", 1, &taskmaster::gpsHomeCallback, this);

    _nh.param<std::string>("WP_Location", trajectory_location, "/home/zhengtian/px4_offb/src/px4_offb/param/waypoints");
    std::cout << "WP_Location: " << trajectory_location << std::endl;

    ROS_INFO("Offboard node is ready!");
}

taskmaster::~taskmaster()
{
}

void taskmaster::gpsHomeCallback(const mavros_msgs::HomePosition::ConstPtr &msg)
{
    uav_gps_home = *msg;
}

void taskmaster::gpsCurrentCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    uav_gps_cur = *msg;
}

void taskmaster::missionTimer(const ros::TimerEvent &)
{

    switch (uavTask)
    {
    case kTakeOff:
    {
        if (!takeoff_announced)
        {
            ROS_INFO("Mission timer Doing takeoff!");

            std::cout << "Current Altitude is: " << uav_pose.pose.position.z << std::endl;

            if (abs(uav_pose.pose.position.z - takeoff_height) < 0.01)
            {
                ROS_INFO("Takeoff Complete");
                takeoff_announced = true;
            }
        }

        mavros_msgs::PositionTarget pos_sp;
        pos_sp.position.x = takeoff_x;
        pos_sp.position.y = takeoff_y;
        pos_sp.position.z = takeoff_height;

        pos_sp.type_mask = 3576;
        Position_Setpoint_Pub.publish(pos_sp);
        break;
    }

    case kMission:
    {
        mavros_msgs::PositionTarget pos_sp;
        ROS_INFO("Mission timer Doing mission!");
        if (_points_id <= _traj_list.size() - 1)
        {
            std::cout << "Go to next ref: " << _points_id << std::endl;
            std::cout << "Position: " << _traj_list[_points_id].pos.x << " " << _traj_list[_points_id].pos.y << " " << _traj_list[_points_id].pos.z << std::endl;
            std::cout << "Velocity: " << _traj_list[_points_id].vel.x << " " << _traj_list[_points_id].vel.y << " " << _traj_list[_points_id].vel.z << std::endl;
            std::cout << "Aceleration: " << _traj_list[_points_id].acc.x << " " << _traj_list[_points_id].acc.y << " " << _traj_list[_points_id].acc.z << std::endl;
            pos_sp.position.x = _traj_list[_points_id].pos.x;
            pos_sp.position.y = _traj_list[_points_id].pos.y;
            pos_sp.position.z = _traj_list[_points_id].pos.z;
            pos_sp.velocity.x = _traj_list[_points_id].vel.x;
            pos_sp.velocity.y = _traj_list[_points_id].vel.y;
            pos_sp.velocity.z = _traj_list[_points_id].vel.z;
            pos_sp.acceleration_or_force.x = _traj_list[_points_id].acc.x;
            pos_sp.acceleration_or_force.y = _traj_list[_points_id].acc.y;
            pos_sp.acceleration_or_force.z = _traj_list[_points_id].acc.z;
            //pos_sp.yaw = atan2(pos_sp.velocity.y, pos_sp.velocity.x); // yaw control
            pos_sp.yaw = 0; //fixed yaw

            _points_id++;
        }
        else if (_points_id == _traj_list.size())
        {
            _points_id--;
            std::cout << "Hold last ref: " << _points_id << std::endl;
            std::cout << "Position: " << _traj_list[_points_id].pos.x << " " << _traj_list[_points_id].pos.y << " " << _traj_list[_points_id].pos.z << std::endl;
            std::cout << "Velocity: " << _traj_list[_points_id].vel.x << " " << _traj_list[_points_id].vel.y << " " << _traj_list[_points_id].vel.z << std::endl;
            std::cout << "Aceleration: " << _traj_list[_points_id].acc.x << " " << _traj_list[_points_id].acc.y << " " << _traj_list[_points_id].acc.z << std::endl;
            pos_sp.position.x = _traj_list[_points_id].pos.x;
            pos_sp.position.y = _traj_list[_points_id].pos.y;
            pos_sp.position.z = _traj_list[_points_id].pos.z;
            pos_sp.velocity.x = _traj_list[_points_id].vel.x;
            pos_sp.velocity.y = _traj_list[_points_id].vel.y;
            pos_sp.velocity.z = _traj_list[_points_id].vel.z;
            pos_sp.acceleration_or_force.x = _traj_list[_points_id].acc.x;
            pos_sp.acceleration_or_force.y = _traj_list[_points_id].acc.y;
            pos_sp.acceleration_or_force.z = _traj_list[_points_id].acc.z;
            _points_id++;
        }

        pos_sp.type_mask = 32768;
        std::cout << "Yaw: " << pos_sp.yaw << std::endl;
        Position_Setpoint_Pub.publish(pos_sp);
    }

    default:
        break;
    }
}

void taskmaster::land()
{
    ros::Rate rate(20.0);
    double curr_alt = uav_gps_cur.altitude;
    double curr_lon = uav_gps_cur.longitude;
    double curr_lat = uav_gps_cur.latitude;
    double home_alt = uav_gps_home.geo.altitude;

    mavros_msgs::CommandTOL landing_cmd;
    landing_cmd.request.altitude = home_alt;
    landing_cmd.request.longitude = curr_lon;
    landing_cmd.request.latitude = curr_lat;
    while (!(land_client.call(landing_cmd)) &&
           landing_cmd.response.success)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Vehicle landing...");
    return;
}

void taskmaster::uavStateCallBack(const mavros_msgs::State::ConstPtr &msg)
{
    uav_current_state = *msg;
}

void taskmaster::uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_pose = *msg;
}

bool taskmaster::set_offboard()
{
    ros::Rate rate(20.0);
    ros::Time last_request = ros::Time::now();
    mavros_msgs::PositionTarget init_sp;

    // To be converted to NED in setpoint_raw plugin
    init_sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; 

    init_sp.position.x = uav_pose.pose.position.x;
    init_sp.position.y = uav_pose.pose.position.y;
    init_sp.position.z = uav_pose.pose.position.z;
    init_sp.velocity.x = 0;
    init_sp.velocity.y = 0;
    init_sp.velocity.z = 0;
    init_sp.acceleration_or_force.x = 0;
    init_sp.acceleration_or_force.y = 0;
    init_sp.acceleration_or_force.z = 0;
    init_sp.yaw = 0;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        Position_Setpoint_Pub.publish(init_sp);
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
            (ros::Time::now() - last_request > ros::Duration(1.0)))
        {
            ROS_INFO("Try set offboard");
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            is_mode_ready = (uav_current_state.mode == "OFFBOARD");
        }
        Position_Setpoint_Pub.publish(init_sp);
        ros::spinOnce();
        rate.sleep();
    }

    if (is_mode_ready)
    {
        ROS_INFO("Offboard mode activated!");
    }

    return is_mode_ready;
}

void taskmaster::cmd_cb(const std_msgs::Byte::ConstPtr &msg)
{
    int cmd = msg->data;
    ROS_INFO("user cmd received");
    switch (cmd)
    {
    case TAKEOFF:
    {
        if (!uav_current_state.armed)
        {
            ROS_ERROR("Vehicle is not armed, please ask safety pilot to arm the vehicle.");
            break;
        }
        ROS_INFO("TAKEOFF command received!");
        uavTask = kTakeOff;
        takeoff_height = uav_pose.pose.position.z + 1.0;
        takeoff_x = uav_pose.pose.position.x;
        takeoff_y = uav_pose.pose.position.y;
        if (set_offboard())
        {
            ROS_INFO("offboard mode activated going to run takeoff");
            mission_timer.start();
            ROS_INFO("Mission timer started!");
            takeoff_flag = true;
        }
        break;
    }

    case MISSION:
    {
        if (!takeoff_flag)
        {
            ROS_ERROR("Vehilce has not taken off, please issue takeoff command first.");
            break;
        }
        ROS_INFO("command received!");
        ROS_INFO("Loading Trajectory...");
        if (loadTrajectory())
        {

            ROS_INFO("trajectory is loaded.");
            uavTask = kMission;
        }
        else
        {
            break;
        }

        break;
    }

    case LAND:
    {
        ROS_INFO("command received!");
        uavTask = kLand;
        land();
        ROS_INFO("UAV is landing");
        takeoff_flag = false;
        break;
    }

    default:
        break;
    }
}

bool taskmaster::loadTrajectory()
{
    std::ifstream infile(trajectory_location.c_str());
    std::string line;
    clearTrajectory();
    bool succeed = false;
    ROS_INFO("Loading from trajectory reference file");

    std::ifstream f(trajectory_location.c_str());
    if (!f.good())
    {
        ROS_ERROR("Wrong file name!");
        return false;
    }

    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        double _1, _2, _3, _4, _5, _6, _7, _8, _9;
        if (!(iss >> _1 >> _2 >> _3 >> _4 >> _5 >> _6 >> _7 >> _8 >>
              _9))
        {
            ROS_ERROR("The data size is not correct!");
            return false;
        } // error
        common_msgs::state p;
        p.pos.x = _1;
        p.pos.y = _2;
        p.pos.z = _3;
        p.vel.x = _4;
        p.vel.y = _5;
        p.vel.z = _6;
        p.acc.x = _7;
        p.acc.y = _8;
        p.acc.z = _9;
        _traj_list.push_back(p);
        std::cout << "Position : " << _traj_list[_points_id].pos.x << " " << _traj_list[_points_id].pos.y << " " << _traj_list[_points_id].pos.z << " "
                  << "Velocity : " << _traj_list[_points_id].vel.x << " " << _traj_list[_points_id].vel.y << " " << _traj_list[_points_id].vel.z << " "
                  << "Accel : " << _traj_list[_points_id].acc.x << " " << _traj_list[_points_id].acc.y << " " << _traj_list[_points_id].acc.z << " "
                  << std::endl;

        _points_id++;
    }
    printf("size of the list is %lu\n", _traj_list.size());
    ROS_INFO("trajctory reference has been loaded.");
    succeed = true;
    _points_id = 0;
    return true;
}

void taskmaster::clearTrajectory()
{
    _traj_list.clear();
    _points_id = 0;
    ROS_INFO("Clear Mission");
}