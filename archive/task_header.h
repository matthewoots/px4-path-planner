#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandTOL.h> 
#include <common_msgs/state.h>
#include <common_msgs/target.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/HomePosition.h>
#include <std_msgs/Byte.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
// #define ENGINE0 0
// #define TAKEOFF 1
#define TAKEOFF 1
#define MISSION 2
#define LAND 5


enum UavTaskState
{
    kIdle,
    kReady,
    kTakeOff,
    kHover,
    kMission,
    kSwarm,
    kLand,
};
// #define MISSION 2
// #define HOVER 3
// #define LAND 4
// #define POS_CONTROL 0
// #define VEL_CONTROL 1

// enum VehicleTask
// {
//     kIdle,
//     kReady,
//     kTakeOff,
//     kHover,
//     kMission,
//     kLand,
// };

class taskmaster
{
public:
    taskmaster(ros::NodeHandle &nodeHandle);
    ~taskmaster();

    bool loadTrajectory();
    void clearTrajectory();
    void pubTrajectory();
    //void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void cmd_cb(const std_msgs::Byte::ConstPtr &msg);
    void uavStateCallBack(const mavros_msgs::State::ConstPtr &msg);

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    //void missionTimerCallBack(const ros::TimerEvent &);

    bool set_offboard();
    void initialisation();
    
    void land();

    void missionTimer(const ros::TimerEvent &);

    void gpsCurrentCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void gpsHomeCallback(const mavros_msgs::HomePosition::ConstPtr &msg);

    // void calculateYaw(const mavros_msgs::PositionTarget::ConstPtr &msg);
    

private:
    ros::NodeHandle _nh;
    //VehicleTask _vehicle_task;
    ros::Subscriber uav_state_sub;
    ros::Subscriber cmd_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient land_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient takeoff_client;
    ros::Publisher Position_Setpoint_Pub;
    ros::Subscriber uav_pose_sub;
    ros::Subscriber uav_gps_cur_sub;
    ros::Subscriber uav_gps_home_sub;

    geometry_msgs::Point pos_init;

    UavTaskState uavTask;

    std::vector<common_msgs::state> _traj_list;
    std::string trajectory_location;
    mavros_msgs::State uav_current_state;
    
    // uav_pose in enu frame
    geometry_msgs::PoseStamped uav_pose;

    int _points_id;
    double _interval;
    double takeoff_height;
    double takeoff_x;
    double takeoff_y;
    ros::Timer mission_timer;

    sensor_msgs::NavSatFix uav_gps_cur;
    mavros_msgs::HomePosition uav_gps_home;

    bool takeoff_flag; // this flag is set if the drone has took off
    bool takeoff_announced;

    // Initialisation
    bool _initialised;
};
