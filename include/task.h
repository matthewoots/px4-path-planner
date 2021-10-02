#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>

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

#include <tf/tf.h>

using namespace Eigen;
using namespace std;

#define IDLE 0
#define TAKEOFF 1
#define HOVER 2
#define MISSION 3
#define LAND 4

enum VehicleTask
{
    kIdle,
    kTakeOff,
    kHover,
    kMission,
    kLand
};

class taskmaster
{
public:
    taskmaster(ros::NodeHandle &nodeHandle);
    ~taskmaster();

    bool loadTrajectory();
    void clearTrajectory();
    void pubTrajectory();    

    bool set_offboard();
    void initialisation();
    
    void land();

    void missionTimer(const ros::TimerEvent &);

    void uavCommandCallBack(const std_msgs::Byte::ConstPtr &msg);
    void gpsHomeCallback(const mavros_msgs::HomePosition::ConstPtr &msg)
    {
        uav_gps_home = *msg;
    }
    void gpsCurrentCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        uav_gps_cur = *msg;
    }
    void uavStateCallBack(const mavros_msgs::State::ConstPtr &msg)
    {
        uav_current_state = *msg;
        // Initialise state at the beginning
        if (!_initialised)
            taskmaster::initialisation();
    }

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        uav_pose = *msg;
        tf::Quaternion q(
            uav_pose.pose.orientation.x, uav_pose.pose.orientation.y,
            uav_pose.pose.orientation.z, uav_pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(curr_roll, curr_pitch, curr_yaw);
    }

private:
    ros::NodeHandle _nh;

    ros::Subscriber state_sub;
    ros::Subscriber uav_gps_cur_sub;
    ros::Subscriber uav_gps_home_sub;
    ros::Subscriber uav_pose_sub;
    ros::Subscriber uav_cmd_sub;

    ros::Publisher local_pos_pub; 

    ros::ServiceClient arming_client; 
    ros::ServiceClient set_mode_client; 

    ros::Timer mission_timer;

    // Initialisation
    bool _initialised;
    double _interval;
    double _takeoff_height;

    int retry = 0;
    int uav_task;
    int uav_prev_task;
    bool takeoff_flag;
    bool task_complete;
    double curr_roll, curr_pitch, curr_yaw;

    ros::Time last_request_timer;

    mavros_msgs::CommandBool arm_cmd;

    Vector3d takeoff_pos;

    mavros_msgs::HomePosition uav_gps_home;
    sensor_msgs::NavSatFix uav_gps_cur;
    mavros_msgs::State uav_current_state;
    geometry_msgs::PoseStamped uav_pose;

    geometry_msgs::PoseStamped home;
};
