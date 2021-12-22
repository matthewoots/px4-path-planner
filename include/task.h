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
#include <common_msgs/state.h>
#include <common_msgs/target.h>
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
public:
    taskmaster(ros::NodeHandle &nodeHandle);
    ~taskmaster();

    bool set_offboard();
    void initialisation();

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
        m.getRPY(curr_roll, curr_pitch, curr_yaw);
    }

    /** @brief Send Desired Command to PX4 via Mavros (Using Vector3d)*/
    void uavDesiredControlHandler(Vector3d command_pose, Vector3d command_vel)
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
            // For the type mask we have to ignore the rest (3520)
            // 64	POSITION_TARGET_TYPEMASK_AX_IGNORE	Ignore acceleration x
            // 128	POSITION_TARGET_TYPEMASK_AY_IGNORE	Ignore acceleration y
            // 256	POSITION_TARGET_TYPEMASK_AZ_IGNORE	Ignore acceleration z
            // 1024	POSITION_TARGET_TYPEMASK_YAW_IGNORE	Ignore yaw
            // 2048	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE	Ignore yaw rate
            pos_sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            pos_sp.type_mask = 3520;
            local_pos_raw_pub.publish(pos_sp);
        }
    }

    bool UnpackWaypoint(MatrixXd *_waypoint, string _file_location);

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

    // Initialisation
    bool _initialised;
    bool _setpoint_raw_mode;
    bool takeoff_flag;
    bool task_complete;

    double _send_desired_interval;
    double _trajectory_pub_rate;
    double _takeoff_height;
    double _common_max_vel;
    double _common_min_vel;
    double curr_roll, curr_pitch, curr_yaw;
    
    /** @brief Used for mission timer, when to replan the trajectory */ 
    double mission_start_time;
    double prev_traj_replan;

    int retry = 0;
    int uav_task;
    int uav_prev_task;
    int _unique_id_range;
    int _order;
    int _control_points_division;
    vector<int> uav_id;
    
    std::string _wp_file_location;

    double last_request_timer;

    mavros_msgs::CommandBool arm_cmd;

    Vector3d takeoff_pos;
    Vector3d current_pos;
    Vector3d current_vel;
    Vector3d last_mission_pos;

    mavros_msgs::HomePosition uav_gps_home;
    sensor_msgs::NavSatFix uav_gps_cur;
    mavros_msgs::State uav_current_state;
    geometry_msgs::PoseStamped uav_pose;
    geometry_msgs::TwistStamped uav_vel;

    geometry_msgs::PoseStamped home;

    trajectory traj;
    MatrixXd wp;
};
