#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <spline.h>
#include <csv.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;

typedef std::pair<double, Eigen::Vector3d> Waypoint;
typedef std::vector<Waypoint> T_Waypoint;

class trajectory
{
public:
    // trajectory();
    // ~trajectory();

    bool setTrajectory(string file_location, double interval, double cp_interval, Vector3d current_vel, Vector3d final_vel)
    {
        _completed = false;
        _file_location = file_location;
        _interval = interval;
        _cp_interval = cp_interval;
        T_Waypoint waypoints;
        std::vector<double> t; std::vector<double> x; std::vector<double> y; std::vector<double> z;
        printf("%s[trajectory.h] Trying to open %s \n", KYEL, _file_location.c_str());
        ifstream file(_file_location);
        
        if (!file)
        {
            printf("%s[trajectory.h] File not present! \n", KRED);
            return false;
        }
        printf("%s[trajectory.h] Success, found %s \n", KGRN, _file_location.c_str());

        io::CSVReader<4> in(_file_location);
        in.read_header(io::ignore_extra_column, "time", "xpos", "ypos", "zpos");
        double time; double xpos; double ypos; double zpos;
        int rowcount = 0;
        while (in.read_row(time, xpos, ypos, zpos)){
            // do stuff with the data
            rowcount ++;
            if (debug_mode_on)
            {
                printf("%s  [trajectory.h] Waypoint[%d] %.3lf, %.3lf, %.3lf, %.3lf \n", KBLU, rowcount, time, xpos, ypos, zpos);
            }
            // Create waypoints
            t.push_back(time); // must be increasing 
            x.push_back(xpos); y.push_back(ypos); z.push_back(zpos);
        }            
        duration = t.back();

        tk::spline sx(t,x,tk::spline::cspline,false,
		    tk::spline::first_deriv,current_vel.x(),
	       	tk::spline::first_deriv,final_vel.x());
        tk::spline sy(t,y,tk::spline::cspline,false,
		    tk::spline::first_deriv,current_vel.y(),
	       	tk::spline::first_deriv,final_vel.y());
        tk::spline sz(t,z,tk::spline::cspline,false,
		    tk::spline::first_deriv,current_vel.z(),
	       	tk::spline::first_deriv,final_vel.z());
        double testing_time = 0.91;

        // evaluate spline print
        if (debug_mode_on)
        {
            printf("%s[trajectory.h] Example trajectory at %lf %lf %lf \n", KGRN, 
                sx(testing_time),
                sy(testing_time),
                sz(testing_time));

            MatrixXd m(7,2);
            m.resize(3,2);
            m(0,0) = 3;
            m(1,0) = 2.5;
            m(2,0) = m(1,0) + m(0,1);
            m(0,1) = -1;
            m(1,1) = m(1,0) + m(0,1);   
            m(2,1) = m(1,0) + m(0,1);
            printf("%s[trajectory.h] Testing Eigen MatrixXd resizing, size (%ld,%ld) with %ld elements\n", KYEL, m.rows() ,m.cols(), m.size());
        }

        // Start to populate spline trajectory about the desired interval
        maxidx = ceil(duration/_interval)+1;
        printf("%s[trajectory.h] With duration of %lf, interval size %lf, number of partitions %d \n", KGRN, duration, _interval, maxidx);

        _traj.resize(7,maxidx);

        for (int i=0; i<maxidx; i++)
        {
            // time
            _traj(0,i) = i * _interval;
            // x position
            _traj(1,i) = sx(i * _interval);
            // y position
            _traj(2,i) = sy(i * _interval);
            // z position
            _traj(3,i) = sz(i * _interval);
            // vx velocity
            _traj(4,i) = sx.deriv(1, i * _interval);
            // vy velocity
            _traj(5,i) = sy.deriv(1, i * _interval);
            // vz velocity
            _traj(6,i) = sz.deriv(1, i * _interval);

            // evaluate spline print
            if (debug_mode_on)
            {
                printf("%s  [trajectory.h] [%.2lf] [%.2lf %.2lf %.2lf] [%.2lf %.2lf %.2lf] \n", KGRN, 
                    _traj(0,i), _traj(1,i), _traj(2,i), _traj(3,i),
                    _traj(4,i), _traj(5,i), _traj(6,i));
            }
        }

        double max_cp_idx = ceil(duration/_interval)+1;
        // We will initialise the first control points
        for (int i=0; i<max_cp_idx; i++)
        {
            t_cp.push_back(i * _cp_interval);
            x_cp.push_back(sx(i * _cp_interval)); 
            y_cp.push_back(sy(i * _cp_interval)); 
            z_cp.push_back(sz(i * _cp_interval));
        }

        initialised = true;
        return true;
    }

    void setSimpleTrajectory(Vector3d start_pose, Vector3d end_pose, double interval, double suggested_max_vel)
    {
        _completed = false;
        _interval = interval;
        Vector3d pose_diff = start_pose - end_pose;
        double sqrt_diff = sqrt(pow(pose_diff.x(),2) + pow(pose_diff.y(),2) + pow(pose_diff.z(),2));
        double suggested_time = sqrt_diff / suggested_max_vel;

        // Size must be bigger or equal to 3 points
        std::vector<double> t = {0, suggested_time/2, suggested_time};
        std::vector<double> x = {start_pose.x(), (start_pose.x()+ end_pose.x())/2, end_pose.x()};
        std::vector<double> y = {start_pose.y(), (start_pose.y()+ end_pose.y())/2, end_pose.y()};
        std::vector<double> z = {start_pose.z(), (start_pose.z()+ end_pose.z())/2, end_pose.z()};

        tk::spline sx(t,x,tk::spline::cspline,false,
		    tk::spline::first_deriv,0,
	       	tk::spline::first_deriv,0);
        tk::spline sy(t,y,tk::spline::cspline,false,
		    tk::spline::first_deriv,0,
	       	tk::spline::first_deriv,0);
        tk::spline sz(t,z,tk::spline::cspline,false,
		    tk::spline::first_deriv,0,
	       	tk::spline::first_deriv,0);
        
        // Start to populate spline trajectory about the desired interval
        maxidx = ceil(suggested_time/_interval)+1;
        _traj.resize(7,maxidx);

        for (int i=0; i<maxidx; i++)
        {
            // time
            _traj(0,i) = i * _interval;
            // x position
            _traj(1,i) = sx(i * _interval);
            // y position
            _traj(2,i) = sy(i * _interval);
            // z position
            _traj(3,i) = sz(i * _interval);
            // vx velocity
            _traj(4,i) = sx.deriv(1, i * _interval);
            // vy velocity
            _traj(5,i) = sy.deriv(1, i * _interval);
            // vz velocity
            _traj(6,i) = sz.deriv(1, i * _interval);

            // evaluate spline print
            if (debug_mode_on)
            {
                printf("%s  [trajectory.h] [%.2lf] [%.2lf %.2lf %.2lf] [%.2lf %.2lf %.2lf] \n", KGRN, 
                    _traj(0,i), _traj(1,i), _traj(2,i), _traj(3,i),
                    _traj(4,i), _traj(5,i), _traj(6,i));
            }
        }
        printf("%s[trajectory.h] Simple Trajectory Planning Complete! \n", KGRN);
    }

    void recalculateTrajectory(Vector3d current_pos, Vector3d current_vel, Vector3d final_vel, double time) 
    {
        // Time after the start of the trajectory recalculation
        // We need to concancate the vector
        int finished_idx = (int)ceil(time / _cp_interval);
        int total_idx = t_cp.size() - finished_idx + 1;


        std::vector<double> tmp_t_cp(total_idx); std::vector<double> tmp_x_cp(total_idx); 
        std::vector<double> tmp_y_cp(total_idx); std::vector<double> tmp_z_cp(total_idx);
        
        tmp_t_cp.push_back(0.0);
        tmp_x_cp.push_back(current_pos.x());
        tmp_y_cp.push_back(current_pos.y());
        tmp_z_cp.push_back(current_pos.z());

        for (int i=finished_idx; i<t_cp.size(); i++)
        {
            tmp_t_cp.push_back(t_cp[i] - finished_idx * _cp_interval);
            tmp_x_cp.push_back(x_cp[i]);
            tmp_y_cp.push_back(y_cp[i]);
            tmp_z_cp.push_back(z_cp[i]);
        }

        // // Control points will change and will be updated with the latest position and after optimization
        // tk::spline sx(t_cp,x_cp,tk::spline::cspline,false,
		//     tk::spline::first_deriv,current_vel.x(),
	    //    	tk::spline::first_deriv,final_vel.x());
        // tk::spline sy(t_cp,y_cp,tk::spline::cspline,false,
		//     tk::spline::first_deriv,current_vel.y(),
	    //    	tk::spline::first_deriv,final_vel.y());
        // tk::spline sz(t_cp,z_cp,tk::spline::cspline,false,
		//     tk::spline::first_deriv,current_vel.z(),
	    //    	tk::spline::first_deriv,final_vel.z());
        // double testing_time = 0.91;

        //  // Start to populate spline trajectory about the desired interval
        // maxidx = ceil(duration - time/_interval)+1;
        // _traj.resize(7,maxidx);

        // for (int i=0; i<maxidx; i++)
        // {
        //     // time
        //     _traj(0,i) = i * _interval;
        //     // x position
        //     _traj(1,i) = sx(i * _interval);
        //     // y position
        //     _traj(2,i) = sy(i * _interval);
        //     // z position
        //     _traj(3,i) = sz(i * _interval);
        //     // vx velocity
        //     _traj(4,i) = sx.deriv(1, i * _interval);
        //     // vy velocity
        //     _traj(5,i) = sy.deriv(1, i * _interval);
        //     // vz velocity
        //     _traj(6,i) = sz.deriv(1, i * _interval);

        //     // evaluate spline print
        //     // printf("%s  [trajectory.h] [%.2lf] [%.2lf %.2lf %.2lf] [%.2lf %.2lf %.2lf] \n", KGRN, 
        //     //     _traj(0,i), _traj(1,i), _traj(2,i), _traj(3,i),
        //     //     _traj(4,i), _traj(5,i), _traj(6,i));
        // }

    }

    void calcDesired(double time) 
    {
        double time_sec = 0.0; int idx = 0; 
        for (int i=0; i<maxidx; i++)
        {
            if (time <= _traj(0,i))
            {
                time_sec = _traj(0,i);
                idx = i;
                break;
            }
            // Last one so we would continue to use it as the idx
            if (i == maxidx-1)
            {
                time_sec = _traj(0,maxidx-1);
                idx = maxidx-1;
                _completed = true;
                break;
            }
        }
        desired_pos.x() = _traj(1,idx);
        desired_pos.y() = _traj(2,idx);
        desired_pos.z() = _traj(3,idx);

        desired_vel.x() = _traj(4,idx);
        desired_vel.y() = _traj(5,idx);
        desired_vel.z() = _traj(6,idx);
        // evaluate spline print
        if (debug_mode_on)
        {
            printf("%s  [trajectory.h] Returning desired @ Time [%.2lf] Index [%d/%d] Completed [%s]\n", KYEL, 
                time_sec, idx, maxidx, _completed ? "true" : "false");
            printf("%s  [trajectory.h] - pos [%.2lf, %.2lf, %.2lf] vel [%.2lf, %.2lf, %.2lf] \n", KGRN, 
                desired_pos.x(), desired_pos.y(), desired_pos.z(),
                desired_vel.x(), desired_vel.y(), desired_vel.z());
        }
    }

    Vector3d returnPose() {return desired_pos;}

    Vector3d returnVel() {return desired_vel;} 

    void debug_mode(bool debug) {debug_mode_on = debug;} 

    bool isCompleted() {return _completed;}     

    void clearTrajectory();

private:
    bool initialised;
    bool _completed;
    bool debug_mode_on;
    int maxidx;
    double duration;
    double _interval;
    double _cp_interval;
    string _file_location;
    Vector3d desired_pos;
    Vector3d desired_vel;

    MatrixXd _traj;

    std::vector<double> t_cp;
    std::vector<double> x_cp; std::vector<double> y_cp; std::vector<double> z_cp;
};
