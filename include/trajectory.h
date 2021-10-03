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

    bool setTrajectory(string file_location, double interval)
    {
        _file_location = file_location;
        _interval = interval;
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
            printf("%s  [trajectory.h] Row[%d] %.3lf, %.3lf, %.3lf, %.3lf \n", KBLU, rowcount, time, xpos, ypos, zpos);
            // Create waypoints
            t.push_back(time); // must be increasing 
            x.push_back(xpos); y.push_back(ypos); z.push_back(zpos);
        }            
        duration = t.back();
        tk::spline sx(t,x); tk::spline sy(t,y); tk::spline sz(t,z);
        double testing_time = 0.91;

        // evaluate spline print
        printf("%s[trajectory.h] Example trajectory at %lf %lf %lf \n", KGRN, 
            sx(testing_time),
            sy(testing_time),
            sz(testing_time));

        MatrixXd m(3,5);
        m.resize(3,2);
        m(0,0) = 3;
        m(1,0) = 2.5;
        m(2,0) = m(1,0) + m(0,1);
        m(0,1) = -1;
        m(1,1) = m(1,0) + m(0,1);   
        m(2,1) = m(1,0) + m(0,1);
        printf("%s[trajectory.h] Testing Eigen MatrixXd resizing, size (%ld,%ld) with %ld elements\n", KYEL, m.rows() ,m.cols(), m.size());

        // Start to populate spline trajectory about the desired interval
        maxidx = ceil(duration/_interval)+1;
        printf("%s[trajectory.h] With duration of %lf, interval size %lf, number of partitions %d \n", KGRN, duration, _interval, maxidx);

        _traj.resize(4,maxidx);
        _traj.resize(4,maxidx);
        for (int i=0; i<maxidx; i++)
        {
            // time
            _traj(0,i) = i * _interval;
            // x position
            _traj(1,i) = sx(i * _interval);
            //y position
            _traj(2,i) = sy(i * _interval);
            //z position
            _traj(3,i) = sz(i * _interval);
            // evaluate spline print
            printf("%s  [trajectory.h] [%.2lf] [%.2lf %.2lf %.2lf] \n", KGRN, 
                _traj(0,i),
                _traj(1,i),
                _traj(2,i),
                _traj(3,i));
        }

        initialised = true;
        return true;
    }

    MatrixXd returnTrajectory() {return _traj;}

    Vector3d returnDesiredPos(double milli_time) 
    {
        double time_sec = 0.0;
        int idx = 0;
        Vector3d desired_pos;
        for (int i=0; i<maxidx; i++)
        {
            if (milli_time/1000 >= _traj(0,i))
            {
                time_sec = _traj(0,i);
                idx = i;
                break;
            }
        }
        desired_pos.x() = _traj(1,idx);
        desired_pos.y() = _traj(2,idx);
        desired_pos.z() = _traj(3,idx);
        return desired_pos;
    }

    void clearTrajectory();

private:
    bool initialised;
    string _file_location;
    double _interval;
    MatrixXd _traj;
    int maxidx;
    double duration;
};
