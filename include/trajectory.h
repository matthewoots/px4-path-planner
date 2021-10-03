#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <spline/helpers/effector_spline.h>
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
// loading helper class namespace
using namespace spline::helpers;
typedef std::pair<double, Eigen::Vector3d> Waypoint;
typedef std::vector<Waypoint> T_Waypoint;

class trajectory
{
public:
    // trajectory();
    // ~trajectory();

    bool setTrajectory()
    {
        T_Waypoint waypoints;
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
            printf("%s[trajectory.h] Row[%d] %.3lf, %.3lf, %.3lf, %.3lf \n", KBLU, rowcount, time, xpos, ypos, zpos);
            // Create waypoints
            waypoints.push_back(std::make_pair(time, Eigen::Vector3d(xpos,ypos,zpos)));
        }            
        duration = waypoints.back().first;

        exact_cubic_t* eff_traj = spline::helpers::effector_spline(waypoints.begin(),waypoints.end());
        double testing_time = 0.91;

        // evaluate spline print
        printf("%s[trajectory.h] Example trajectory at %lf %lf %lf \n", KGRN, 
            (*eff_traj)(testing_time)[0],
            (*eff_traj)(testing_time)[1],
            (*eff_traj)(testing_time)[2]);

        MatrixXd m(3,5);
        m.resize(2,2);
        m(0,0) = 3;
        m(1,0) = 2.5;
        m(0,1) = -1;
        m(1,1) = m(1,0) + m(0,1);
        printf("%s[trajectory.h] Testing Eigen MatrixXd resizing, size (%ld,%ld) with %ld elements\n", KYEL, m.rows() ,m.cols(), m.size());

        // Start to populate spline trajectory about the desired interval
        maxidx = ceil(waypoints.back().first/_interval);
        printf("%s[trajectory.h] With duration of %lf, interval size %lf, number of partitions %d \n", KGRN, duration, _interval, maxidx);

        return true;
    }

    void start(string file_location, double interval)
    {
        _file_location = file_location;
        _interval = interval;
        if (!setTrajectory())
            return;
    }

    void clearTrajectory();

private:
    string _file_location;
    double _interval;
    MatrixXd _traj;
    int maxidx;
    double duration;
};
