/*
 * helper.h
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2022 Matthew (matthewoots at gmail.com)
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

/*
* With help from 
* https://github.com/swadhagupta/RRT/blob/master/rrt.cpp
* https://pcl.readthedocs.io/projects/tutorials/en/latest/kdtree_search.html
*/
#ifndef HELPER_H
#define HELPER_H

#include <bspline.h>

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>

#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace Eigen;
using namespace std;

#define dmax std::numeric_limits<double>::max();

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

namespace helper
{


// *** Helper functions ***

Vector3d rotate_vector(Vector3d rotation, Vector3d translation)
{
    // https://github.com/felipepolido/EigenExamples
    // for affine3d examples
    double deg2rad = - 1.0 / 180.0 * M_PI;

    Quaterniond q;
    q = AngleAxisd(rotation.x() * deg2rad, Vector3d::UnitX())
        * AngleAxisd(rotation.y() * deg2rad, Vector3d::UnitY())
        * AngleAxisd(rotation.z() * deg2rad, Vector3d::UnitZ());
    
    // w,x,y,z
    Eigen::Quaterniond rot(q.w(), q.x(), q.y(), q.z());
    rot.normalize();

    Eigen::Quaterniond p;
    p.w() = 0;
    p.vec() = - translation;
    Eigen::Quaterniond rotatedP = rot * p * rot.inverse(); 
    
    return rotatedP.vec();
}

Vector3d euler_rpy(Matrix3d R)
{
    Vector3d euler_out;
    // Each vector is a row of the matrix
    Vector3d m_el[3];
    m_el[0] = Vector3d(R(0,0), R(0,1), R(0,2));
    m_el[1] = Vector3d(R(1,0), R(1,1), R(1,2));
    m_el[2] = Vector3d(R(2,0), R(2,1), R(2,2));

    // Check that pitch is not at a singularity
    if (abs(m_el[2].x()) >= 1)
    {
        euler_out.z() = 0;

        // From difference of angles formula
        double delta = atan2(m_el[2].y(),m_el[2].z());
        if (m_el[2].x() < 0)  //gimbal locked down
        {
            euler_out.y() = M_PI / 2.0;
            euler_out.x() = delta;
        }
        else // gimbal locked up
        {
            euler_out.y() = -M_PI / 2.0;
            euler_out.x() = delta;
        }
    }
    else
    {
        euler_out.y() = - asin(m_el[2].x());

        euler_out.x() = atan2(m_el[2].y()/cos(euler_out.y()), 
            m_el[2].z()/cos(euler_out.y()));

        euler_out.z() = atan2(m_el[1].x()/cos(euler_out.y()), 
            m_el[0].x()/cos(euler_out.y()));
    }

    return euler_out;
}

/* 
* @brief Transform pose according to the translation and rpy given
*/
// geometry_msgs::Point transform_point(geometry_msgs::Point _p,
//     Vector3d _rpy, Vector3d _translation)
// {
//     geometry_msgs::Point point;

//     geometry_msgs::TransformStamped transform;
//     geometry_msgs::Quaternion q; geometry_msgs::Vector3 t;
//     tf2::Quaternion quat_tf;

//     t.x = _translation.x(); t.y = _translation.y(); t.z = _translation.z(); 

//     double deg2rad = 1.0 / 180.0 * M_PI;

//     quat_tf.setRPY(_rpy.x() * deg2rad, 
//         _rpy.y() * deg2rad, 
//         _rpy.z() * deg2rad); // Create this quaternion from roll/pitch/yaw (in radians)
//     q = tf2::toMsg(quat_tf);

//     transform.transform.translation = t;
//     transform.transform.rotation = q;
//     transform.child_frame_id = "/base";
//     transform.header.frame_id = "/map";

//     tf2::doTransform(_p, point, transform);

//     return point;
// }

geometry_msgs::Point vector_to_point(Vector3d v)
{
    geometry_msgs::Point tmp;
    tmp.x = v.x(); 
    tmp.y = v.y(); 
    tmp.z = v.z();

    return tmp;
}

Vector3d point_to_vector(geometry_msgs::Point p)
{
    Vector3d tmp;
    tmp.x() = p.x; 
    tmp.y() = p.y; 
    tmp.z() = p.z;

    return tmp;
}

geometry_msgs::Quaternion quaternion_to_orientation(Quaterniond q)
{
    geometry_msgs::Quaternion tmp;
    tmp.x = q.x(); 
    tmp.y = q.y(); 
    tmp.z = q.z();
    tmp.w = q.w();

    return tmp;
}

Quaterniond orientation_to_quaternion(geometry_msgs::Quaternion q)
{
    Quaterniond tmp;
    tmp.x() = q.x; 
    tmp.y() = q.y; 
    tmp.z() = q.z;
    tmp.w() = q.w;

    return tmp;
}

double constrain_between_180(double x)
{
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

double constrain_between_90(double x)
{
    x = fmod(x + M_PI_2,M_PI);
    if (x < 0)
        x += M_PI;
    return x - M_PI_2;
}

Affine3d posestamped_to_affine(geometry_msgs::PoseStamped ps)
{
    Affine3d output = Affine3d::Identity();
    Quaterniond orientation = Quaterniond(ps.pose.orientation.w,
        ps.pose.orientation.x,
        ps.pose.orientation.y,
        ps.pose.orientation.z
    );
    Vector3d position = Vector3d(ps.pose.position.x,
        ps.pose.position.y,
        ps.pose.position.z
    );
    output.translation() = position;
    output.linear() = orientation.toRotationMatrix();

    return output;
}

Quaterniond enu_to_nwu() {return Quaterniond(0.7073883, 0, 0, 0.7068252);}


VectorXd linspace(double min, double max, double n)
{
    VectorXd linspaced((int)n);
    double delta = (max - min) / (n - 1.0);
    linspaced(0) = min;
    
    for (int i = 1; i < (int)n; i++)
    {
        linspaced(i) = (linspaced(i-1) + delta);
    }
    return linspaced;
}

/* 
* @brief Stretch/Scale operation on point cloud
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr 
    pcl_z_scale(pcl::PointCloud<pcl::PointXYZ>::Ptr input, 
    double z)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0 ; i < input->points.size(); i++)
    {
        pcl::PointXYZ pt = input->points[i];
        output->points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z * z));
    }
    
    return output;
}

/* 
* @brief Filter point cloud with the dimensions given
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr 
    pcl_ptr_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, 
    Vector3d centroid, Vector3d dimension)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    
    Vector3d min = centroid - dimension/2;
    Vector3d max = centroid + dimension/2;

    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(Eigen::Vector4f(min.x(), min.y(), min.z(), 1.0));
    box_filter.setMax(Eigen::Vector4f(max.x(), max.y(), max.z(), 1.0));

    box_filter.setInputCloud(input);
    box_filter.filter(*output);

    // printf("%s[rrtstar.h] return fromPCLPointCloud2! \n", KBLU);
    return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr base_to_transform_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr _pc, Vector3d rotation, Vector3d translation)
{
    // https://github.com/felipepolido/EigenExamples
    // for affine3d examples
    geometry_msgs::Quaternion q;
    tf2::Quaternion quat_tf;
    double deg2rad = - 1.0 / 180.0 * M_PI;

    quat_tf.setRPY(rotation.x() * deg2rad, 
        rotation.y() * deg2rad, 
        rotation.z() * deg2rad); // Create this quaternion from roll/pitch/yaw (in radians)
    q = tf2::toMsg(quat_tf);
    
    // w,x,y,z
    Eigen::Quaterniond rot(q.w, q.x, q.y, q.z);
    rot.normalize();

    Eigen::Quaterniond p;
    p.w() = 0;
    p.vec() = - translation;
    Eigen::Quaterniond rotatedP = rot * p * rot.inverse(); 
    Eigen::Vector3d rotatedV = rotatedP.vec();

    Eigen::Affine3d aff_t = Eigen::Affine3d::Identity();
    Eigen::Affine3d aff_r = Eigen::Affine3d::Identity();
    // aff_t.translation() = - translation;
    aff_r.translation() = rotatedV;
    aff_r.linear() = rot.toRotationMatrix();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pcl_trans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pcl_rot(new pcl::PointCloud<pcl::PointXYZ>);
    // transformPointCloud(*_pc, *tmp_pcl_trans, aff_t, true);
    transformPointCloud(*_pc, *tmp_pcl_rot, aff_r, true);

    return tmp_pcl_rot;
}

/** 
* @brief Convert point cloud from ROS sensor message to 
* pcl point ptr
**/
pcl::PointCloud<pcl::PointXYZ>::Ptr 
    pcl2_converter(sensor_msgs::PointCloud2 _pc)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(_pc, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromPCLPointCloud2(pcl_pc2, *tmp_cloud);
    
    return tmp_cloud;
}

/** 
* @brief Transform sensor cloud according to the translation and rpy given
**/
sensor_msgs::PointCloud2 transform_sensor_cloud(sensor_msgs::PointCloud2 _pc,
    Vector3d _rpy, Vector3d _translation)
{
    sensor_msgs::PointCloud2 transformed_cloud;

    geometry_msgs::TransformStamped transform;
    geometry_msgs::Quaternion q; geometry_msgs::Vector3 t;
    tf2::Quaternion quat_tf;

    t.x = _translation.x(); t.y = _translation.y(); t.z = _translation.z(); 

    double deg2rad = 1.0 / 180.0 * M_PI;

    quat_tf.setRPY(_rpy.x() * deg2rad, 
        _rpy.y() * deg2rad, 
        _rpy.z() * deg2rad); // Create this quaternion from roll/pitch/yaw (in radians)
    q = tf2::toMsg(quat_tf);

    transform.transform.translation = t;
    transform.transform.rotation = q;
    transform.child_frame_id = "/base";
    transform.header.frame_id = "/map";

    tf2::doTransform(_pc, transformed_cloud, transform);

    return transformed_cloud;
}

/* 
* @brief Forward transform pose according to the translation and rpy given
*/
geometry_msgs::Point forward_transform_point(geometry_msgs::Point _p,
    Vector3d _rpy, Vector3d _translation)
{
    // geometry_msgs::Point tmp = transform_point(_p,
    //     - Vector3d(0, 0, 0), - _translation);
    // tmp = transform_point(tmp,
    //     - Vector3d(0, 0, _rpy.z()), Vector3d(0, 0, 0));

    geometry_msgs::Point tmp;

    double roll = 0, pitch = 0, yaw = -_rpy.z() / 180.0 * M_PI;    
    Quaterniond q;
    q = AngleAxisd(roll, Vector3d::UnitX())
        * AngleAxisd(pitch, Vector3d::UnitY())
        * AngleAxisd(yaw, Vector3d::UnitZ());

    Eigen::Quaterniond p;
    p.w() = 0;
    p.vec() = Vector3d(_p.x, _p.y, _p.z) - _translation;
    Eigen::Quaterniond rotatedP = q * p * q.inverse(); 

    tmp = vector_to_point(rotatedP.vec());

    return tmp;
}

/* 
* @brief Backward transform pose according to the translation and rpy given
*/
geometry_msgs::Point backward_transform_point(geometry_msgs::Point _p,
    Vector3d _rpy, Vector3d _translation)
{
    // geometry_msgs::Point tmp = transform_point(_p,
    //     - Vector3d(0, 0, - _rpy.z()), Vector3d(0, 0, 0));
    // tmp = transform_point(tmp, - Vector3d(0, 0, 0), _translation);

    geometry_msgs::Point tmp;
    double roll = 0, pitch = 0, yaw = _rpy.z() / 180.0 * M_PI;    
    Quaterniond q;
    q = AngleAxisd(roll, Vector3d::UnitX())
        * AngleAxisd(pitch, Vector3d::UnitY())
        * AngleAxisd(yaw, Vector3d::UnitZ());

    Eigen::Quaterniond p;
    p.w() = 0;
    p.vec() = Vector3d(_p.x, _p.y, _p.z);
    Eigen::Quaterniond rotatedP = q * p * q.inverse(); 

    tmp = vector_to_point(rotatedP.vec() + _translation);

    return tmp;
}

MatrixXd set_clamped_path(MatrixXd wp, 
        double max_vel, double _knot_span, int _order, 
        Vector3d start_pose) 
{
    bs::bspline _bsp;
    /* Uniform Distribution */
    MatrixXd cp_raw = MatrixXd::Zero(3,1); VectorXd time_waypoint = VectorXd::Zero(1);

    _bsp.UniformDistribution(start_pose, wp, max_vel, _knot_span, 
        &time_waypoint, &cp_raw);

    /* Clamp the Bspline */
    // Update global control points and start and end time 
    MatrixXd _global_cp = _bsp.ClampBspline(_order, cp_raw);

    return _global_cp;
}


VectorXd set_knots_path(MatrixXd _global_cp, double   _knot_span, int _order)
{
    double _start = ros::Time::now().toSec();
    double _end = _start + ((_global_cp.cols() - (_order)) * _knot_span);
    VectorXd _fixed_knots_tmp = linspace(_start, _end, (double)(_global_cp.cols() - (_order-1)));

    return _fixed_knots_tmp;
}


/** 
* @brief Update Full Path
*/
std::vector<Vector3d> update_full_path(MatrixXd _global_cp, int _knotdiv, int _order, VectorXd _fixed_knots)
{
    bs::bspline _bsp;
    // Reset pos, vel, acc and time
    MatrixXd _pos = MatrixXd::Zero(3,1); 
    MatrixXd _vel = MatrixXd::Zero(3,1); 
    MatrixXd _acc = MatrixXd::Zero(3,1); 
    VectorXd _time = VectorXd::Zero(1);

    // Truncate control points from global to local
    MatrixXd _local_cp = MatrixXd::Zero(3,_global_cp.cols());

    for(int i = 0; i < _local_cp.cols(); i++)
    {
        _local_cp.col(i) = _global_cp.col(i);
    }

    double start = _fixed_knots(0); 
    double end = _fixed_knots(_fixed_knots.size()-1);

    // Bspline Creation using Function
    _bsp.GetBspline3(_order, _local_cp, start, end, _knotdiv, &_pos, &_vel, &_acc, &_time);

    std::cout << KYEL << "[common_utils.h] " << "Full Path Update Complete" << KNRM << std::endl;

    int pos_size = _pos.cols();
    std::vector<Vector3d> tmp;
    for (int i = 0; i < pos_size; i++)
    {
        Vector3d tmp_v;
        tmp_v.x() = _pos.col(i).x();
        tmp_v.y() = _pos.col(i).y();
        tmp_v.z() = _pos.col(i).z();

        tmp.push_back(tmp_v);
    }

    return tmp;
}

bool kdtree_collide_pcl(Vector3d point, pcl::PointCloud<pcl::PointXYZ>::Ptr _obs,
    double c)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud(_obs);

    pcl::PointXYZ searchPoint;
    searchPoint.x = point.x();
    searchPoint.y = point.y();
    searchPoint.z = point.z();

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

    float radius = (float)c;

    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        {
            return true;
        }
    }

    return false;
}

vector<Vector3d> kdtree_find_points_pcl(Vector3d point, pcl::PointCloud<pcl::PointXYZ>::Ptr _obs,
    double c, int K)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    vector<Vector3d> points;

    kdtree.setInputCloud(_obs);

    // Maybe we need to check to see any number here is NA, or inf

    pcl::PointXYZ searchPoint;
    searchPoint.x = point.x();
    searchPoint.y = point.y();
    searchPoint.z = point.z();

    // if (isnan(searchPoint.x) ||
    //     isnan(searchPoint.y) ||
    //     isnan(searchPoint.z) ||
    //     isinf(searchPoint.x) ||
    //     isinf(searchPoint.y) ||
    //     isinf(searchPoint.z) )
    // {
    //     printf("[kdtree_pcl] Found a point with inf or nan!");
    //     std::cout << searchPoint.x << "," << 
    //     searchPoint.y << "," << searchPoint.z << std::endl;
    //     return points;
    // }

    // K nearest neighbor search

    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);

    // float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

    float radius = (float)c;

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
    {
        // for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i)
        {
            Vector3d kd_point;
            // When the point is larger than the radius, we do not consider it
            if (pointKNNSquaredDistance[i] - pow(radius,2) > 0)
                continue;
            kd_point.x() = (*_obs)[ pointIdxKNNSearch[i] ].x; 
            kd_point.y() = (*_obs)[ pointIdxKNNSearch[i] ].y;
            kd_point.z() = (*_obs)[ pointIdxKNNSearch[i] ].z;
            points.push_back(kd_point);
        }
    }

    return points;
}

// *** End Helper functions ***
   
}

#endif