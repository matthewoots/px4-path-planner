/*
 * rrt.h
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
#ifndef RRT_H
#define RRT_H

#include "rrt_standalone.h"

#include <vector>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

class rrt_sample
{
private:
    std::vector<Vector3d> rrt_path;
    double _step_size;
    double _obs_threshold;

    double _xybuffer, _zbuffer;

    double _min_height, _max_height;
    double _passage_size;

    int _max_tries;
    double _timeout;
    double _scale_z;

    bool pcl_received;

    double yaw;
    Vector3d translation;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc1(new pcl::PointCloud<pcl::PointXYZ>);
   


public:

    std::vector<Vector3d> bspline;

    bool unpack_rrt_params(std::string _file_location)
    {
        printf("%s[rrt.h] Trying to open %s \n", KYEL, _file_location.c_str());
        ifstream file(_file_location);
        
        if (!file)
        {
            printf("%s[rrt.h] File not present! \n", KRED);
            return false;
        }
        printf("%s[rrt.h] Success, found %s \n", KGRN, _file_location.c_str());

        io::CSVReader<10> in(_file_location);
        in.read_header(io::ignore_extra_column, "step_size", 
            "obs_threshold", "xybuffer", "zbuffer", "passage_size",
            "min_height", "max_height", "max_tries", "timeout", "z_scale");
        
        double step_size;
        double obs_threshold;
        double xybuffer;
        double zbuffer;
        double passage_size;

        double min_height;
        double max_height;
        int max_tries;
        double timeout;
        double z_scale;

        // First pass is to get number of rows
        while (in.read_row(step_size, obs_threshold, xybuffer, zbuffer,
            passage_size, min_height, max_height, max_tries, timeout, z_scale))
        {
            _step_size = step_size;
            _obs_threshold = obs_threshold;
            _xybuffer = xybuffer;
            _zbuffer = zbuffer;
            _passage_size = passage_size;

            _min_height = min_height;
            _max_height = max_height;
            _max_tries = max_tries;
            _timeout = timeout;
            _scale_z = z_scale;
        }

        return true;
    } 

    bool RRT(sensor_msgs::PointCloud2 pcl_pc, 
    Vector3d start, Vector3d end, vector<VectorXd> no_fly_zone)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr original_pcl_pc = 
            pcl2_converter(pcl_pc);

        printf("%s[rrt.cpp] Start (%lf %lf %lf) End (%lf %lf %lf) \n", 
            KBLU, start.x(), start.y(), start.z(),
            end.x(), end.y(), end.z());

        // Find the origin of the transformed frame
        Vector3d _origin;
        _origin.x() = (start.x() + end.x()) / 2;
        _origin.y() = (start.y() + end.y()) / 2;
        _origin.z() = (start.z() + end.z()) / 2;
        
        // Do the preparation for transformation
        // Find the translation vector and the yaw angle
        Vector3d tmp_vect = end - start;
        yaw = atan2(tmp_vect.y(), tmp_vect.x()) / 3.1415926535 * 180;
        Vector3d rotation = Vector3d(0,0,yaw);

        translation = Vector3d(_origin.x(), _origin.y(), 0);       
        
        printf("%s[rrt.cpp] translation vector [%lf %lf %lf] yaw %lf\n", KBLU, 
            translation.x(), translation.y(), translation.z(), yaw);

        Vector3d transformed_translation = rotate_vector(rotation, translation);

        // We align everthing to the rotated x axis
        // So now we are playing in X and Z axis
        // Translate then rotate to temporary frame for start and end points
        // geometry_msgs::Point transformed_start = transform_point(
        //     vector_to_point(start), -rotation, transformed_translation);
        // geometry_msgs::Point transformed_end = transform_point(
        //     vector_to_point(end), -rotation, transformed_translation);

        geometry_msgs::Point transformed_start = forward_transform_point(
            vector_to_point(start), rotation, translation);
        geometry_msgs::Point transformed_end = forward_transform_point(
            vector_to_point(end), rotation, translation);

        Vector3d t_start = point_to_vector(transformed_start);
        Vector3d t_end = point_to_vector(transformed_end);

        printf("%s[rrt.cpp] transformed start (%lf %lf %lf) transformed end (%lf %lf %lf) \n", 
            KBLU, t_start.x(), t_start.y(), t_start.z(),
            t_end.x(), t_end.y(), t_end.z());

        // We find the original pcl in the transformed frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pcl_pc =
            base_to_transform_pcl(original_pcl_pc, 
            Vector3d(0, 0, yaw), translation);

        // Map size and origin should be determined and isolated 
        // Find a way to rotate the boundary so that we can minimize the space
        Vector3d _map_size;
        _map_size.x() = abs(t_start.x() - t_end.x()) + _xybuffer;
        _map_size.y() = abs(t_start.y() - t_end.y()) + _xybuffer + _passage_size;
        _map_size.z() = abs(t_start.z() - t_end.z()) + _zbuffer;

        printf("%s[rrt.cpp] map_size start (%lf %lf %lf)\n", 
            KBLU, _map_size.x(), _map_size.y(), _map_size.z());

        // We can crop the pointcloud to the dimensions that we are using
        // Origin will already to (0,0,0)
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cropped_pc1 = pcl2_filter(
            transformed_pcl_pc, Vector3d(0,0,_origin.z()), _map_size);

        // *** For DEBUG ***
        // transformed_pc1->points.clear();
        // transformed_pc1 = transformed_cropped_pc1;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cropped_pc = 
            pcl_z_scale(transformed_cropped_pc1, _scale_z);

        // map size is affected too
        // _min_height and _max_height is different too

        // *** Added in transform ***
        Vector3d map_size = Vector3d(_map_size.x(), _map_size.y(), _map_size.z() * _scale_z);
        
        double min_height = _min_height * _scale_z;
        double max_height = _max_height * _scale_z;

        double t_z = _origin.z() * _scale_z;

        double step_size = _step_size + _scale_z / 2.0;

        Vector3d t_t_start = Vector3d(t_start.x(), t_start.y(), t_start.z() *_scale_z);
        Vector3d t_t_end = Vector3d(t_end.x(), t_end.y(), t_end.z() *_scale_z);
        // *** End of adding in transform ***
        
        // Set line division to floor of step_size
        // Size must be 3 or more for linspace to work
        int line_search_division = max(4,(int)floor(step_size)); 

        size_t num_points = original_pcl_pc->size();
        int total = static_cast<int>(num_points);
        printf("%s[rrt.cpp] Actual obstacle size %d! \n", KGRN, total);

        // Run RRT 
        rrt_node rrt;
        rrt.error = true;
        int rrt_tries = 0;
        while (rrt_tries < _max_tries && rrt.process_status())
        {
            // Use for transformed start and end in transformed frame
            rrt.initialize(t_t_start, t_t_end, transformed_cropped_pc,
                map_size, Vector3d(0,0,t_z),
                step_size, _obs_threshold,
                min_height, max_height,
                line_search_division, _timeout, no_fly_zone,
                rotation, translation);
            rrt.run();
            rrt_tries++;
        }

        if (rrt_tries > _max_tries)
        {
            printf("%s[rrt.cpp] Will not continue bspline and publishing process! %s\n", KRED, KNRM);
            return false;
        }

        if (rrt.process_status())
        {
            printf("%s[rrt.cpp] Will not continue bspline and publishing process! %s\n", KRED, KNRM);
            return false;
        }

        printf("%s[rrt.cpp] rrt_size %d! %s\n", KGRN, (rrt.path_extraction()).size(), KNRM);


        // Extract path from transformed frame into normal frame
        // Remember to factor in z scale back
        std::vector<Vector3d> transformed_path1 = rrt.path_extraction();
        std::vector<Vector3d> transformed_path;

        // Transformed path needs to scale back the z
        for (int j = 0; j < transformed_path1.size(); j++)
        {
            Vector3d tmp_path = Vector3d(transformed_path1[j].x(), 
                transformed_path1[j].y(), transformed_path1[j].z() / _scale_z);

            transformed_path.push_back(tmp_path);
        }

        std::vector<Vector3d> path;

        // Vector3d transformed_translation_to_original = rotate_vector(
        //     -rotation, -translation);
        for (int j = 0; j < transformed_path.size(); j++)
        {
            Vector3d tmp_path = transformed_path[j];
            
            // geometry_msgs::Point n_tmp_path = transform_point(
            //     vector_to_point(tmp_path),
            //     -rotation, transformed_translation_to_original);
            geometry_msgs::Point n_tmp_path = backward_transform_point(
                vector_to_point(tmp_path), rotation, translation);


            path.push_back(point_to_vector(n_tmp_path));
        }

        // Somehow the algorithm will miss the start data
        path.push_back(start);

        // Now the whole path is flipped, hence we need to flip it back in bspline

        rrt_path = path;
        
        // create_bspline(_bs_order, end, start);
        return true;
    }

    void rrt_bspline(int _order)
    {
        bspline.clear();

        int v_size = rrt_path.size();
        MatrixXd wp = MatrixXd::Zero(3,v_size);
        
        // We need to flip it back since RRT path is inverted 
        for (int i = rrt_path.size()-1; i >= 0; i--)
        {
            wp(0,rrt_path.size()-1 - i) = rrt_path[i].x(); 
            wp(1,rrt_path.size()-1 - i) = rrt_path[i].y(); 
            wp(2,rrt_path.size()-1 - i) = rrt_path[i].z();
        }

        // Somehow the algorithm will miss the start data
        // wp(0,v_size-1) = s.x(); 
        // wp(1,v_size-1) = s.y(); 
        // wp(2,v_size-1) = s.z();

        // // Somehow the algorithm will miss the start data
        // // Since the nodes are counting backwards, hence the start point is the end point
        // Vector3d end_pose = e;

        // Start position will be at rrt_path.size()-1
        MatrixXd global_cp = setClampedPath(wp, 
        2, 4, _order, rrt_path[rrt_path.size()-1]);
        VectorXd knots = setKnotsPath(global_cp, 1, _order);
        std::vector<Vector3d> bs_tmp = updateFullPath(global_cp, 
            1, _order, knots);

        // Since the nodes are flipped we have to flip them back first
        // Change back the order
        for (int i = 0; i < bs_tmp.size(); i++)
            bspline.push_back(bs_tmp[i]);


    }

};

#endif