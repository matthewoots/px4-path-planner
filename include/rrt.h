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
#include <future>

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr base_pcl;


public:

    std::vector<Vector3d> bspline;

    // bool unpack_rrt_params(std::string _file_location)
    // {
    //     printf("%s[rrt.h] Trying to open %s \n", KYEL, _file_location.c_str());
    //     ifstream file(_file_location);
        
    //     if (!file)
    //     {
    //         printf("%s[rrt.h] File not present! \n", KRED);
    //         return false;
    //     }
    //     printf("%s[rrt.h] Success, found %s \n", KGRN, _file_location.c_str());

    //     io::CSVReader<10> in(_file_location);
    //     in.read_header(io::ignore_extra_column, "step_size", 
    //         "obs_threshold", "xybuffer", "zbuffer", "passage_size",
    //         "min_height", "max_height", "max_tries", "timeout", "z_scale");
        
    //     double step_size;
    //     double obs_threshold;
    //     double xybuffer;
    //     double zbuffer;
    //     double passage_size;

    //     double min_height;
    //     double max_height;
    //     int max_tries;
    //     double timeout;
    //     double z_scale;

    //     // First pass is to get number of rows
    //     while (in.read_row(step_size, obs_threshold, xybuffer, zbuffer,
    //         passage_size, min_height, max_height, max_tries, timeout, z_scale))
    //     {
    //         _step_size = step_size;
    //         _obs_threshold = obs_threshold;
    //         _xybuffer = xybuffer;
    //         _zbuffer = zbuffer;
    //         _passage_size = passage_size;

    //         _min_height = min_height;
    //         _max_height = max_height;
    //         _max_tries = max_tries;
    //         _timeout = timeout;
    //         _scale_z = z_scale;
    //     }

    //     return true;
    // } 

    sensor_msgs::PointCloud2 get_query_pcl()
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*base_pcl, msg);
        sensor_msgs::PointCloud2 tmp_pc = transform_sensor_cloud(msg,
            - Vector3d(0, 0, - yaw), Vector3d(0, 0, 0));
        msg = transform_sensor_cloud(tmp_pc,
            - Vector3d(0, 0, 0), translation);

        return msg;
    }

    bool initialize_rrt_params(vector<double> params, int param_size)
    {
        if (params.size() != param_size)
            return false;

        _step_size = params[0];
        _obs_threshold = params[1];
        _xybuffer = params[2];
        _zbuffer = params[3];
        _passage_size = params[4];

        _min_height = params[5];
        _max_height = params[6];
        _max_tries = params[7];
        _timeout = params[8];
        _scale_z = params[9];

        return true;
    }

    bool RRT(sensor_msgs::PointCloud2 pcl_pc, 
    Vector3d start, Vector3d end, vector<VectorXd> no_fly_zone, int threads)
    {
        rrt_path.clear();
        pcl::PointCloud<pcl::PointXYZ>::Ptr original_pcl_pc = 
            pcl2_converter(pcl_pc);

        printf("%s[rrt.cpp] Start (%lf %lf %lf) End (%lf %lf %lf) \n", 
            KBLU, start.x(), start.y(), start.z(),
            end.x(), end.y(), end.z());

        // Find the origin of the transformed frame
        Vector3d _origin = (start + end) / 2;
        
        // Do the preparation for transformation
        // Find the translation vector and the yaw angle
        Vector3d tmp_vect = end - start;
        yaw = atan2(tmp_vect.y(), tmp_vect.x()) / M_PI * 180;
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cropped_pc1 = pcl_ptr_filter(
            transformed_pcl_pc, Vector3d(0,0,_origin.z()), _map_size);

        // *** For DEBUG ***
        pcl::PointCloud<pcl::PointXYZ>::Ptr empty(new pcl::PointCloud<pcl::PointXYZ>);
        base_pcl = empty;
        base_pcl = transformed_cropped_pc1;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cropped_pc = 
            pcl_z_scale(transformed_cropped_pc1, _scale_z);

        // map size is affected too
        // _min_height and _max_height is different too

        // *** Added in transform ***
        Vector3d map_size = Vector3d(_map_size.x(), _map_size.y(), _map_size.z() * _scale_z);
        
        double min_height = _min_height * _scale_z;
        double max_height = _max_height * _scale_z;

        double t_z = _origin.z() * _scale_z;

        double step_size;
        if (_scale_z == 1.0)
            step_size = _step_size;
        else
            step_size = _step_size + _scale_z / 2;
        

        Vector3d t_t_start = Vector3d(t_start.x(), t_start.y(), t_start.z() *_scale_z);
        Vector3d t_t_end = Vector3d(t_end.x(), t_end.y(), t_end.z() *_scale_z);
        // *** End of adding in transform ***
        
        // Set line division to floor of step_size
        // Size must be 3 or more for linspace to work
        int line_search_division = max(4,(int)floor(step_size/_obs_threshold)); 

        size_t num_points = original_pcl_pc->size();
        int total = static_cast<int>(num_points);
        printf("%s[rrt.cpp] Actual obstacle size %d! \n", KGRN, total);

        // Run RRT 
        // rrt_node rrt;
        // rrt.error = true;
        // int rrt_tries = 0;
        // while (rrt_tries < _max_tries && rrt.process_status())
        // {
        //     // Use for transformed start and end in transformed frame
        //     rrt.initialize(t_t_start, t_t_end, transformed_cropped_pc,
        //         map_size, Vector3d(0,0,t_z),
        //         step_size, _obs_threshold,
        //         min_height, max_height,
        //         line_search_division, _timeout, no_fly_zone,
        //         rotation, translation);
        //     rrt.run();
        //     rrt_tries++;
        // }
        
        std::atomic<bool> ready;
        ready = false;
        std::vector<Vector3d> final_result;

        int rrt_counter = 0;
        while (!ready)
        {
            printf("%s  rrt retry %d! \n", KRED, rrt_counter);
            std::vector<std::future<std::vector<Vector3d>>> futures;

            for (size_t i = 0; i < max(1,threads-2); i++)
            {
                futures.push_back(
                    std::async(std::launch::async, [&]()
                    {
                        rrt_node rrt;
                        rrt.error = true;

                        // Use for transformed start and end in transformed frame
                        rrt.initialize(t_t_start, t_t_end, transformed_cropped_pc,
                            map_size, Vector3d(0,0,t_z),
                            step_size, _obs_threshold,
                            min_height, max_height,
                            line_search_division, _timeout, no_fly_zone,
                            rotation, translation);

                        return rrt.run(ready);
                    })
                );
            }
            
            std::vector<std::vector<Vector3d>> results;

            for (auto& currentFuture : futures)
                results.push_back(currentFuture.get());

            for (const auto& currentResult : results)
            {
                if (currentResult.size() > 0)
                {
                    final_result = currentResult;
                    break;
                }
            }
            rrt_counter++;
        } //while(!ready)

        // if (rrt_tries > _max_tries)
        // {
        //     printf("%s[rrt.cpp] Will not continue bspline and publishing process! %s\n", KRED, KNRM);
        //     return false;
        // }

        // if (rrt.process_status())
        // {
        //     printf("%s[rrt.cpp] Will not continue bspline and publishing process! %s\n", KRED, KNRM);
        //     return false;
        // }

        printf("%s[rrt.cpp] rrt_size %lu! %s\n", KGRN, final_result.size(), KNRM);

        if (final_result.empty())
        {
            printf("%s[rrt.cpp] final_result is empty! %s\n", KRED, KNRM);
            printf("%s[rrt.cpp] Will not continue bspline and publishing process! %s\n", KRED, KNRM);
            return false;
        }

        // Extract path from transformed frame into normal frame
        // Remember to factor in z scale back
        std::vector<Vector3d> transformed_path1 = final_result;
        std::vector<Vector3d> transformed_path;

        // Transformed path needs to scale back the z
        for (int j = 0; j < transformed_path1.size(); j++)
        {
            Vector3d tmp_path = Vector3d(transformed_path1[j].x(), 
                transformed_path1[j].y(), transformed_path1[j].z() / _scale_z);

            transformed_path.push_back(tmp_path);
        }

        std::vector<Vector3d> shortest_transformed_path;
        shortest_transformed_path.push_back(transformed_path[0]);
        int shortest_idx = 0;

        // Find Shortest Path
        while (shortest_idx < transformed_path.size()-1)
        {
            // If the next point is the last point, we just push back the last point
            if (shortest_idx+1 == transformed_path.size()-1)
            {
                shortest_idx = transformed_path.size()-1;
                shortest_transformed_path.push_back(transformed_path[shortest_idx]);
                printf("    last value %d\n", shortest_idx);
                break;
            }

            // Check 2 steps ahead since the next value would be the fall back by default
            for (int k = shortest_idx + 2; k < transformed_path.size(); k++)
            {
                if (!check_validity_pcl(shortest_transformed_path[shortest_transformed_path.size()-1],
                    transformed_path[k], _obs_threshold,
                    transformed_cropped_pc))
                {
                    // We take the index that was successful which is the previous value
                    shortest_idx = k-1;
                    shortest_transformed_path.push_back(transformed_path[shortest_idx]);
                    printf("    next value %d\n", shortest_idx);
                    break;
                }
                else if (k == transformed_path.size()-1)
                {
                    shortest_idx = k;
                    shortest_transformed_path.push_back(transformed_path[shortest_idx]);
                    printf("    next value %d\n", shortest_idx);
                    break;
                }
            }
        }

        std::vector<Vector3d> path;

        // Somehow the shortest path algorithm will miss the end data
        // Bspline somehow misses the last point
        // path.push_back(end);
        // path.push_back(end);

        // Vector3d transformed_translation_to_original = rotate_vector(
        //     -rotation, -translation);
        for (int j = 0; j < shortest_transformed_path.size(); j++)
        {
            Vector3d tmp_path = shortest_transformed_path[j];
            
            // geometry_msgs::Point n_tmp_path = transform_point(
            //     vector_to_point(tmp_path),
            //     -rotation, transformed_translation_to_original);
            geometry_msgs::Point n_tmp_path = backward_transform_point(
                vector_to_point(tmp_path), rotation, translation);


            path.push_back(point_to_vector(n_tmp_path));
        }

        // Somehow the algorithm will miss the start data
        // path.push_back(start);
        // path.push_back(start);

        // Now the whole path is flipped, hence we need to flip it back in bspline

        rrt_path = path;
        
        // create_bspline(_bs_order, end, start);
        return true;
    }

    void rrt_bspline(int _order)
    {
        bspline.clear();

        if (_order == 0)
        {
            for (int i = rrt_path.size() - 1 ; i >= 0; i--)
                bspline.push_back(rrt_path[i]);
            return;
        }

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
        MatrixXd global_cp = set_clamped_path(wp, 
        2, 4, _order, rrt_path[rrt_path.size()-1]);
        VectorXd knots = set_knots_path(global_cp, 1, _order);
        std::vector<Vector3d> bs_tmp = update_full_path(global_cp, 
            1, _order, knots);

        // Since the nodes are flipped we have to flip them back first
        // Change back the order
        for (int i = 0; i < bs_tmp.size(); i++)
            bspline.push_back(bs_tmp[i]);


    }

    bool check_validity_pcl(Vector3d p, Vector3d q, double obs_threshold,
        pcl::PointCloud<pcl::PointXYZ>::Ptr obs)
    {
        Vector3d large, small;
        int i = 0, j1 = 0, j2 = 0;
        Vector3d difference = q - p;
        int n = (int)ceil(difference.norm() / obs_threshold);
        MatrixXd line_vector;
        line_vector = MatrixXd::Zero(3, n);

        Vector3d valid_origin;
        valid_origin.x() = (p.x() + q.x()) / 2;
        valid_origin.y() = (p.y() + q.y()) / 2;
        valid_origin.z() = (p.z() + q.z()) / 2;

        // printf("%s  p (%lf %lf %lf) q (%lf %lf %lf) m (%lf %lf %lf)\n", 
        //     KGRN, p.x(), p.y(), p.z(),
        //     q.x(), q.y(), q.z(),
        //     valid_origin.x(), valid_origin.y(), valid_origin.z());

        Vector3d local_map_size;
        local_map_size.x() = abs(p.x() - q.x()) + 2*obs_threshold;
        local_map_size.y() = abs(p.y() - q.y()) + 2*obs_threshold;
        local_map_size.z() = abs(p.z() - q.z()) + 2*obs_threshold;

        // printf("%s  local_map_size (%lf %lf %lf)\n", 
        //     KBLU, local_map_size.x(), local_map_size.y(), local_map_size.z());

        pcl::PointCloud<pcl::PointXYZ>::Ptr local_obs;
        local_obs = pcl_ptr_filter(obs, valid_origin, 
                local_map_size);

        size_t local_num_points = local_obs->size();
        int local_points_total = static_cast<int>(local_num_points);

        if (local_points_total == 0)
            return true;

        // pcl::PointCloud<pcl::PointXYZ>::Ptr local_obs = obs;

        for (int i = 0; i < 3; i++)
        {
            line_vector.row(i) = linspace(p[i], q[i], (double)n);
        }
        int column_size = line_vector.cols();

        for(int i = 0; i < column_size; i++)
        {
            Vector3d tmp;
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_obs;

            tmp.x() = line_vector.col(i).x();
            tmp.y() = line_vector.col(i).y();
            tmp.z() = line_vector.col(i).z();

            // Check to see whether the point in the line 
            // collides with an obstacle
            

            // ------ Optimization on PCL ------
            tmp_obs = pcl_ptr_filter(local_obs, tmp, 
                Vector3d(2*obs_threshold, 
                2*obs_threshold, 2*obs_threshold));
            size_t num_points = tmp_obs->size();
            int total = static_cast<int>(num_points);
            // printf("%s[rrt_standalone.h] tmp_obs size %d! \n", KGRN, total);

            size_t t_num_points = tmp_obs->size();
            int t_total = static_cast<int>(t_num_points);
            // printf("%s[rrt_standalone.h] obs size %d! \n", KGRN, t_total);
            
            
            if (total == 0)
                continue;

            if (kdtree_collide_pcl(line_vector.col(i), tmp_obs, obs_threshold))
                return false;

            // -------- Original on PCL (2) ------
            // ---- (1) is 20-100x faster than (2) -----
            // if (kdtree_pcl(line_vector.col(i), obs, obs_threshold))
            //     return false;

            // -------- Original on PCL (3) ------
            // ---- (2) is 100x faster than (3) -----
            // if (obstacle_map_filter(line_vector.col(i)))
            //     return false;

            // Check to see whether the line is within the boundaries
            // Don't think we need this check
            // if((i<0) || (i>400) || (j1<0) || (j1>400) || (j2<0) || (j2>400))
            //     continue;
        }
        return true;
    }

};

#endif
