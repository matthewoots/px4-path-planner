/*
 * rrt_standalone.h
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
#include <rrt_helper.h>

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>

#include <geometry_msgs/Point.h>

using namespace Eigen;
using namespace std;
using namespace rrt_helper;

#define dmax std::numeric_limits<double>::max();

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

class rrt_node
{
    private:

    struct Node 
    {
        vector<Node *> children;
        Node *parent;
        Vector3d position;
    };

    constexpr static int max_nodes = 2000;
    Node start_node;
    Node end_node;
    Node* nodes[max_nodes]; // ?? need to clean this up
    pcl::PointCloud<pcl::PointXYZ>::Ptr obs;

    bool reached;
    double obs_threshold;
    int step_size, iter, total_nodes;
    int line_search_division;
    double _min_height, _max_height;

    double timeout = 0.1;

    Vector3d map_size = Vector3d::Zero();
    Vector3d origin = Vector3d::Zero(); 

    Vector3d rotation, translation;

    vector<VectorXd> no_fly_zone;   

    Vector3d get_pc_pose(int idx)
    {
        Vector3d p = Vector3d(obs->points[idx].x,
            obs->points[idx].y,
            obs->points[idx].z);
        
        return p;
    }

    void rrt()
    {
        double prev =ros::Time::now().toSec();
        int index = 0;

        Node* random_node = new Node;
        Node* step_node = new Node;
    
        // Provide to .2 decimals
        srand((unsigned)(ros::WallTime::now().toNSec()));
        double random_x = (double)rand() / (INT_MAX + 1.0);
        (random_node->position).x() = random_x * map_size.x() - map_size.x()/2;
        

        srand((unsigned)(ros::WallTime::now().toNSec()));
        double random_y = (double)rand() / (INT_MAX + 1.0);
        (random_node->position).y() = random_y * map_size.y() - map_size.y()/2;
        
        Vector3d transformed_vector = Vector3d((random_node->position).x(),
                (random_node->position).y(), 0);
        for (int i = 0; i < no_fly_zone.size(); i++)
        {
            // x_min, x_max, y_min, y_max in original frame
            double x_min = no_fly_zone[i][0], x_max = no_fly_zone[i][1];
            double y_min = no_fly_zone[i][2], y_max = no_fly_zone[i][3];
            
            // Let us transform the point back to original frame
            geometry_msgs::Point n_tmp_path = backward_transform_point(
            vector_to_point(transformed_vector), rotation, translation);

            // Reject point if it is in no fly zone
            if ( n_tmp_path.x <= x_max && n_tmp_path.x >= x_min &&
                n_tmp_path.y <= y_max && n_tmp_path.y >= y_min)
                return;
        }

        // Constrain height within the min-max range
        srand((unsigned)(ros::WallTime::now().toNSec()));
        double random_z = (double)rand() / (INT_MAX + 1.0);
        double tmp =  random_z * map_size.z() - map_size.z()/2 + origin.z();
        tmp = max(tmp, _min_height);
        tmp = min(tmp, _max_height);
        (random_node->position).z() = tmp;        

        // printf("%srandom_xyz %s %lf %lf %lf %s\n", 
        //     KBLU, KNRM, random_x, random_y, random_z, KBLU);

        index = near_node(*random_node);
        
        if((separation(random_node->position, nodes[index]->position)) < step_size)
            return;
        else
        {
            step_node->position = stepping(nodes[index]->position, random_node->position);
            Vector3d transformed_vector = Vector3d(step_node->position.x(),
                step_node->position.y(), step_node->position.z());
            // Let us transform the point back to original frame
            geometry_msgs::Point n_tmp_path = backward_transform_point(
                vector_to_point(transformed_vector), rotation, translation);
            for (int i = 0; i < no_fly_zone.size(); i++)
            {
                // x_min, x_max, y_min, y_max in original frame
                double x_min = no_fly_zone[i][0], x_max = no_fly_zone[i][1];
                double y_min = no_fly_zone[i][2], y_max = no_fly_zone[i][3];
                
                // Reject point if it is in no fly zone
                if ( n_tmp_path.x <= x_max && n_tmp_path.x >= x_min &&
                    n_tmp_path.y <= y_max && n_tmp_path.y >= y_min)
                {
                    // printf("%sRejected x %lf y %lf!\n", KRED, n_tmp_path.x, n_tmp_path.y);
                    return;
                }
                // printf("%sAccepted x %lf y %lf!\n", KGRN, n_tmp_path.x, n_tmp_path.y);
                    
            }
        }
        
        bool flag = check_validity(nodes[index]->position, step_node->position);

        if(flag)
        {
            nodes[total_nodes++] = step_node;
            step_node->parent = nodes[index];
            (nodes[index]->children).push_back(step_node);
            
            if((check_validity(step_node->position, end_node.position)) && 
                (separation(step_node->position,end_node.position) < step_size/1.5))
            {
                printf("%sReached path!\n", KGRN);
                reached = true;
                nodes[total_nodes++] = &end_node;
                end_node.parent = step_node;
                (nodes[total_nodes-1]->children).push_back(&end_node);
                return;
            }
        }
        iter++;

        double final_secs =ros::Time::now().toSec() - prev;
        printf("%squery (%s%.2lf %.2lf %.2lf%s) time-taken (%s%.4lf%s)\n", 
            KBLU, KNRM,
            random_node->position.x(),
            random_node->position.y(),
            random_node->position.z(),
            KBLU, KNRM, final_secs, KBLU);
    }

    // [near_node] is responsible for finding the nearest node in the tree 
    // for a particular random node. 
    int near_node(Node random)
    {
        double min_dist = dmax;
        // We give dist a default value if total node is 1 it will fall back on this
        double dist = separation(start_node.position, random.position);
        
        int linking_node = 0;

        for(int i = 0; i < total_nodes; i++)
        {
            // Other nodes than start node
            dist = separation(nodes[i]->position, random.position);
            // Evaluate distance
            if(dist < min_dist)
            {
                min_dist = dist;
                linking_node = i;
            }
        }
        return linking_node;
    }

    // [separation] function takes two coordinates
    // as its input and returns the distance between them.
    double separation(Vector3d p, Vector3d q)
    {
        Vector3d v;
        v.x() = p.x() - q.x();
        v.y() = p.y() - q.y();
        v.z() = p.z() - q.z();
        return sqrt(pow(v.x(), 2) + pow(v.y(), 2) + pow(v.z(), 2));
    }

    // [stepping] function takes the random node generated and its nearest node in the tree 
    // as its input, and returns the coordinates of the step node. 
    // This function determines the step node by generating a new node at a distance 
    // of step_size from nearest node towards random node
    Vector3d stepping(Vector3d nearest_node, Vector3d random_node)
    {
        Vector3d tmp = Vector3d::Zero();
        Vector3d step = Vector3d::Zero();
        Vector3d norm = Vector3d::Zero();
        double magnitude = 0.0;
        double x, y, z;

        tmp.x() = random_node.x() - nearest_node.x();
        tmp.y() = random_node.y() - nearest_node.y();
        tmp.z() = random_node.z() - nearest_node.z();
        magnitude = sqrt(pow(tmp.x(),2) + 
            pow(tmp.y(),2) +
            pow(tmp.z(),2));

        norm.x() = (tmp.x() / magnitude);
        norm.y() = (tmp.y() / magnitude);
        norm.z() = (tmp.z() / magnitude);

        step.x() = nearest_node.x() + step_size * norm.x();
        step.y() = nearest_node.y() + step_size * norm.y();
        step.z() = nearest_node.z() + step_size * norm.z();

        return step;
    }

    // They check if the step node that has been generated is valid or not
    // It will be invalid if the step node either lies in the proximity of a pointcloud (that is, an obstacle) 
    // or if the straight line path joining nearest_node and step_node goes through an obstacle
    bool check_validity(Vector3d p, Vector3d q)
    {
        Vector3d large, small;
        int i = 0, j1 = 0, j2 = 0;
        int n = line_search_division;
        MatrixXd line_vector;
        line_vector = MatrixXd::Zero(3, n);
        for (int i = 0; i < 3; i++)
        {
            line_vector.row(i) = linspace(p[i], q[i], (double)n);
        }
        int column_size = line_vector.cols();

        for(int i = 0; i < column_size; i++)
        {
            Vector3d tmp;
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_obs;
            pcl::PointCloud<pcl::PointXYZ>::Ptr local_obs = obs;

            tmp.x() = line_vector.col(i).x();
            tmp.y() = line_vector.col(i).y();
            tmp.z() = line_vector.col(i).z();

            // Check to see whether the point in the line 
            // collides with an obstacle
            

            // ------ Optimization on PCL ------
            tmp_obs = pcl2_filter_ptr(local_obs, tmp, 
                Vector3d(2*step_size, 2*step_size, 2*step_size));
            size_t num_points = tmp_obs->size();
            int total = static_cast<int>(num_points);
            // printf("%s[rrt_standalone.h] tmp_obs size %d! \n", KGRN, total);

            size_t t_num_points = obs->size();
            int t_total = static_cast<int>(t_num_points);
            // printf("%s[rrt_standalone.h] obs size %d! \n", KGRN, t_total);
            
            
            if (total == 0)
                continue;

            if (kdtree_pcl(line_vector.col(i), tmp_obs, obs_threshold))
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

    // Terrible brute force method that very upsettingly have poor performance
    bool obstacle_map_filter(Vector3d point)
    {
        // Load obstacle map
        size_t num_points = obs->size();
        int total = static_cast<int>(num_points);

        for (int i = 0; i < total; i++)
        {
            double tmp = separation(point, 
                Vector3d(obs->points[i].x, obs->points[i].y, obs->points[i].z));
            if (tmp < obs_threshold)
                return false;
        }
        return true;    
    }

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
    * @brief Convert point cloud from ROS sensor message to 
    * pcl point ptr
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr 
        pcl2_converter(sensor_msgs::PointCloud2 _pc)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(_pc, pcl_pc2);

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        pcl::fromPCLPointCloud2(pcl_pc2, *tmp_cloud);
        
        return tmp_cloud;
    }

    /* 
    * @brief Filter point cloud with the dimensions given
    */
    pcl::PointCloud<pcl::PointXYZ>::Ptr 
        pcl2_filter_ptr(pcl::PointCloud<pcl::PointXYZ>::Ptr _pc, 
        Vector3d centroid, Vector3d dimension)
    {   
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

        float minX = centroid.x() - dimension.x()/2;
        float maxX = centroid.x() + dimension.x()/2;

        float minY = centroid.y() - dimension.y()/2;
        float maxY = centroid.y() + dimension.y()/2;

        float minZ = centroid.z() - dimension.z()/2;
        float maxZ = centroid.z() + dimension.z()/2;

        pcl::CropBox<pcl::PointXYZ> box_filter;
        box_filter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
        box_filter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));

        box_filter.setInputCloud(_pc);
        box_filter.filter(*output);

        // printf("%s[rrt_standalone.h] return fromPCLPointCloud2! \n", KBLU);
        return output;
    }


    public:

    bool error;
    // constructor
    rrt_node()
    {
        cout<<"Constructor called"<<endl;
    }
 
    // destructor
    ~rrt_node()
    {
        cout<<"Destructor called"<<endl;
    }

    bool kdtree_pcl(Vector3d point, pcl::PointCloud<pcl::PointXYZ>::Ptr _obs,
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

    void initialize(Vector3d _start, 
        Vector3d _end, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr _pc,
        Vector3d _map_size,
        Vector3d _origin,
        double _step_size,
        double _obs_threshold,
        double min_height,
        double max_height,
        int _line_search_division,
        double _timeout,
        vector<VectorXd> _no_fly_zone,
        Vector3d _rotation,
        Vector3d _translation)
    {
        // printf("%s[rrt_standalone.h] Key Variables! \n", KBLU);
        start_node.position = _start;
        start_node.parent = NULL;
        total_nodes = 0;
        reached = false;
        timeout = _timeout;

        no_fly_zone.clear(); 
        no_fly_zone = _no_fly_zone;
        printf("%s[rrt_standalone.h] no_fly_zone size %d! \n", KBLU, no_fly_zone.size());

        rotation = _rotation;
        translation = _translation;

        // Reset and clear the nodes
        for (int i = 0; i < max_nodes; i++)
        {
            Node* random_node = new Node;
            nodes[i] = random_node;
            nodes[i]->children.clear();
        }

        error = false;

        nodes[total_nodes++] = &start_node;
        end_node.position = _end;

        // printf("%s[rrt_standalone.h] PCL conversion! \n", KBLU);
        // obs(new pcl::PointCloud<pcl::PointXYZ>);
        obs = _pc;
        obs_threshold = _obs_threshold;

        _min_height = min_height;
        _max_height = max_height;

        // printf("%s[rrt_standalone.h] Map parameters! \n", KBLU);
        map_size = _map_size;
        origin = _origin;
        step_size = _step_size;
        iter = 0;
        line_search_division = _line_search_division;
        printf("%s[rrt_standalone.h] Initialized! \n", KBLU);
        // srand(time(NULL));
    }

    void run()
    {
        size_t num_points = obs->size();
        int total = static_cast<int>(num_points);
        double prev = ros::Time::now().toSec();
        double fail_timer = ros::Time::now().toSec();
        printf("%s[rrt_standalone.h] Obstacle size %d! \n", KGRN, total);
        printf("%s[rrt_standalone.h] Start run process! \n", KGRN);
        
        while(!reached)
        {
            rrt();
            if (total_nodes > max_nodes || ros::Time::now().toSec() - fail_timer > timeout)
            {
                printf("%s[rrt_standalone.h] Failed run process! \n", KRED);
                printf("%s[rrt_standalone.h] Exceeded max nodes or runtime too long! \n", KRED);
                error = true;
                break;
            }
        }
        printf("%sSolution found! with %d iter and %d nodes\n", 
            KGRN, iter, total_nodes);
        printf("%sTotal Time Taken = %lf!\n", KGRN, ros::Time::now().toSec() - prev);
    }

    std::vector<Vector3d> path_extraction()
    {
        Node up, down;
        int breaking = 0;
        down = end_node;
        up = *(end_node.parent);
        std::vector<Vector3d> path;
        int i = 0;
        while(1)
        {
            path.push_back(down.position);
            if(up.parent == NULL)
                break;
            up = *(up.parent);
            down = *(down.parent);
            i++;
        }

        return path;
    }

    bool process_status()
    {
        return error;
    }
};
