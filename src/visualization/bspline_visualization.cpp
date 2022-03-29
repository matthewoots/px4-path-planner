/*
 * bspline_visualization.cpp
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2021 Matthew (matthewoots at gmail.com)
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

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

#include <string>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include "px4_path_planner/Bspline.h"

using namespace std;
using namespace Eigen;

ros::Publisher cp_marker_pub, global_pose_pub, opt_cp_marker_pub;
ros::Subscriber bspline_sub, global_sub, opt_bspline_sub;
std::string _id; 

void bsplineCallback(const px4_path_planner::Bspline::ConstPtr &msg)
{
  px4_path_planner::Bspline bs = *msg;

  visualization_msgs::Marker control_points, line_strip, line_list;
  control_points.header.frame_id = line_strip.header.frame_id = "/map_nwu";
  control_points.header.stamp = line_strip.header.stamp = ros::Time::now();
  control_points.ns = line_strip.ns = "bspline_visualization";
  control_points.action = line_strip.action = visualization_msgs::Marker::ADD;
  control_points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  control_points.id = 0;
  line_strip.id = 1;

  control_points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;


  // POINTS markers use x and y scale for width/height respectively
  control_points.scale.x = 0.08;
  control_points.scale.y = 0.08;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.03;

  // Points are green
  control_points.color.g = 1.0f;
  control_points.color.a = 1.0;

  // Line strip is red
  line_strip.color.r = 1.0;
  line_strip.color.a = 1.0;

  // bsplineCallback;
  int cp_size = bs.global_control_points.size();
  geometry_msgs::Point cp_vector[cp_size];
  // std::cout << KYEL << "[bspline_visualization.cpp] cp_size \n" << KNRM << cp_size << std::endl;


  // Create the vertices for the points and lines
  for (int i = 0; i < cp_size; i++)
  {
    geometry_msgs::Point p;
    p = bs.global_control_points[i];
    // std::cout << KYEL << "[bspline_visualization.cpp] CP \n" << KNRM << p << std::endl;


    control_points.points.push_back(p);
    line_strip.points.push_back(p);
  }

  cp_marker_pub.publish(control_points);
  cp_marker_pub.publish(line_strip);
}

void optbsplineCallback(const px4_path_planner::Bspline::ConstPtr &msg)
{
  px4_path_planner::Bspline bs = *msg;

  visualization_msgs::Marker control_points, line_strip, line_list;
  control_points.header.frame_id = line_strip.header.frame_id = "/map_nwu";
  control_points.header.stamp = line_strip.header.stamp = ros::Time::now();
  control_points.ns = line_strip.ns = "opt_bspline_visualization";
  control_points.action = line_strip.action = visualization_msgs::Marker::ADD;
  control_points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  control_points.id = 0;
  line_strip.id = 1;

  control_points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;


  // POINTS markers use x and y scale for width/height respectively
  control_points.scale.x = 0.08;
  control_points.scale.y = 0.08;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.03;

  // Points are green
  control_points.color.g = 1.0f;
  control_points.color.b = 1.0f;
  control_points.color.a = 0.6;

  // Line strip is red
  line_strip.color.b = 1.0;
  line_strip.color.a = 0.6;

  // bsplineCallback;
  int cp_size = bs.global_control_points.size();
  geometry_msgs::Point cp_vector[cp_size];
  // std::cout << KYEL << "[bspline_visualization.cpp] cp_size \n" << KNRM << cp_size << std::endl;


  // Create the vertices for the points and lines
  for (int i = 0; i < cp_size; i++)
  {
    geometry_msgs::Point p;
    p = bs.global_control_points[i];
    // std::cout << KYEL << "[bspline_visualization.cpp] CP \n" << KNRM << p << std::endl;


    control_points.points.push_back(p);
    line_strip.points.push_back(p);
  }

  opt_cp_marker_pub.publish(control_points);
  opt_cp_marker_pub.publish(line_strip);
}

void globalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  geometry_msgs::PoseStamped pose_msg = *msg;
  
  visualization_msgs::Marker text_points;
  text_points.header.frame_id = "/map_nwu";
  text_points.header.stamp = ros::Time::now();
  text_points.ns = "pose_visualization_points";
  text_points.action = visualization_msgs::Marker::ADD;
  text_points.pose.orientation.w = 1.0;

  text_points.id = 0;

  text_points.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  text_points.scale.z = 0.3;
  text_points.color.r = 0.0f;
  text_points.color.g = 0.0f;
  text_points.color.b = 1.0f;
  text_points.color.a = 0.8f;

  text_points.pose.position = pose_msg.pose.position;
  text_points.pose.position.z += 1.0;

  text_points.text = _id;
  
  global_pose_pub.publish(text_points);
}

int main( int argc, char** argv )
{
  double rate = 1.0;
   
  ros::init(argc, argv, "bspline_visualization");
  ros::NodeHandle n("~");
  
  n.param<std::string>("agent_id", _id, "S0");

  cp_marker_pub = n.advertise<visualization_msgs::Marker>(
        "cp_visualization_marker", 10);
  global_pose_pub = n.advertise<visualization_msgs::Marker>(
        "gp_visualization_marker", 10);
  opt_cp_marker_pub = n.advertise<visualization_msgs::Marker>(
        "opt_cp_visualization_marker", 10);

  bspline_sub = n.subscribe<px4_path_planner::Bspline>(
        "/" + _id + "/path/bspline", 10, &bsplineCallback);
  global_sub = n.subscribe<geometry_msgs::PoseStamped>(
        "/" + _id + "/global_pose", 10, &globalPoseCallback);
  opt_bspline_sub = n.subscribe<px4_path_planner::Bspline>(
        "/" + _id + "/path/opt_bspline", 10, &optbsplineCallback);

  ros::Rate r(rate);

  ros::spin();

  return 0;
}


  



