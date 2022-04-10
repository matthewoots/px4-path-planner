#! /usr/bin/env python3
import rospy, rostopic
import rosnode
import sys
import os
from tabulate import tabulate

# Modules names when launching:
# px4_path_planner_visualization
# formation_controller_standalone
# px4_path_planner
# mavros

nodes_to_check = []
nodes_to_check.append("px4_path_planner")
nodes_to_check.append("mavros")
nodes_to_check.append("formation_controller_standalone")
nodes_to_check.append("px4_path_planner_visualization")
nodes_to_check.append("oak_ros")
nodes_to_check.append("basalt_vio_ros")
nodes_to_check.append("apriltag_detection")
nodes_to_check.append("loop_fusion")

topics_to_check = [] 
topics_to_check.append("mavros/setpoint_raw/local")
topics_to_check.append("mavros/local_position/pose")
topics_to_check.append("global_pose")
topics_to_check.append("mavros/odom_nwu")

rospy.init_node("watchdog")
# print ("number of arguments", len(sys.argv)-1)
lists = []
table = [[] for x in range(len(sys.argv)-1)]
count = []
# print (table) 
for x in range(len(sys.argv)-1):
        lists.append("/S"+sys.argv[x+1]+"/")
        table[x].append("0")
        table[x] = [[] for x in range(len(nodes_to_check)+2)]
        table[x][0] = sys.argv[x+1]
        count.append("0")
# print (lists)  
# print (nodes_to_check)

sub_count = []
h_count = []
topic_counter = 0
for j in range(len(lists)):
        for k in range(len(topics_to_check)):
                h_count.append(rostopic.ROSTopicHz(-1))  
                sub_count.append(rospy.Subscriber(lists[j]+topics_to_check[k], rospy.AnyMsg, h_count[topic_counter].callback_hz, callback_args=lists[j]+topics_to_check[k])) 
                topic_counter = topic_counter + 1
                      

rate = rospy.Rate(1) # ROS Rate at 5Hz
while not rospy.is_shutdown():
        os.system("clear") # Linux - OSX
        print ("Available Nodes:")
        nodes_names = rosnode.get_node_names()
        # Go through all the uav index
        for j in range(len(lists)):
                count[j] = 0
                # Go through the various nodes names
                for k in range(len(nodes_to_check)):
                        # Go through all the various found nodes
                        for i in range(len(nodes_names)):
                                # Filter those which nodes 
                                if (nodes_names[i]) == (lists[j]+nodes_to_check[k]):
                                        table[j][k+1] = "o"
                                        count[j] = count[j]+1
                                        break
                                if i == len(nodes_names)-1:
                                        table[j][k+1] = "x"
                if count[j] == len(nodes_to_check):
                        table[j][len(nodes_to_check)+1] = "ready"
                else:
                        table[j][len(nodes_to_check)+1] = "not ready"
                                        
        # print (table)
        #define header names
        col_names = ["Index", "path", "mavr", 
        "form", "vis", "oak", "vio", "detn", "loop", "status"]
        
        #display table
        print(tabulate(table, headers=col_names, tablefmt="github", stralign="center")) 

        topic_counter = 0
        for j in range(len(lists)):
                print("-------")
                for k in range(len(topics_to_check)):
                        
                        data = h_count[topic_counter].get_hz(lists[j]+topics_to_check[k])
                        # print(type(data))
                        hz = 0
                        if data is None:
                                hz = 0
                        else:
                                hz = data[0]
                        print(lists[j]+topics_to_check[k] + " = " + str(hz)[:6] + " Hz")
                        topic_counter = topic_counter + 1
                
        rate.sleep()

        

        