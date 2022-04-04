#! /usr/bin/env python3
import rospy
import rosnode
import sys
from tabulate import tabulate

def first4(s):
    return s[:4]
def first5(s):
    return s[:5]

rospy.init_node("watchdog")
print ("number of arguments", len(sys.argv)-1)
lists = []
table = [[] for x in range(len(sys.argv)-1)]
for x in range(len(sys.argv)-1):
        lists.append("/S"+sys.argv[x+1]+"/")
        table.append(sys.argv[x+1])
print (lists)        

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

rate = rospy.Rate(1) # ROS Rate at 5Hz
while not rospy.is_shutdown():
        print ("Available Nodes:")
        nodes_names = rosnode.get_node_names()
        # Go through all the various nodes
        for i in range(len(nodes_names)):
                # Go through all the uav index
                for j in range(len(lists)):
                        if first4(nodes_names[i]) == lists[j]:
                                for k in range(len(nodes_to_check)):
                                        if nodes_names[i] == lists[j]+nodes_to_check[k]:
                                                table[j].append("1")
                                        else:
                                                table[j].append("0")
        # print (nodes_names)
        #define header names
        col_names = ["Index", nodes_to_check[0], nodes_to_check[1], nodes_to_check[2], nodes_to_check[3]]
        
        #display table
        print(tabulate(table, headers=col_names))
        rate.sleep()