#!/bin/bash

RED='\033[0;31m' # ${RED}
NC='\033[0m' # ${NC} no color
GREEN='\033[0;32m' # ${GREEN}
BLUE='\033[0;34m' # ${BLUE}

export LC_ALL="en_US.UTF-8"

echo "${BLUE}The system is booting...${NC}"
delay_05=0.5
delay_1=1
delay_2=2
delay_5=5


sleep ${delay_2}
echo "${GREEN}Sending Takeoff${NC}"
rostopic pub /${UAV_GROUPNAME}/global_planner_mission std_msgs/Float32MultiArray "layout: 
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [1.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0]" -1

sleep ${delay_5}
echo "${GREEN}Delay 5${NC}"
sleep ${delay_5}
echo "${GREEN}Delay 10${NC}"
sleep ${delay_5}
echo "${GREEN}Delay 15${NC}"
sleep ${delay_5}
echo "${GREEN}Delay 20${NC}"

echo "${GREEN}Sending Bspline mode and mission for S0${NC}"
rostopic pub /${UAV_GROUPNAME}/global_planner_mission std_msgs/Float32MultiArray "layout: 
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [3.0, 2.0, -1.0, 0.0, 1.6, 999.0, 999.0, 999.0, 999.0]" -1 

sleep ${delay_5}
echo "${GREEN}Delay 5${NC}"
sleep ${delay_5}
echo "${GREEN}Delay 10${NC}"

echo "${GREEN}Sending Bspline mode and mission for S0${NC}"
rostopic pub /${UAV_GROUPNAME}/global_planner_mission std_msgs/Float32MultiArray "layout: 
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [3.0, 2.0, 1.0, 1.0, 1.6, 999.0, 999.0, 999.0, 999.0]" -1 

sleep ${delay_5}
echo "${GREEN}Delay 5${NC}"
sleep ${delay_5}
echo "${GREEN}Delay 10${NC}"

echo "${GREEN}Sending Bspline mode and mission for S0${NC}"
rostopic pub /${UAV_GROUPNAME}/global_planner_mission std_msgs/Float32MultiArray "layout: 
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [3.0, 2.0, -1.0, -1.0, 1.6, 999.0, 999.0, 999.0, 999.0]" -1 

sleep ${delay_5}
echo "${GREEN}Delay 5${NC}"
sleep ${delay_5}
echo "${GREEN}Delay 10${NC}"

echo "${GREEN}Sending land${NC}"
rostopic pub /${UAV_GROUPNAME}/global_planner_mission std_msgs/Float32MultiArray "layout: 
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [5.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0]" -1 
