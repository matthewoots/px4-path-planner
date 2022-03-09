#!/bin/bash

RED='\033[0;31m' # ${RED}
NC='\033[0m' # ${NC} no color
GREEN='\033[0;32m' # ${GREEN}
BLUE='\033[0;34m' # ${BLUE}

export LC_ALL="en_US.UTF-8"

echo "${GREEN}Sending land${NC}"
rostopic pub /${UAV_GROUPNAME}/global_planner_mission std_msgs/Float32MultiArray "layout: 
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [5.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0, 999.0]" -1 

