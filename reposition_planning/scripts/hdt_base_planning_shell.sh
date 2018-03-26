#!/bin/bash

rosrun tf static_transform_publisher $1 $2 $3 $4 $5 $6 top_shelf camera_link 20 &
sleep 1
pgrep static_transfor | xargs kill

