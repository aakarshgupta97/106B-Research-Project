#!/bin/bash

cd $HOME
xterm -hold -e "roscore" &
xterm -hold -e "rosrun master_discovery_fkie master_discovery" &
xterm -hold -e "rosrun master_sync_fkie master_sync" &

exit 0

