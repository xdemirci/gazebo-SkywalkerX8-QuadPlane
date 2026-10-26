#/bin/bash
clear
x-terminal-emulator -e sim_vehicle.py -v ArduPlane --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/skywalker_x8_quad.param --console -I0 --sysid=101 --mavproxy-args="--state-basedir=./logs"
sleep 3
gz sim -r skywalker_x8_quad_runway.sdf
