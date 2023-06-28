#!/bin/bash
#python3 $HOME/eimo_remote/src/pca9685/src/pca9685/PCA9685.py
screen -dmS scl bash -c 'source /home/ubuntu/.bashrc; roslaunch scl_passthrough serial_passthrough.launch'
screen -dmS angle bash -c 'sleep 1; source /home/ubuntu/.bashrc; rosrun eimo_msgs witsensor.py'
screen -dmS depth bash -c 'sleep 1; source /home/ubuntu/.bashrc; rosrun eimo_msgs depth_ms5837.py'
screen -dmS pca bash -c 'sleep 1; source /home/ubuntu/.bashrc; rosrun pca9685 propelling.py'
screen -dmS dive bash -c 'sleep 3; source /home/ubuntu/.bashrc; rosrun scl_passthrough diving.py'