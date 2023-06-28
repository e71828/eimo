# eimo

## Package Description
| Package name     | Description                                                                                   | Scripts                                                                    | Dependency                                |
|------------------|-----------------------------------------------------------------------------------------------|----------------------------------------------------------------------------|-------------------------------------------|
| eimo_msgs        | Control, Depth, Angle.                                                                        | depth_ms5837.py<br/>mouse_multi.py<br/>witsensor.py<br/>request_voltage.py | ms5837<br/>pyspacemouse<br/>libhidapi-dev |
| pac9685          | Light, Propeller.                                                                             |                                                                            | pigpio<br/>simple_pid<br/>                |
| scl_passthrough  | Passthrough for Moons step servo Motor; Provide Depth, Voltage.                               | diving.py<br/>srv: serial_passthrough.py                                   | serial                                    |
| witprotocol      | Provide Angle.                                                                                |                                                                            | serial                                    |

## Rosnode Description

![rosgraph](C:\Users\admin\PycharmProjects\eimo\rosgraph.png)

## Roslaunch Description

| Roslaunch Name                         | Description                                                                                              | ~/.bashrc                                                                                                                                                                   | Machine |
|----------------------------------------|----------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
| eimo.launch                            | robot will wait for control                                                                              | source /opt/ros/noetic/setup.bash <br/>source /home/ubuntu/eimo_remote/devel/setup.sh <br/>export ROS_MASTER_URI=http://192.168.31.16:11311<br/>export ROS_IP=192.168.31.16 | remote  |
| mouse.launch<br/>(include eimo.launch) | robot will be controlled<br/>Attention: cp [eimo.sh](eimo_msgs\scripts\eimo.sh)  to /home/ubuntu/eimo.sh | source /opt/ros/noetic/setup.bash <br/>source /home/zngz/eimo_ws/devel/setup.sh <br/>export ROS_MASTER_URI=http://192.168.31.16:11311<br/>export ROS_IP=192.168.31.236      | local   |



## Service
```bash
#cat /etc/systemd/system/clash.service
[Unit]
Description=A rule-based tunnel in go
After=network-online.target
[Service]
Type=simple
User=root
UMask=007
ExecStart=/opt/clash/clash-linux-amd64-v1.11.8 -d /opt/clash/
Restart=on-failure
RestartSec=1
# Configures the time to wait before service is stopped forcefully.
TimeoutStopSec=300
[Install]
WantedBy=multi-user.target
```

```bash
#cat /etc/systemd/system/pigpiod.service
[Unit]
Description=Daemon required to control GPIO pins via pigpio
After=remote-fs.target
After=syslog.target
After=network.target
After=systemd-user-sessions.service
[Service]
Type=forking
User=root
UMask=007
ExecStart=/usr/local/bin/pigpiod
ExecStop=/bin/systemctl kill pigpiod
[Install]
WantedBy=multi-user.target
```

```bash
#cat /etc/systemd/system/eimo.service
[Unit]
Description=start eimo_remote
After=remote-fs.target
After=syslog.target
After=network.target
After=systemd-user-sessions.service
After=pigpiod.service
[Service]
Type=forking
User=ubuntu
UMask=007
ExecStart=/home/ubuntu/ros_autoRun.sh
[Install]
WantedBy=multi-user.target
```

```bash
# cat /etc/systemd/system/camera.service
[Unit]
Description=start gstreamer
After=remote-fs.target
After=syslog.target
After=network.target
After=systemd-user-sessions.service
[Service]
Type=simple
User=root
UMask=007
ExecStart=/home/ubuntu/camera.sh
[Install]
WantedBy=multi-user.target
```

```bash
# cat /etc/systemd/system/roscore.service
[Unit]
Description=start roscore
After=remote-fs.target
After=syslog.target
After=network.target
After=systemd-user-sessions.service
[Service]
Type=idle
User=ubuntu
UMask=007
ExecStart=/home/ubuntu/roscore.sh
Restart=on-abort
[Install]
WantedBy=multi-user.target
```
