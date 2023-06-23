# eimo

## Package Description
| Package name     | Description                                                                                   | Scripts                                                                    | Dependency                                |
|------------------|-----------------------------------------------------------------------------------------------|----------------------------------------------------------------------------|-------------------------------------------|
| eimo_msgs        | Control, Depth, Angle.                                                                        | depth_ms5837.py<br/>mouse_multi.py<br/>witsensor.py<br/>request_voltage.py | ms5837<br/>pyspacemouse<br/>libhidapi-dev |
| pac9685          | Light, Propeller.                                                                             |                                                                            | pigpio<br/>simple_pid<br/>                |
| pyspacenavigator | Angle on Windows Platform Only; Provide Control. P.S. there is alternative package for Linux. | mouse.py<br/>mouse_test.py<br/>mouse_trigger.py                            | pywinusb                                  |
| scl_passthrough  | Passthrough for Moons step servo Motor; Provide Depth, Voltage.                               | diving.py<br/>srv: serial_passthrough.py                                   | serial                                    |
| testa            | Test as fresh man.                                                                            |                                                                            |                                           |
| witprotocol      | Provide Angle.                                                                                |                                                                            | serial                                    |

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