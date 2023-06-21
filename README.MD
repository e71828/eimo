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
