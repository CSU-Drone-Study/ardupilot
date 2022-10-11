# Overview
It is possible to connect a LORD Microstrain AHRS system to a Pixhawk 4 running a custom version of Ardupilot.

## Hardware
- Pixhawk 4
- 3DM-CX5-45 (or theoretically any other sensor with EKF support and the MIP packet protocol)
- Custom Cable to connect sensor to UART

### Flight Platform
It is important to set up your flight platform in a way that data read from the flight controller and lord is consistent, this requires setting up both on the same plane and facing the same direction. If this is not convenient it is also possible to correct for the orientation in software, instruction for this and further mounting information can be found [here](https://ardupilot.org/copter/docs/common-mounting-the-flight-controller.html).

### LORD Sensor
It is important to properly configure the lord to send out the correct data and use the corresponding baudrate with the Pixhawk.

INS Packet (50hz for plane or 500hz for copter)

- Ambient pressure field
- Scaled Magnetometer field
- Scaled Accelerometer field
- Scaled Gyro field
- CF Quaternion field

## Software
While the current release of Ardupilot includes code that includes support for parsing data from 3DM series sensors, we have seen a number of issues when using the GPS so we have been working on developing an branch of Ardupilot that works around these issues.

### Building the Software
Lord Documentation Content Without Formatting

### Ardupilot Parameters
There are a variety of important parameters that need to be set using mission planner

- AHRS\_EKF\_TYPE 11

This parameter tells Ardupilot to use the External AHRS as the primary estimator

- EAHRS\_TYPE 2

EAHRS\_TYPE specifies which External AHRS backend to use

- SERIALx\_PROTOCOL 36

The protocol needs to be set to 36 for the serial port used for the AHRS, when using the Pixhawk4 this would be SERIAL4\_PROTOCOL

- SERIALx\_BAUD

This should be set to the baudrate the lord is configured at, we used 115 on Plane and 921 on Copter

- EAHRS\_RATE

This needs to be set to the rate in HZ of the fastest data being provided by the AHRS, we used 50 for Plane and 500 for Copter

- GPS\_TYPE

This specifies the GPS to use, for the Pixhawk GPS set it to 2 and for the LORD GPS set it to 21.

- GPS\_AUTO\_SWITCH

This should be set to 0 to prevent Ardupilot from switching between GPSs

## Support
LORD support engineers are always available to expand on this subject and support you in any way we can.

## Example Technical Notes for reference**

1. <https://www.microstrain.com/sites/default/files/8401-0087-3dm-gx5-using-an-hardware-datalogger-with-inertial-sensors.pdf>
2. <https://www.microstrain.com/sites/default/files/applications/files/8401-0081-3dm-gx5-45-bluetooth.pdf>

