# Esp32-FC
This is a complex and highly specialized piece of code designed for a quadcopter or drone flight controller. It handles Wi-Fi communication, UDP packet parsing (using the Crazyflie CRTP protocol structure), sensor reading (MPU6050 gyroscope), and basic PID control loop execution.

🚁 ESP32 Wi-Fi Quadcopter Flight Controller (CRTP Protocol)
This project utilizes an ESP32 as the core flight controller for a quadcopter, implementing a basic PID control loop and receiving real-time setpoint commands over Wi-Fi via UDP, mimicking the data structure of the Crazyflie Real-Time Protocol (CRTP).

The system is configured as a Wi-Fi Access Point (AP) to provide a direct, low-latency communication link for sending control inputs (Roll, Pitch, Yaw, and Thrust) from a remote client (e.g., a desktop controller or smartphone app).

✨ Features
Custom Wi-Fi Access Point: The ESP32 creates its own dedicated network for reliable control signal transmission.

UDP Communication: Uses the UDP protocol for low-latency, real-time command reception.

CRTP Setpoint Decoding: Parses incoming UDP packets structured similarly to the Crazyflie's CRTP setpoint packets (specifically, ports 0x03 and 0x07).

MPU Gyroscope Integration: Reads angular rates (Roll, Pitch, Yaw) from an MPU (likely MPU6050/9250 via I2C) for stabilization feedback.

PID Rate Controller: Implements a three-axis PID control function (pid_equetion) to stabilize the drone's angular rates.

Motor Mixing: Calculates final thrust values (M1 to M4) for a standard X-configuration quadcopter.

Disconnection Safeguard: Automatically cuts motor power (M1 - M4 set to 0) and resets the PID integrator if no command packets are received within a 900ms timeout.

🛠️ Hardware Requirements
Microcontroller: ESP32 Development Board (handles Wi-Fi AP and UDP).

Inertial Measurement Unit (IMU): MPU-6050, MPU-9250, or similar, connected via I2C (Wire library).

Motor Drivers: Four Electronic Speed Controllers (ESCs) for the motors.

Motors & Propellers: Four brushless DC motors.

Status LED: Connected to GPIO 2 (used to indicate connection status).

Parameter,Value,Description
SSID,"""ESP-DRONE_F09E9E21DB55""",The name of the Wi-Fi network.
Password,"""12345678""",The password to connect to the network.
AP IP,192.168.43.42,The fixed IP address of the ESP32.
UDP Port,2390,The port used to listen for incoming setpoint commands.

I2C and LED Setup
I2C (Wire.begin()): Required for communication with the MPU sensor at the standard address (0x68).

Control LED (GPIO 2):

OFF (LOW): Receiving control packets (Connected).

ON (HIGH): Disconnected/Timeout (Motor power cut).

🛩️ Flight Control Logic
1. Packet Decoding (processUdpData)
The function checks the first byte (Header) of the incoming UDP packet to determine its type:

Setpoint (Port 0x03 or 0x07, Channel 0): The packet payload is decoded into setpoint_t structure, updating Input_Roll, Input_Pitch, Input_Yaw, and thrust. This is the primary control input.

2. Control Loop (loop())The main loop executes the following steps every interval (4ms):Read Gyroscope: gyro_signals() reads the raw angular rates and subtracts the initial calibration offsets (RCR, RCP, RCY).Calculate Error: Compares the desired angular rates (Input_Roll, Input_Pitch, Input_Yaw) from the incoming packets with the current measured rates (RateRoll, RatePitch, RateYaw) to find the error (ERR, ERP, ERY).Run PID: The error is passed to the pid_equetion function to calculate the motor correction values (IRoll, IPitch, IYaw).Motor Mixing: The final motor values (M1 to M4) are calculated using the standard X-quadcopter mixing formula, combining the Throttle (Input_Throttle) with the PID corrections.$$\begin{aligned} M_1 &= \text{Throttle} - \text{Roll} - \text{Pitch} - \text{Yaw} \\ M_2 &= \text{Throttle} - \text{Roll} + \text{Pitch} + \text{Yaw} \\ M_3 &= \text{Throttle} + \text{Roll} + \text{Pitch} - \text{Yaw} \\ M_4 &= \text{Throttle} + \text{Roll} - \text{Pitch} + \text{Yaw} \end{aligned}$$The TV (Throttle Value) variable is used as a safety gate, ensuring that motor power is only applied if the commanded thrust is greater than zero.

Meta Command (Port 0x07, Channel 1): Used for non-flight commands. The provided code includes a decoder for metaNotifySetpointsStop which simply prints "Stop command received".
