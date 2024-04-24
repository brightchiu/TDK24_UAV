# 0621 Altitude & Attitude Controller
# Use Python 2.7 or 3.7

# !/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import numpy as np
import threading
import time
import math

# Parameter
# Ultrasonic Sensor Setting
TRIG_PIN = 2
ECHO_PIN = 3
AIR_TEMP = 29             # Calibrate before flight (degree Celsius)
ALT_OFFSET_US = -0.065    # Distance from ultrasonic sensor to ground (m)
ALT_OFFSET_BM = 0         # Distance from barometer sensor to ground (m)
ALT_OFFSET_LS = 0         # Distance from laser sensor to ground (m)
SAMPLE_NUMBER = 5         # Numbers of sampling
WEIGHT_US = 1             # Weight of ultrasonic height
WEIGHT_BM = 0             # Weight of barometer height
WEIGHT_LS = 0             # Weight of laser height (Disable)

# Global Variable (Don't Change)
ALT_TARGET = 0.0          # Target altitude (m)
ALT_ACTUAL = 0.0          # Actual altitude after combination (m)
ALT_ACTUAL_US = 0.0       # Actual altitude from ultrasonic (m)
ALT_ACTUAL_BM = 0.0       # Actual altitude from barometer (m)
ALT_ACTUAL_LS = 0.0       # Actual altitude from laser (m)
ATT_TARGET_PITCH = 0.0
ATT_TARGET_ROLL = 0.0
ATT_TARGET_YAW = 0.0
ACTUAL_VEL = 0.0
VELOCITY_TARGET = 0.0
STATE = 1                 # 1:Enable Program  0:Disable Program
MODE = ''                 # AP MODE: LINE, POS

# Altitude Controller Setting
ALT_Kp = 1                # P-gain value for P-controller
ALT_Ki = 0                # Set 0 to close I-controller
ALT_Kd = 0                # Set 0 to close D-controller
ALT_Gain_Division = 1
THRUST = 0.5              # Velocity thrust (0:full decent 0.5:leveling 1:full climb)

# Attitude Controller Setting
ATT_Kp = 1                # P-gain value for P-controller
ATT_Ki = 0                # Set 0 to close I-controller
ATT_Kd = 0                # Set 0 to close D-controller
ATT_Gain_Division = 1
ATT_FEEDBACK_OFFSET = -0.0625   # Move velocity value to zero

# Plot controller data for debug
R_NUM = []
R_HEIGHT_TARGET = []
R_HEIGHT_ACTUAL = []
R_HEIGHT_BM = []
R_HEIGHT_US = []
R_ERROR = []
R_GAIN_P = []
R_GAIN_I = []
R_GAIN_D = []
R_GAIN_SUM = []
R_THRUST_OUT = []
R_THRUST_OFFSETED = []
R_THRUST = []

R_VX_ACTUAL = []
R_VX_TARGET = []
R_VX_ERROR = []
R_PITCH_OUT = []

# Loop frequency
CMD_FREQ = 5              # Hz
ALT_FREQ = 10
ATT_FREQ = 10
SENSOR_FREQ = 20


# Initializing
# Setting connect parameter
print('Connecting to the UAV ,please wait.')
uav = connect('/dev/serial0', wait_ready=True, baud=921600)
print('UAV connected.')


# Function Def. Area
# Altitude controller (Background Function)
def altitude_controller():
    # Preset variable
    global THRUST
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    print('Altitude Controller ON.')

    # Controller loop
    while STATE:
        # Calculate error
        error = ALT_TARGET - ALT_ACTUAL
        # R_ERROR.append(error)     # Record error
        R_HEIGHT_TARGET.append(ALT_TARGET)
        R_HEIGHT_ACTUAL.append(ALT_ACTUAL)
        # R_HEIGHT_BM.append(ALT_ACTUAL_BM)
        # R_HEIGHT_US.append(ALT_ACTUAL_US)

        # Error Value Preprocess
        current_time = time.time()
        delta_error = error - previous_error
        delta_time = current_time - previous_time
        int_error += (error * delta_time)
        derivative = delta_error / delta_time

        # PID Gain and Summation
        gain_p = error * ALT_Kp
        gain_i = int_error * ALT_Ki
        gain_d = derivative * ALT_Kd
        gain_sum = gain_p + gain_i + gain_d
        # R_GAIN_P.append(gain_p)
        # R_GAIN_I.append(gain_i)
        # R_GAIN_D.append(gain_d)
        # R_GAIN_SUM.append(gain_sum)      # Record gains

        # Map output into specific range (thrust range -0.5~+0.5)
        thrust_out = gain_sum / ALT_Gain_Division
        # R_THRUST_OUT.append(thrust_out)

        # Offset to midpoint
        thrust_out += 0.5
        # R_THRUST_OFFSETED.append(thrust_out)

        # Output Limitation
        if thrust_out > 1:
            thrust_out = 1
        if thrust_out < 0:
            thrust_out = 0

        # Renew the thrust value
        THRUST = thrust_out
        R_THRUST.append(THRUST)

        # Pass values for next loop
        previous_error = error
        previous_time = current_time

        time.sleep(1/ALT_FREQ)     # Send message at designated Hz

        # Close threading when STATE = 0

    print('Altitude Controller Closed.')


# Altitude Sensor (Background Function)
def altitude_sensor():
    # Initializing
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    GPIO.output(TRIG_PIN, False)
    height_us_collect = []
    height_bm_collect = []
    # height_ls_collect = []
    pulse_end = time.time()
    pulse_start = time.time()

    print('Altitude Sensor ON.')

    # Sensor loop
    while STATE:
        # Trig the emitter
        GPIO.output(TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(TRIG_PIN, False)

        # Wait for a pulse and record
        while GPIO.input(ECHO_PIN) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO_PIN) == 1:
            pulse_end = time.time()

        # Calculate the height and set the filter
        pulse_duration = pulse_end - pulse_start
        height = (pulse_duration * (331.3 + 0.606 * AIR_TEMP)) / 2 + ALT_OFFSET_US
        if height > 2:
            height = 2
        if height < 0:
            height = 0

        # Convert to actual Attitude
        roll_angel = uav.attitude.roll
        pitch_angel = uav.attitude.pitch
        height_us = height * math.cos(roll_angel) * math.cos(pitch_angel)

        # Read barometer data
        height_bm = uav.location.global_relative_frame.alt + ALT_OFFSET_BM
        # height_ls = LS.range / 1000 + ALT_OFFSET_LS

        # Noise reduction and update values
        global ALT_ACTUAL_US, ALT_ACTUAL_BM, ALT_ACTUAL_LS, ALT_ACTUAL
        # Ultrasonic sensor
        height_us_collect.insert(0, height_us)
        if len(height_us_collect) > SAMPLE_NUMBER:
            height_us_collect.pop()
        ALT_ACTUAL_US = np.average(height_us_collect)

        # Barometer sensor
        height_bm_collect.insert(0, height_bm)
        if len(height_bm_collect) > SAMPLE_NUMBER:
            height_bm_collect.pop()
        ALT_ACTUAL_BM = np.average(height_bm_collect)

        # Laser sensor
        # height_ls_collect.insert(0, height_ls)
        # if len(height_ls_collect) > SAMPLE_NUMBER:
        #    height_ls_collect.pop()
        # ALT_ACTUAL_LS = np.average(height_ls_collect)

        # Combined the height by weighted average method
        alt_sum = (WEIGHT_US * ALT_ACTUAL_US + WEIGHT_BM * ALT_ACTUAL_BM +
                   WEIGHT_LS * ALT_ACTUAL_LS) / (WEIGHT_US + WEIGHT_BM + WEIGHT_LS)
        if alt_sum > 2:
            alt_sum = 2
        ALT_ACTUAL = alt_sum

        time.sleep(1 / SENSOR_FREQ)  # Send message at designated Hz(Exclude loop time 10ms)

        # Close threading when STATE = 0

    print('Altitude Sensor Closed.')


# Attitude controller (Background Function go with Line Tracking Mode)
def attitude_controller():
    # Preset variable
    global ATT_TARGET_PITCH
    int_error = 0
    previous_error = 0
    previous_time = time.time()

    print('Attitude Controller ON.')

    # Controller loop
    while STATE:
        # Renew the velocity data

        # vy = uav.velocity[1]
        # vz = uav.velocity[2]
        R_VX_ACTUAL.append(ACTUAL_VEL)

        # Calculate error
        error = VELOCITY_TARGET - ACTUAL_VEL
        R_VX_TARGET.append(VELOCITY_TARGET)
        R_VX_ERROR.append(error)
        # Velocity direction need to Verify

        # Error Value Preprocess
        current_time = time.time()
        delta_error = error - previous_error
        delta_time = current_time - previous_time
        int_error += (error * delta_time)
        derivative = delta_error / delta_time

        # PID Gain and Summation
        gain_p = error * ATT_Kp
        gain_i = int_error * ATT_Ki
        gain_d = derivative * ATT_Kd
        gain_sum = gain_p + gain_i + gain_d

        # Map output into specific range (pitch range -10~+10 degree)
        pitch_out = gain_sum / ATT_Gain_Division

        # Output Limitation
        if pitch_out > 10:
            pitch_out = 10
        if pitch_out < -10:
            pitch_out = -10

        # Renew the pitch value only in Line Tracking mode
        if MODE == 'LINE':
            ATT_TARGET_PITCH = pitch_out
        R_PITCH_OUT.append(ATT_TARGET_PITCH)

        # Pass values for next loop
        previous_error = error
        previous_time = current_time

        time.sleep(1/ATT_FREQ)     # Send message at designated Hz

        # Close threading when STATE = 0

    print('Attitude Controller closed.')


# Velocity Sensor
def velocity_sensor():
    # Preset variable
    global ACTUAL_VEL
    previous_time = time.time()
    vx = 0.0
    vy = 0.0

    print('Velocity Sensor ON.')

    # Sensor loop
    while STATE:
        # Receive acceleration
        ax = uav.velocity[0]
        ay = uav.velocity[1]

        # Calculate delta time
        current_time = time.time()
        delta_time = current_time - previous_time

        # Calculate velocity on x and y direction
        vx += ax * delta_time
        vy += ay * delta_time

        # Calculate total velocity
        v = math.sqrt(math.pow(vx, 2) + math.pow(vy, 2))

        # Output the value to global variable
        ACTUAL_VEL = v

        # Pass values for next loop
        previous_time = current_time

        # Close threading when STATE = False

    print('Velocity Sensor Closed.')


# Command Sending Function (Background Function)
def command_sending():
    while STATE:
        msg = uav.message_factory.set_attitude_target_encode(
            0,  # time boot
            0,  # target system
            0,  # target component
            0b00000000,  # type mask: bit 1 is LSB
            to_quaternion(ATT_TARGET_ROLL, ATT_TARGET_PITCH, ATT_TARGET_YAW),  # quaternion
            0,  # body roll rate in radian
            0,  # body pitch rate in radian
            0,  # body yaw rate in radian
            THRUST)  # thrust
        uav.send_mavlink(msg)

        time.sleep(1/CMD_FREQ)     # Send message at designated Hz

        # Close threading when STATE = 0

    print('Command Sending Closed.')


# Convert euler Angel to quaternion
def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


# CMD input function:
def command(v, h, t):
    global VELOCITY_TARGET, ALT_TARGET
    VELOCITY_TARGET = v
    ALT_TARGET = h
    print('Flight at %.2f m/s, height %.2f m, duration %d sec.)' % (v, h, t))
    time.sleep(t)


# Close the UAV
def shutdown():
    print("UAV landed.")

    print("Switching to Stabilize Mode")
    uav.mode = VehicleMode("STABILIZE")
    time.sleep(1)
    print("Now is in Mode: %s" % uav.mode.name)

    # Wait of disarmed
    while not uav.mode.name == 'STABILIZE':
        print('Wait for changing to stabilize mode')
        time.sleep(1)
    while not uav.armed:
        print('Wait for disarmed.')
        time.sleep(1)
    print("Armed = %s" % uav.armed)

    # Close the connection
    print('Close the connect to UAV.')
    uav.close()
    print('Shutdown Completed')


# Threading procedure
def startup():
    # Def. threading.thread
    alt_sen = threading.Thread(target=altitude_sensor)
    alt_ctrl = threading.Thread(target=altitude_controller)
    att_ctrl = threading.Thread(target=attitude_controller)
    cmd_send = threading.Thread(target=command_sending)
    m = threading.Thread(target=main)

    # Start the threading
    alt_sen.start()
    alt_ctrl.start()
    att_ctrl.start()
    cmd_send.start()
    m.start()

    # Wait for threading close
    m.join()
    cmd_send.join()
    alt_ctrl.join()
    att_ctrl.join()
    alt_sen.join()
    print('All threading closed.')


# Main function
def main():
    print("UAV is standby.")
    # Check RC switch is in Stabilize mode
    while not uav.mode.name == 'STABILIZE':
        print('Please push RC mode switch to Stabilize mode')
        time.sleep(1)

    print("Switching to Guided_NoGPS Mode")
    uav.mode = VehicleMode("GUIDED_NOGPS")
    time.sleep(1)
    print("Now is in Mode: %s" % uav.mode.name)

    print('Setting the UAV armed')
    uav.armed = True
    time.sleep(1)
    print("Armed = %s" % uav.armed)
    print('Start the mission.')
    global MODE
    MODE = 'LINE'        # Enable attitude controller

    # Flight procedure
    # format command(velocity, height, duration)

    command(0, 1, 10)
    command(1, 1, 3)
    command(-1, 0.5, 3)
    command(0, 0.5, 5)

    # Landing & Safety check
    while ALT_ACTUAL > 0.05:
        command(0, 0, 1)
        print('Wait for landing...')
        time.sleep(0.5)

    for i in range(1, len(R_PITCH_OUT) + 1, 1):
        R_NUM.append(i)

    # Close threading
    global STATE
    STATE = 0


# Show data's trend
def show_data():
    # plt.plot(R_NUM, R_ERROR, label="Error Height")
    # plt.plot(R_NUM, R_HEIGHT_US, label="Actual Height -Ultrasonic")
    # plt.plot(R_NUM, R_HEIGHT_BM, label="Actual Height -Barometer")
    # plt.plot(R_NUM, R_HEIGHT_TARGET, label="Target Height")
    # plt.plot(R_NUM, R_HEIGHT_ACTUAL, label="Actual Height")

    # plt.plot(R_NUM, R_GAIN_P, label="P-gain")
    # plt.plot(R_NUM, R_GAIN_I, label="I-gain")
    # plt.plot(R_NUM, R_GAIN_D, label="D-gain")
    # plt.plot(R_NUM, R_GAIN_SUM, label="Gain sum")

    # plt.plot(R_NUM, THRUST_OUT, label="Thrust-row")
    # plt.plot(R_NUM, THRUST_OFFSETED, label="Thrust-offseted")
    # plt.plot(R_NUM, R_THRUST, label="Thrust")

    plt.plot(R_NUM, R_VX_TARGET, label="Vx-Target")
    plt.plot(R_NUM, R_VX_ACTUAL, label="Vx-Actual")
    plt.plot(R_NUM, R_PITCH_OUT, label="Pitch Out")

    # naming the x axis
    plt.ylabel('Value')
    # naming the y axis
    plt.xlabel('Data number')
    # giving a title to my graph
    plt.title('Output Data Diagram')
    # show a legend on the plot
    plt.legend()
    # function to show the plot
    plt.show()


startup()
shutdown()
show_data()
