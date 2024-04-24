# 0612 Altitude Controller
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
# Ultrasonic Range Sensor
TRIG_PIN = 2
ECHO_PIN = 3
AIR_TEMP = 35             # Calibrate before flight (degree Celsius)
ALT_OFFSET_US = -0.065    # distance from ultrasonic sensor to ground (m)
ALT_OFFSET_BM = 0         # distance from barometer sensor to ground (m)
SAMPLE_NUMBER = 5         # Numbers of sampling

# Global Variable (Don't Change)
ALT_TARGET = 0.0      # Target altitude (m)
ALT_ACTUAL = 0.0      # Actual altitude after combination (m)
ALT_ACTUAL_US = 0.0   # Actual altitude from ultrasonic (m)
ALT_ACTUAL_BM = 0.0   # Actual altitude from barometer (m)
ALT_ACTUAL_LS = 0.0   # Actual altitude from laser (m)
WEIGHT_US = 1         # Weight of ultrasonic height
WEIGHT_BM = 0         # Weight of barometer height
WEIGHT_LS = 0         # Weight of laser height (Disable)
ALT_OLDCMD = 0.0      # Previous altitude command (m)
THRUST = 0.5          # Velocity thrust (0:full decent 0.5:leveling 1:full climb)
STATE = 1             # 1:Enable Program  0:Disable Program

# Altitude controller setting
Kp = 1                # P-gain value for P-controller
Ki = 0                # Set 0 to close I-controller
Kd = 0                # Set 0 to close D-controller
Gain_Division = 1

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

# UAV property
CMD_FREQ = 5             # Hz
ALT_FREQ = 10            # Hz

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
        R_ERROR.append(error)     # Record error
        R_HEIGHT_TARGET.append(ALT_TARGET)
        R_HEIGHT_ACTUAL.append(ALT_ACTUAL)
        R_HEIGHT_BM.append(ALT_ACTUAL_BM)
        R_HEIGHT_US.append(ALT_ACTUAL_US)

        # Error Value Preprocess
        current_time = time.time()
        delta_error = error - previous_error
        delta_time = current_time - previous_time
        int_error += (error * delta_time)
        derivative = delta_error / delta_time

        # PID Gain and Summation
        gain_p = error * Kp
        gain_i = int_error * Ki
        gain_d = derivative * Kd
        gain_sum = gain_p + gain_i + gain_d
        # R_GAIN_P.append(gain_p)
        # R_GAIN_I.append(gain_i)
        # R_GAIN_D.append(gain_d)
        # R_GAIN_SUM.append(gain_sum)      # Record gains

        # Map output into specific range (thrust range -0.5~+0.5)
        thrust_out = gain_sum / Gain_Division
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

    # i2c = busio.I2C(board.SCL, board.SDA)
    # vl53 = adafruit_vl53l0x.VL53L0X(i2c)

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

        # Calculate the height
        pulse_duration = pulse_end - pulse_start
        height = (pulse_duration * (331.3 + 0.606 * AIR_TEMP)) / 2 + ALT_OFFSET_US

        # Convert to actual Attitude
        roll_angel = uav.attitude.roll
        pitch_angel = uav.attitude.pitch
        height_us = height * math.cos(roll_angel) * math.cos(pitch_angel)
        if height_us > 2:
            height_us = 2
        if height_us < 0:
            height_us = 0

        # Read barometer data
        height_bm = uav.location.global_relative_frame.alt + ALT_OFFSET_BM
        # height_ls = vl53.range / 1000

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
        # height_ls_collect.pop()
        # ALT_ACTUAL_LS = np.average(height_ls_collect)

        # Combined the height by weighted average method
        ALT_ACTUAL = (WEIGHT_US * ALT_ACTUAL_US + WEIGHT_BM * ALT_ACTUAL_BM +
                      WEIGHT_LS * ALT_ACTUAL_LS) / (WEIGHT_US + WEIGHT_BM + WEIGHT_LS)

        time.sleep(0.04)      # Update at 20 Hz (include loop time 10ms)

        # Close threading when STATE = 0

    print('Altitude Sensor Closed.')


# Sending the attitude command
def flight_command(roll_angle=0.0, pitch_angle=0.0, yaw_angle=0.0,
                   roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0,
                   alt=0.0, duration=0):
    global ALT_TARGET
    ALT_TARGET = alt
    msg = uav.message_factory.set_attitude_target_encode(
        0,  # time boot
        0,  # target system
        0,  # target component
        0b00000000,    # type mask: bit 1 is LSB
        to_quaternion(roll_angle, pitch_angle, yaw_angle),  # quaternion
        roll_rate,     # body roll rate in radian
        pitch_rate,    # body pitch rate in radian
        yaw_rate,      # body yaw rate in radian
        THRUST)        # thrust
    uav.send_mavlink(msg)

    if duration != 0:
        # Divide the duration into the fractional and integer parts
        elapsed_time = math.modf(duration)

        # Send command to vehicle on Hz cycle
        for x in range(0, int(elapsed_time[1]) * CMD_FREQ):
            msg = uav.message_factory.set_attitude_target_encode(
                0, 0, 0, 0b00000000,
                to_quaternion(roll_angle, pitch_angle, yaw_angle),
                roll_rate, pitch_rate, yaw_rate, THRUST)
            uav.send_mavlink(msg)

            time.sleep(1 / CMD_FREQ)   # Send message at designated Hz


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


# Threading procedure
def startup():
    # Def. threading.thread
    alt_sen = threading.Thread(target=altitude_sensor)
    alt_ctrl = threading.Thread(target=altitude_controller)
    m = threading.Thread(target=main)

    alt_sen.start()
    alt_ctrl.start()
    m.start()

    m.join()
    alt_ctrl.join()
    alt_sen.join()
    print('All threading closed.')


# Close the UAV connection
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

    # Flight procedure
    # format flight_command(roll_angle=0.0, pitch_angle=0.0, yaw_angle=0.0,
    #                       roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0,
    #                       alt=0, duration=0)

    print('CMD1')
    flight_command(alt=1.5, duration=5)
    print('CMD2')
    flight_command(alt=1, duration=2)
    print('CMD3')
    flight_command(alt=0.5, duration=2)
    print('CMD4')
    flight_command(alt=0.2, duration=1)

    # Landing & Safety check
    while ALT_ACTUAL > 0.05:
        flight_command(alt=0, duration=1)
        print('Wait for landing...')

    print('Flight CMD ended.')

    # Close threading
    global STATE
    STATE = 0


# Show data's trend
def show_data():
    for i in range(1, len(R_THRUST) + 1, 1):
        R_NUM.append(i)
    # plt.plot(R_NUM, R_ERROR, label="Error Height")
    # plt.plot(R_NUM, R_HEIGHT_US, label="Actual Height -Ultrasonic")
    # plt.plot(R_NUM, R_HEIGHT_BM, label="Actual Height -Barometer")
    plt.plot(R_NUM, R_HEIGHT_TARGET, label="Target Height")
    plt.plot(R_NUM, R_HEIGHT_ACTUAL, label="Actual Height")

    # plt.plot(R_NUM, R_GAIN_P, label="P-gain")
    # plt.plot(R_NUM, R_GAIN_I, label="I-gain")
    # plt.plot(R_NUM, R_GAIN_D, label="D-gain")
    # plt.plot(R_NUM, R_GAIN_SUM, label="Gain sum")

    # plt.plot(R_NUM, THRUST_OUT, label="Thrust-row")
    # plt.plot(R_NUM, THRUST_OFFSETED, label="Thrust-offseted")
    plt.plot(R_NUM, R_THRUST, label="Thrust")

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
