
import accelerom
import gyro2
import pressure_sensor
import gpsdata
import magnet
import pic16

import smbus
import math
import time


def lpf(x0, x1, time_diff):
    alpha = math.exp(-1000 * time_diff)
    x = alpha * x0 + (1 - alpha) * x1
    return x


def roll_s(x, y, z):
    #Returns the rotation around the X axis in radians
    try:
        dist = math.sqrt((x * x) + (z * z))
    except ValueError:
        dist = -math.sqrt(-((x * x) + (z * z)))
    return math.degrees(math.atan2(y, dist))


def pitch_s(x, y, z):
    #Returns the rotation around the X axis in radians
    try:
        dist = math.sqrt((y * y) + (z * z))
    except ValueError:
        dist = -math.sqrt(-((y * y) + (z * z)))
    return math.degrees(math.atan2(x, dist))


#-------------configure sensors--------------


def config():
    global cal_data
    accelerom.configure()
    gyro2.configure()
    magnet.configure()
    cal_data = pressure_sensor.calibration()
    return


#-------------initialize imu data------------
roll = 0
pitch = 0
yaw = 0

gyro_roll = 0
gyro_pitch = 0
gyro_yaw = 0

#take initial read of accelerometer set initial roll and pitch with this reading
(accel_x0, accel_y0, accel_z0) = accelerom.read()
accel_roll = roll_s(accel_x0, accel_y0, accel_z0)
accel_pitch = pitch_s(accel_x0, accel_y0, accel_z0)

#initial read of gyro
(gyro_x0, gyro_y0, gyro_z0) = gyro2.read()
time0 = time.time()


def read_pitch_roll_yaw():
    global time0
    global gyro_roll
    global gyro_pitch
    global gyro_yaw
    global accel_roll
    global accel_pitch

    (accel_x, accel_y, accel_z) = accelerom.read()                          #read accelerometer data
    (gyro_x, gyro_y, gyro_z) = gyro2.read()                                 #read gyro data
    time_now = time.time()                                                  #read time of reading
    time_diff = time_now - time0                                            #calculate time difference

    gyro_x = lpf(gyro_x0, gyro_x, time_diff)                                #low pass filter of inputs
    gyro_y = lpf(gyro_y0, gyro_y, time_diff)
    gyro_z = lpf(gyro_z0, gyro_z, time_diff)

    accel_x = lpf(accel_x0, accel_x, time_diff)
    accel_y = lpf(accel_y0, accel_y, time_diff)
    accel_z = lpf(accel_z0, accel_z, time_diff)

    d_rx = (gyro_x + gyro_x0) * time_diff / 2                               #use time difference to calculate change in angle
    d_ry = (gyro_y + gyro_y0) * time_diff / 2                               #use time difference to calculate change in angle
    d_rz = (gyro_z + gyro_z0) * time_diff / 2                               #use time difference to calculate change in angle

    accel_roll = roll_s(accel_x, accel_y, accel_z)                          #calculate roll from accelerometer data
    accel_pitch = pitch_s(accel_x, accel_y, accel_z)                        #calculate pitch from accelerometer data

    gyro_roll = gyro_roll + d_rx - 0.00213                                  #calculate roll from gyro data(with an error correction)
    gyro_pitch = gyro_pitch + d_ry - 0.00213
    gyro_yaw = gyro_yaw + d_rz - 0.00213

    #roll_comb = (accel_roll * 0) + (gyro_roll) * 1                         #gives an approximation using both sensors(not used)
    time0 = time_now
    return gyro_roll, gyro_pitch, gyro_yaw, accel_roll, accel_pitch


def read_all():

    (gyro_roll, gyro_pitch, gyro_yaw, accel_roll, accel_pitch) = read_pitch_roll_yaw()
    (temperature, pressure, altitude) = pressure_sensor.read(cal_data)
    (bearing) = magnet.read()
    (velocity) = pic16.read_velocity()
    (altitude_sr) = pic16.read_altitude()
    (latitude, latNS, longitude, lonEW, status, nsatellites, altitude_GPS, grspeed) = gpsdata.read()

    return temperature, altitude, bearing, latitude, latNS, longitude, lonEW, status, altitude_GPS, grspeed,\
           gyro_roll, gyro_pitch, gyro_yaw, accel_roll, accel_pitch, velocity, altitude_sr
