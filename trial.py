__author__ = 'Bruno'
#turn on
import imu
import gpsdata
import pic16
import math
import time
from serial import Serial


def pid_loop(y_c, y_now, kp, ki, kd, limit, ts, tau, integrator, differentiator, error_0):

    error = y_c - y_now
    integrator = integrator + (ts*(error + error_0)/2)
    differentiator = ((2 * tau - ts) / (2 * tau + ts)) * differentiator + 2 / (2 * tau + ts) * (error - error_0)
    error_0 = error

    out = (kp * error) + (ki * integrator) + (kd * differentiator)
    if out > limit:
        out = limit
    elif out < -limit:
        out = -limit

    if ki != 0:
        out_unsat = (kp * error) + (ki * integrator) + (kd * differentiator)
        integrator = integrator + ts * (out - out_unsat)/ki
    return out, integrator, differentiator, error_0

alpha = 0
def aileron_to_bearing_rq():
    #calculate a1 and a2
    CL = cl_data[1][alpha + 10]
    #CL2 = weight * 9.81 / (0.5 * density * velocity * velocity * wing_area)
    dCl = (cl_data[1][alpha + 10] - cl_data[1][alpha + 11])
    Clalpha_w = (dCl * 180 / math.pi) / (1 + ((dCl * 180 / math.pi) / (math.pi * aspect_ratio)))  #per radian

    clp = -Clalpha_w / 6
    cnp = -CL / 8
    cpp = ro3 * clp + ro4 * cnp

    clda = (2 * Clalpha_w * 0.35 * chord / (wing_area * wing_span)) * ((0.9 * 0.9 - 0.08 * 0.08) / 2)
    cnda = 2 * (-0.165) * cl0 * clda
    cpda = ro3 * clda + ro4 * cnda

    a1 = -0.5 * density * velocity * wing_area * wing_span * cpp * wing_span / 2
    a2 = 0.5 * density * velocity * velocity * wing_area * wing_span * cpda
    #calculate
    #set:
    damping_theta = 5
    damping_x = 5
    ki_theta = 0.1
    damax = 45
    emax = 10
    omegan_theta = math.sqrt(abs(a2) * damax / emax)
    omegan_x = omegan_theta / 5
    #gains
    kp_theta = math.copysign(1, a2) * damax / emax
    kd_theta = ((2 * damping_theta * omegan_theta) - a1) / a2
    kp_x = 2 * damping_x * omegan_x * grspeed / 9.81
    ki_x = omegan_x * omegan_x * grspeed / 9.81
    tau = 0.00001
    int_r, diff_r, err0_r, int_a, diff_a, err0_a = 0, 0, 0, 0, 0, 0
    (roll_angle_rq, int_r, diff_r, err0_r) = pid_loop(bearing_rq, bearing, kp_x, ki_x, 0, 45, ts, tau,
                                                      int_r, diff_r, err0_r)
    (aileron_angle, int_a, diff_a, err0_a) = pid_loop(roll_angle_rq, accel_roll, kp_theta, ki_theta,
                                                      kd_theta, 45, ts, tau, int_a, diff_a, err0_a)
    return (aileron_angle)

clalphaw = 0
tau = 0
def pitch_to_altitude():
    #longitudinal
    cmalpha = clalphaw * ((xcg - xac) / c) + cmalpha_fus - (1 * tail_volume_ratio * clalpha_t * (1 - de_dalpha))
    cmde = -eff_factor * tail_volume_ratio * dclt_dde
    a_theta2 = (-density * velocity * velocity * chord * wing_area * cmalpha) / (2 * Iy)
    a_theta3 = (-density * velocity * velocity * chord * wing_area * cmde) / (2 * Iy)
    #set
    demax = 45
    emaxp = 10
    damping_pitch = 3
    damping_h = 3
    omegan_pitch = math.sqrt(a_theta2 + (a_theta3 * demax / emaxp))
    omegan_h = omegan_pitch / 5
    kp_pitch = math.copysign(1, a_theta3) * demax / emaxp
    kdc_pitch = kp_pitch * a_theta3 / (a_theta2 + (kp_pitch * a_theta3))
    ki_h = omegan_h * omegan_h / (kdc_pitch * velocity)
    kp_h = 2 * damping_h * omegan_h / (kdc_pitch * velocity)

    int_h, diff_h, err0_h = 0, 0, 0
    (pitch_rq, int_h, diff_h, err0_h) = pid_loop(altitude_rq, altitude, kp_h, 0, ki_h, 10, ts, tau,
                                                 int_h, diff_h, err0_h)
    return pitch_rq


def elevator_to_pitch_rq():
    global int_p, diff_p, err0_p
    #longitudinal
    cmalpha = clalphaw * ((xcg - xac) / c) + cmalpha_fus - (1 * tail_volume_ratio * clalpha_t * (1 - de_dalpha))
    cmq = -2 * eff_factor * clalpha_t * tail_volume_ratio * tail_arm / chord
    cmde = -eff_factor * tail_volume_ratio * dclt_dde
    a_theta1 = (-density * velocity * velocity * chord * wing_area * cmq * chord) / (2 * 2 * Iy * velocity)
    a_theta2 = (-density * velocity * velocity * chord * wing_area * cmalpha) / (2 * Iy)
    a_theta3 = (-density * velocity * velocity * chord * wing_area * cmde) / (2 * Iy)
    #set
    demax = 45
    emaxp = 10
    damping_pitch = 3
    omegan_pitch = math.sqrt(a_theta2 + (a_theta3 * demax / emaxp))
    kp_pitch = math.copysign(1, a_theta3) * demax / emaxp
    kd_pitch = (2 * damping_pitch * omegan_pitch - a_theta1) / a_theta3

    (elevator_angle, int_p, diff_p, err0_p) = pid_loop(pitch_rq, gyro_pitch, kp_pitch, kd_pitch, 0, 45, ts, tau,
                                                       int_p, diff_p, err0_p)
    return elevator_angle


def engine_to_velocity_rq():
    kp_v = 0.1
    kd_v = 0.1
    ki_v = 0.1
    global int_v, diff_v, err0_v
    (engine_angle, int_v, diff_v, err0_v) = pid_loop(velocity_rq, velocity, kp_v, kd_v, ki_v, 45, ts, tau,
                                                     int_v, diff_v, err0_v)
    return engine_angle


def pitch_to_velocity():
    return


def update_data():
    global temperature, altitude, bearing, latitude, latNS, longitude, lonEW, status, altitude_GPS, grspeed, gyro_roll, \
        gyro_pitch, gyro_yaw, accel_roll, accel_pitch, velocity, altitude_sr, time0, time1, ts

    (temperature, altitude, bearing, latitude, latNS, longitude, lonEW, status, altitude_GPS, grspeed, gyro_roll,
        gyro_pitch, gyro_yaw, accel_roll, accel_pitch, velocity, altitude_sr) = imu.read_all()
    time1 = time.time()
    ts = time1 - time0
    time0 = time1
    return

#send back data received for check (not enough time to implement)
#port.writeline(maprcv)
#port.writeline(confrcv)
#received = False
#time0 = time.time()
#while not received:
#    rcv = port.readline()
#    if len(rcv) != 0:
#        #scan line
#        if rcv[0:5] == '$NO':
#            co = False
#            received = True
#        elif rcv[0:5] == '$YES':
#            co = True
#            received = True
#    time1 = time.time()
#    if time1 - time0 > 1:
#        port.writeline(tx)
#        time0 = time1
#    received = m and c
#wait for reply, as above if it takes too long send again
#if data is good then configuration is finished, $MAP and$CONFIG can be sent at any time from control station


#wait for mission start, keep sending data from sensors, and wait for reply from control

#call sensor read and send made above, add if for start mission


#at start:
#  calculate the desired heading in relation to north, substract from current heading to calculate
#  the correction needed.
#  set speed required at take off speed (speed control calculations PID or whatever) keep the speed
#  set control surfaces at centre position
#  keep checking roll and yaw and pitch, and set them at the required values (pitch will have to be corrected
#     by the angle of attack needed for takeoff)(function calculate angle of attack from Cl required through airfoil
#     data(dCl/dalpha)
#  also keep checking heading(in 2 dimensions) from GPS data(2D), compass(2D), gyro data(yaw), and keep correcting
#     it by using the vertical stabilizer or wheels(or both)(done in function in path following.take-off.heading)
#  also check altitude from pressure sensor and ultrasound sensor to check when takeoff is achieved. if it is, keep
#     the altitude at a meter or two above the path
#  when at the end of take off runway , if the plane did not take off yet, act on the elevator to change the angle of
#     attack. if the plane has already taken off, switch to the path following program for normal flight


#-------------------#define constants--------------------------------------------------------


density = 1.225     #kg/m^3


chord = 0.3         #m                          #aircraft constants
wing_span = 1.8     #m
wing_area = chord * wing_span  #m^2
aspect_ratio = wing_span / chord




tail_chord_r = 0.185
tail_chord_t = 0.125
tail_span = 0.64
tail_area = (tail_chord_r + tail_chord_t) * tail_span / 2
tail_arm = 0.8
tail_volume_ratio = tail_arm * tail_area / (wing_area * chord)
eff_factor = 1
clalpha_t = 0.10966     #per degree. assumed 2pi per radian
tau_t = 0.49            #from book p 64
dclt_dde = clalpha_t * tau_t

clalpha=clalphaw

de_dalpha = 2 * clalpha / (math.pi() * aspect_ratio)
cmalpha_fus = ((0.08 * 0.08 * 0.3 * 1.3) + (0.07 * 0.07 * 0.7 * 0.35 * (1 - de_dalpha) / 0.35)) / \
              (36.5 * wing_area * chord)
xac = chord / 4
xcg = 0.15          #needs changing

weight = 3.6  #kg

Ix = 0.3
Iy = 0.1
Iz = 0.3
Ixz = 0.05

ro = (Ix * Iz) - (Ixz * Ixz)
ro3 = Iz / ro
ro4 = Ixz / ro

cl_data = [range(-10, 15, 1), [-0.2, -0.4, -0.5, -0.4, -0.3, -0.2, -0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9,
                               1.0, 1.1, 1.2, 1.3, 1.35, 1.36, 1.38, 1.4, 1.38, 1.35]]
cl0 = 0.4


velocity_toff = 5   #m/s                    #performance
alpha_toff = 2   #degrees


#---------------------------------------------------------------------------


gpsdata.uart_config()           #configure pins


imu.config()                    #configure imu(with try except)

while True:
    #get data from imu(try except)
    (temperature, altitude, bearing, latitude, latNS, longitude, lonEW, status, altitude_GPS, grspeed, gyro_roll,
     gyro_pitch, gyro_yaw, accel_roll, accel_pitch, velocity, altitude_sr) = imu.read_all()


    #send data to control base (NEED to correct values of string lenght)
    port = Serial("/dev/ttyAMA0", 57600)
    port.close()
    port.open()
    tx = "$DATA," + "%02d" % temperature + "," + "%06.2f" % altitude + "," + "%05.2f" % bearing + "," \
         + "%08.3f" % latitude + "," + latNS + "," + "%08.3f" % longitude + "," + lonEW + "," + status + "," \
         + "%05.2f" % altitude_GPS + "," + "%05.2f" % grspeed + "," + "%05.2f" % gyro_roll + "," \
         + "%05.2f" % gyro_pitch + "," + "%05.2f" % gyro_yaw + "," + "%05.2f" % accel_roll + "," \
         + "%05.2f" % accel_pitch + "," + "05.2f" % velocity + "," + "%05.2f" % altitude_sr     #+ "," +

    port.write(tx)
