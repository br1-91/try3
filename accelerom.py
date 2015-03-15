
import math
import time
import smbus

bus = smbus.SMBus(1)
address=0x53

POWER_CTL = 0x2d
DATA_FORMAT = 0x31
FIFO_CTL = 0x38

AFS_2g = 0
AFS_4g = 1
AFS_8g = 2
AFS_16g = 3

ACCEL_START_BLOCK = 0x32
ACCEL_XOUT_H = 1
ACCEL_XOUT_L = 0
ACCEL_YOUT_H = 3
ACCEL_YOUT_L = 2
ACCEL_ZOUT_H = 5
ACCEL_ZOUT_L = 4

ACCEL_SCALE = 0.004 # Always set to this as we are using FULL_RES

afs_scale=AFS_2g

raw_accel_data = [0, 0, 0, 0, 0, 0]

accel_raw_x = 0
accel_raw_y = 0
accel_raw_z = 0

accel_scaled_x = 0
accel_scaled_y = 0
accel_scaled_z = 0

pitch = 0.0
roll = 0.0


def twos_complement(high, low):
    val = (high << 8) + low
    if (val >= 0x8000):
        return -((0xffff - val) + 1)
    else:
        return val


def configure():
    # Wake up the device
    bus.write_byte_data(address, POWER_CTL, 0b00001000)

    # Set data to FULL_RES and user defined scale
    data_format = 0b00001000 | afs_scale
    bus.write_byte_data(address, DATA_FORMAT, data_format)

    # Disable FIFO mode
    bus.write_byte_data(address, FIFO_CTL, 0b00000000)
    return


def read():

    raw_accel_data = bus.read_i2c_block_data(address, ACCEL_START_BLOCK, 6)
    time_r=time.time()
    accel_raw_x = twos_complement(raw_accel_data[ACCEL_XOUT_H], raw_accel_data[ACCEL_XOUT_L])
    accel_raw_y = twos_complement(raw_accel_data[ACCEL_YOUT_H], raw_accel_data[ACCEL_YOUT_L])
    accel_raw_z = twos_complement(raw_accel_data[ACCEL_ZOUT_H], raw_accel_data[ACCEL_ZOUT_L])

    accel_scaled_x = accel_raw_x * ACCEL_SCALE
    accel_scaled_y = accel_raw_y * ACCEL_SCALE
    accel_scaled_z = accel_raw_z * ACCEL_SCALE

    return(accel_scaled_x,accel_scaled_y,accel_scaled_z,time_r)


#--------------------------------------------
