
import smbus
import time

bus = smbus.SMBus(1)    # or bus = smbus.SMBus(1) if you have a revision 2 board
address = 0x77

oss = 3                                 # Ultra high resolution
temp_wait_period = 0.004
pressure_wait_period = 0.0255           # Conversion time

#--------------------defs-------------------


def read_byte(adr):
    return bus.read_byte_data(address, adr)


def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr + 1)
    val = (high << 8) + low
    return val


def read_word_2c(adr):
    val = read_word(adr)
    if val >= 0x8000:
        return -((0xffff - val) + 1)
    else:
        return val


def write_byte(adr, value):
    bus.write_byte_data(address, adr, value)


def twos_compliment(val):
    if val >= 0x8000:
        return -((0xffff - val) + 1)
    else:
        return val


def get_word(array, index, twos):
    val = (array[index] << 8) + array[index + 1]
    if twos:
        return twos_compliment(val)
    else:
        return val


def calculate(calibration_data, temp_raw, pressure_raw):
    ac1 = calibration_data[0]
    ac2 = calibration_data[1]
    ac3 = calibration_data[2]
    ac4 = calibration_data[3]
    ac5 = calibration_data[4]
    ac6 = calibration_data[5]
    b1 = calibration_data[6]
    b2 = calibration_data[7]
    mb = calibration_data[8]
    mc = calibration_data[9]
    md = calibration_data[10]

    # This code is a direct translation from the datasheet
    # and should be optimised for real world use

    #Calculate temperature
    x1 = ((temp_raw - ac6) * ac5) / 32768
    x2 = (mc * 2048) / (x1 + md)
    b5 = x1 + x2
    t = (b5 + 8) / 16

    # Now calculate the pressure
    b6 = b5 - 4000
    x1 = (b2 * (b6 * b6 >> 12)) >> 11
    x2 = ac2 * b6 >> 11
    x3 = x1 + x2
    b3 = (((ac1 * 4 + x3) << oss) + 2) >> 2

    x1 = (ac3 * b6) >> 13
    x2 = (b1 * (b6 * b6 >> 12)) >> 16
    x3 = ((x1 + x2) + 2) >> 2
    b4 = ac4 * (x3 + 32768) >> 15
    b7 = (pressure_raw - b3) * (50000 >> oss)
    if b7 < 0x80000000:
        p = (b7 * 2) / b4
    else:
        p = (b7 / b4) * 2
    x1 = (p >> 8) * (p >> 8)
    x1 = (x1 * 3038) >> 16
    x2 = (-7357 * p) >> 16
    p = p + ((x1 + x2 + 3791) >> 4)
    a = 44330.0 * (1.0 - pow(p / 101325.0, (1.0 / 5.255)))
    return t, p, a


def calibration():
    # The sensor has a block of factory set calibration values we need to read
    # these are then used in a length calculation to get the temperature and pressure
    calibration_d = bus.read_i2c_block_data(address, 0xAA, 22)
    ac1 = get_word(calibration_d, 0, True)
    ac2 = get_word(calibration_d, 2, True)
    ac3 = get_word(calibration_d, 4, True)
    ac4 = get_word(calibration_d, 6, False)
    ac5 = get_word(calibration_d, 8, False)
    ac6 = get_word(calibration_d, 10, False)
    b1 = get_word(calibration_d, 12, True)
    b2 = get_word(calibration_d, 14, True)
    mb = get_word(calibration_d, 16, True)
    mc = get_word(calibration_d, 18, True)
    md = get_word(calibration_d, 20, True)
    calibration_data = [ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md]
    return calibration_data

    #-----------------------------start---------------------------------------


def read(calibration_data):
    # Read raw temperature
    write_byte(0xF4, 0x2E)                      # Tell the sensor to take a temperature reading
    time.sleep(temp_wait_period)                # Wait for the conversion to take place
    temp_raw = read_word_2c(0xF6)

    write_byte(0xF4, 0x34 + (oss << 6))         # Tell the sensor to take a pressure reading
    time.sleep(pressure_wait_period)            # Wait for the conversion to take place
    pressure_raw = ((read_byte(0xF6) << 16) + (read_byte(0xF7) << 8) + (read_byte(0xF8))) >> (8 - oss)

    temperature, pressure, altitude = calculate(calibration_data, temp_raw, pressure_raw)

    #print(time.time(), temperature / 10., pressure / 100.)
    #time.sleep(1)
    return temperature, pressure, altitude
