
import math
import smbus

bus = smbus.SMBus(1)
address = 0x2b

vout_addr = 0x30
altitude_addr = 0x31
servo1_addr = 0x27

vsupply = 255  #max is 5volts=255 out of the ADC
density = 1.225


def twos_complement(high, low):
    val = (high << 8) + low
    return val


def read_velocity():
    try:
        vout = bus.read_byte_data(address, vout_addr)
        dpressure = ((vout / vsupply) - 0.5) * 5
        vel = math.sqrt((2 * dpressure) / density)
        err = False
    except IOError:
        err = True
        vel = 0
    return vel, err


def read_altitude():
    try:
        alt1 = bus.read_byte_data(address, altitude_addr)         #pulse width in 2 byte(tmr1 high and low) in number of instuctions
        alt2 = bus.read_byte_data(address, altitude_addr + 1)       #with prescaler this will have to be converted in time
        alt = twos_complement(alt1, alt2)
        err = False
    except IOError:
        alt = 0
        err = True
    return alt, err


def control_servo(servo1, servo2, servo3, servo4, servo5, servo6):
    try:
        bus.write_byte_data(address, servo1_addr, servo1)
        bus.write_byte_data(address, servo1_addr + 1, servo2)
        bus.write_byte_data(address, servo1_addr + 2, servo3)
        bus.write_byte_data(address, servo1_addr + 3, servo4)
        bus.write_byte_data(address, servo1_addr + 4, servo5)
        bus.write_byte_data(address, servo1_addr + 5, servo6)
        err = False
    except IOError:
        err = True
    return err
