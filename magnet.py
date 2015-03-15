
import smbus
import math

bus = smbus.SMBus(1)
address = 0x1e


def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr + 1)
    val = (high << 8) + low
    return val


def read_word_2c(adr):
    val = read_word(adr)
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val


def configure():
    bus.write_byte_data(address, 0, 0b01110000)     # Set to 8 samples @ 15Hz
    bus.write_byte_data(address, 1, 0b00100000)     # 1.3 gain LSb / Gauss 1090 (default)
    bus.write_byte_data(address, 2, 0b00000000)     # Continuous sampling


def read():
    scale = 0.92

    x_out = read_word_2c(3) * scale
    y_out = read_word_2c(7) * scale
    z_out = read_word_2c(5) * scale

    bearing = math.atan2(y_out, x_out)
    if bearing < 0:
        bearing += 2 * math.pi

    bearing = math.degrees(bearing)
    return bearing
