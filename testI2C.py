import smbus
import time

bus = smbus.SMBus(1)

address = 0x32


while True:
    print(bus.read_byte_data(address, 0x04))
