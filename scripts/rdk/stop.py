import smbus2
import time

def clear_once():
    bus = smbus2.SMBus(5)
    for i in range(5):
        bus.write_i2c_block_data(0x34, 0x33, [0, 0, 0, 0])
        bus.write_i2c_block_data(0x34, 0x1F, [0, 0, 0, 0])
        time.sleep(0.05)
    sp = bus.read_i2c_block_data(0x34, 0x33, 4)
    print("reg:", [x if x < 128 else x - 256 for x in sp])
    bus.close()

for attempt in range(3):
    try:
        clear_once()
        print("STOPPED")
        break
    except Exception as e:
        print("i2c err:", e)
        time.sleep(0.2)
