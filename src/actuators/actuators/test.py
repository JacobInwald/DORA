import smbus
import time
from .control import master_send, master_receive

# Define constants
START_BYTE = 0x0F
I2C_ADDRESS = 0x07

# Initial parameters
sv = [1500, 1500, 1500, 1500, 0, 0]  # servo positions: 0 = Not Used
sd = [5, 10, -5, -15, 20, -20]  # servo sweep speed/direction
# left and right motor speed from -255 to +255 (negative value = reverse)
lmspeed, rmspeed = 0, 0
# how much to change left/right motor speed each loop (use for motor testing)
ldir, rdir = 5, 5
lmbrake, rmbrake = 0, 0  # left and right motor brake (non-zero value = brake)
devibrate = 50  # time delay after impact to prevent false re-triggering due to chassis vibration
sensitivity = 50  # threshold of acceleration/deceleration required to register as an impact
lowbat = 550  # adjust to suit your battery: 550 = 5.50V
i2caddr = 7  # default I2C address of T'REX is 7. If this is changed, the T'REX will automatically store new address in EEPROM
i2cfreq = 0  # I2C clock frequency. Default is 0=100kHz. Set to 1 for 400kHz

# Initialize I2C bus
bus = smbus.SMBus()  # Use the appropriate I2C bus number
time.sleep(1)  # Wait for the bus to stabilize


def loop():
    global lmspeed, rmspeed, lmbrake, rmbrake, ldir, rdir

    master_send(bus, I2C_ADDRESS, 0x00, lmspeed, rmspeed,
                lmbrake, rmbrake, sv[0], sv[1], sv[2], sv[3], sv[4], sv[5], devibrate, sensitivity, lowbat, i2caddr, i2cfreq)
    time.sleep(0.05)

    # Code to test motors and sweep servos
    lmspeed += ldir
    if abs(lmspeed) > 240:
        ldir = -ldir  # reverse direction

    rmspeed += rdir
    if abs(rmspeed) > 240:
        rdir = -rdir  # reverse direction

    lmbrake = int(abs(lmspeed) > 235)  # test left motor brake
    rmbrake = int(abs(rmspeed) > 235)  # test right motor brake

    for i in range(6):  # sweep servos
        if sv[i] != 0:  # a value of 0 indicates no servo attached
            sv[i] += sd[i]  # update servo position to create sweeping motion
            if sv[i] > 2000 or sv[i] < 1000:
                sd[i] = -sd[i]  # reverse direction of servo if limit reached


while True:
    loop()
