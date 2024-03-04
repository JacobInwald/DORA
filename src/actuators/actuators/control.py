# Ported from original Arduino code: https://cdn.sparkfun.com/datasheets/Robotics/T_REX_I2C_tester.zip

import smbus


def master_send(bus, sbyte, pfreq, lspeed, lbrake, rspeed, rbrake, sv0, sv1, sv2, sv3, sv4, sv5, dev, sens, lowbat, i2caddr, i2cfreq):
    bus = smbus.SMBus(1)  # Use the appropriate I2C bus number

    bus.write_byte(i2caddr, sbyte)  # start byte
    bus.write_byte(i2caddr, pfreq)  # pwm frequency

    bus.write_i2c_block_data(
        i2caddr, 0, [lspeed >> 8, lspeed & 0xFF])  # left motor speed
    bus.write_byte(i2caddr, lbrake)  # left motor brake

    bus.write_i2c_block_data(
        i2caddr, 2, [rspeed >> 8, rspeed & 0xFF])  # right motor speed
    bus.write_byte(i2caddr, rbrake)  # right motor brake

    for i, sv in enumerate([sv0, sv1, sv2, sv3, sv4, sv5]):
        bus.write_i2c_block_data(
            i2caddr, 4 + i * 2, [sv >> 8, sv & 0xFF])  # servo positions

    bus.write_byte(i2caddr, dev)  # devibrate
    bus.write_i2c_block_data(
        i2caddr, 16, [sens >> 8, sens & 0xFF])  # impact sensitivity
    bus.write_i2c_block_data(
        i2caddr, 18, [lowbat >> 8, lowbat & 0xFF])  # low battery voltage

    bus.write_byte(i2caddr, i2cfreq)  # I2C clock frequency

    print("Master Command Data Packet Sent")

    if i2cfreq == 0:
        bus.write_byte_data(i2caddr, 0, 72)  # default I2C clock is 100kHz
    else:
        bus.write_byte_data(i2caddr, 0, 12)  # changes the I2C clock to 400kHz


def master_receive(bus, I2C_ADDRESS, START_BYTE):
    # Error Checking
    while True:
        # Request 24 bytes from device at address i2caddr
        data = bus.read_i2c_block_data(I2C_ADDRESS, 0, 24)
        if len(data) < 24:
            print("Waiting for slave to send data.", end="")
            continue
        elif data[0] != START_BYTE:  # Start byte check
            print("Wrong Start Byte:", data[0])
            return
        else:
            break

    # Read Data
    print("Slave Error Message:", data[1])
    print("Battery Voltage:", (data[2] * 256 + data[3]) / 10, "V")
    print("Left Motor Current:", data[4] * 256 + data[5], "mA")
    print("Left Motor Encoder:", data[6] * 256 + data[7])
    print("Right Motor Current:", data[8] * 256 + data[9], "mA")
    print("Right Motor Encoder:", data[10] * 256 + data[11])
    print("X-axis:", data[12] * 256 + data[13])
    print("Y-axis:", data[14] * 256 + data[15])
    print("Z-axis:", data[16] * 256 + data[17])
    print("X-delta:", data[18] * 256 + data[19])
    print("Y-delta:", data[20] * 256 + data[21])
    print("Z-delta:", data[22] * 256 + data[23])
    print("\n\n\n")
