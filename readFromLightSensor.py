import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
bus.write_byte_data(0x44, 0x01, 0x05)

time.sleep(1)

print("Reading colour values and displaying them in a new window\n")


def getAndUpdateColour():
    while True:

        # Read the data from the sensor
        # Insert code here
        # 6 bytes

        data = bus.read_i2c_block_data(0x44, 0x09, 6)

        # Convert the data to green, red and blue int values
        # Insert code here
        green = 256*data[1] + data[0]
        red = 256*data[3] + data[2]
        blue = 256*data[5] + data[4]

        # Output data to the console RGB values
        # Uncomment the line below when you have read the red, green and blue values

        print("RGB(%d %d %d)" % (red, green, blue))

        time.sleep(2)

        return (red, green, blue)


def print_color_name(red, green, blue):
    """
    Takes 16-bit RGB values and prints out the corresponding color name, or an
    approximate color if the input RGB values don't match one of the predefined colors.
    """
    # Convert 16-bit RGB values to 8-bit values by shifting right by 8 bits.
    r = red >> 8
    g = green >> 8
    b = blue >> 8

    # Check if the RGB values match one of the predefined color values.
    if (r, g, b) == (255, 255, 255):
        print("White")
    elif (r, g, b) == (0, 0, 0):
        print("Black")
    elif (r, g, b) == (255, 0, 0):
        print("Red")
    elif (r, g, b) == (0, 255, 0):
        print("Green")
    elif (r, g, b) == (0, 0, 255):
        print("Blue")
    elif (r, g, b) == (255, 255, 0):
        print("Yellow")
    elif (r, g, b) == (255, 0, 255):
        print("Magenta")
    elif (r, g, b) == (0, 255, 255):
        print("Cyan")
    else:
        # If the RGB values don't match any of the predefined colors, approximate the color.
        if r > g and r > b:
            print("Approximate color: Red")
        elif g > r and g > b:
            print("Approximate color: Green")
        elif b > r and b > g:
            print("Approximate color: Blue")
        elif r == g == b:
            print("Approximate color: Gray")
        else:
            print("Hard to approximate")


red, green, blue = getAndUpdateColour()

print("\n")
print_color_name(red, green, blue)
