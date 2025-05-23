import time

import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

# 128x32 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, gpio=1) # setting gpio to 1 is hack to avoid platform detection

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height-padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0

# Load default font.
font = ImageFont.load_default()

print("before while")

while True:
    # Draw a black filled box to clear the image.
    draw.rectangle((0,0,width,height), outline=0, fill=0)

    # Shell scripts for system monitoring from here : https://unix.stackexchange.com/questions/119126/command-to-display-memory-usage-disk-usage-and-cpu-load
    cmd = "hostname -I | cut -d\' \' -f1"
    IP = subprocess.check_output(cmd, shell = True )
    cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
    CPU = subprocess.check_output(cmd, shell = True )
    cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
    MemUsage = subprocess.check_output(cmd, shell = True )
    cmd = "free -m | awk 'NR==3{printf \"Swap: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
    SwapUsage = subprocess.check_output(cmd, shell = True )
    cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
    Disk = subprocess.check_output(cmd, shell = True )
    cmd = "date"
    Date = subprocess.check_output(cmd, shell = True )

    # Write two lines of text.
    draw.text((x, top),       "IP: " + str(IP.decode('utf-8')),  font=font, fill=255)
    draw.text((x, top+8),     str(CPU.decode('utf-8')), font=font, fill=255)
    draw.text((x, top+16),    str(MemUsage.decode('utf-8')),  font=font, fill=255)
    draw.text((x, top+24),    str(SwapUsage.decode('utf-8')),  font=font, fill=255)
    draw.text((x, top+32),    str(Disk.decode('utf-8')),  font=font, fill=255)
    # draw.text((x, top+40),    str(Date.decode('utf-8')),  font=font, fill=255)
    # Display image.
    disp.image(image)
    disp.display()
    time.sleep(.1)
