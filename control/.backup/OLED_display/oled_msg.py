import time
import rospy
from std_msgs.msg import String
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import subprocess

# 128x64 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=1, gpio=1)  # setting gpio to 1 is hack to avoid platform detection

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
draw.rectangle((0, 0, width, height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height - padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0

# Load default font.
font = ImageFont.load_default()

# Initialize the ROS node
rospy.init_node('system_monitor_display', anonymous=True)

# Variable to hold ROS message data
ros_message = "ROS ready!!!!"

# Callback function to handle incoming messages
def string_callback(msg):
    global ros_message
    ros_message = msg.data
    rospy.loginfo(f"ROS: {ros_message}")
    print(f"ROS: {ros_message}")

# Subscribe to a ROS topic (replace 'your_ros_topic' with the actual topic name)
rospy.Subscriber('/oled_msg', String, string_callback)

print("I2C OLED")

# Track previous values for system metrics
previous_ip = ""
previous_cpu = ""
previous_mem = ""
previous_swap = ""
previous_disk = ""
previous_date = ""

# Use a frequency for the loop
rate = rospy.Rate(1)  # 1 Hz, adjust as needed

while not rospy.is_shutdown():
    print(f"ROS: {ros_message}")

    # Shell scripts for system monitoring
    cmd = "hostname -I | cut -d\' \' -f1"
    IP = subprocess.check_output(cmd, shell=True).decode('utf-8').strip()
    
    cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
    CPU = subprocess.check_output(cmd, shell=True).decode('utf-8').strip()

    cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
    MemUsage = subprocess.check_output(cmd, shell=True).decode('utf-8').strip()

    cmd = "free -m | awk 'NR==3{printf \"Swap: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
    SwapUsage = subprocess.check_output(cmd, shell=True).decode('utf-8').strip()

    cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
    Disk = subprocess.check_output(cmd, shell=True).decode('utf-8').strip()

    cmd = "date"
    Date = subprocess.check_output(cmd, shell=True).decode('utf-8').strip()

    # Clear the image if any value has changed
    update_display = False

    # Check if any system metric has changed
    if IP != previous_ip or CPU != previous_cpu or MemUsage != previous_mem or \
       SwapUsage != previous_swap or Disk != previous_disk or Date != previous_date or \
       ros_message != "":
        update_display = True

    if update_display:
        # Draw a black filled box to clear the image.
        draw.rectangle((0, 0, width, height), outline=0, fill=0)

        # Write system monitoring data on the display
        draw.text((x, top), "IP: " + IP, font=font, fill=255)
        draw.text((x, top + 8), CPU, font=font, fill=255)
        draw.text((x, top + 16), MemUsage, font=font, fill=255)
        draw.text((x, top + 24), SwapUsage, font=font, fill=255)
        draw.text((x, top + 32), Disk, font=font, fill=255)

        # Display ROS message if it is available
        draw.text((x, top + 40), ros_message, font=font, fill=255)

        # Display image on OLED
        disp.image(image)
        disp.display()

        # Update the previous values
        previous_ip = IP
        previous_cpu = CPU
        previous_mem = MemUsage
        previous_swap = SwapUsage
        previous_disk = Disk
        previous_date = Date

    # Sleep for the time required by the rate
    rate.sleep()
