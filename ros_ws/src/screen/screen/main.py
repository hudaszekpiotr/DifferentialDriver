import rclpy
from rclpy.node import Node
from interfaces.msg import WheelsVelocities
from sensor_msgs.msg import Imu
import math
import sys
import time
import Adafruit_SSD1306
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import subprocess
from std_msgs.msg import String

class ScreenNode(Node):
    def __init__(self):
        super().__init__('screen')
        self.subscription = self.create_subscription(String, 'info_to_display', self.new_info_to_display_callback,10)
        self.subscription  # prevent unused variable warning

        # Raspberry Pi pin configuration:
        RST = None  # on the PiOLED this pin isnt used

        # 128x32 display with hardware I2C:
        # disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)

        # 128x64 display with hardware I2C:
        self.disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)

        # Note you can change the I2C address by passing an i2c_address parameter like:
        # disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST, i2c_address=0x3C)

        # Alternatively you can specify an explicit I2C bus number, for example
        # with the 128x32 display you would use:
        # disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST, i2c_bus=2)

        # Initialize library.
        self.disp.begin()

        # Clear display.
        self.disp.clear()
        self.disp.display()

        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        width = self.disp.width
        height = self.disp.height
        self.image = Image.new('1', (width, height))

        # Get drawing object to draw on image.
        draw = ImageDraw.Draw(self.image)

        # Draw a black filled box to clear the image.
        draw.rectangle((0, 0, width, height), outline=0, fill=0)

        # First define some constants to allow easy resizing of shapes.
        self.padding = -2
        self.top = self.padding
        self.bottom = height - self.padding
        # Move left to right keeping track of the current x position for drawing shapes.
        self.x = 0
        # Load default font.
        self.font = ImageFont.load_default()

    def new_info_to_display_callback(self, msg):
        #text = msg.data
        # Draw a black filled box to clear the image.
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)

        text = "test"

        # Write two lines of text.

        self.draw.text((self.x, self.top), text, font=self.font, fill=255)
        self.draw.text((self.x, self.top + 8), text, font=self.font, fill=255)
        self.draw.text((self.x, self.top + 16), text, font=self.font, fill=255)
        self.draw.text((self.x, self.top + 25), text, font=self.font, fill=255)

        # Display image.
        self.disp.image(self.image)
        self.disp.display()
        #time.sleep(.1)


def main(args=None):
    rclpy.init(args=args)

    imu_node = ScreenNode()

    rclpy.spin(imu_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()













