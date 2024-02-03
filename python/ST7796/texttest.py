#!/usr/bin/env python3
import sys

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import ST7796

# Create ST7796 LCD display class.
disp = ST7796.ST7796(
    height=480,
    width=320,
    rotation=0,
    port=0,
    cs=0,  # BG_SPI_CS_BACK or BG_SPI_CS_FRONT
    rst=27,
    dc=22,
    spi_speed_hz=16000000,
    offset_left=0,
    offset_top=0
)

# Initialize display.
disp.begin()

WIDTH = disp.width
HEIGHT = disp.height


# Clear the display to a black background.
# Can pass any tuple of red, green, blue values (from 0 to 255 each).
# Get a PIL Draw object to start drawing on the display buffer.
img = Image.new('RGB', (WIDTH, HEIGHT), color=(0, 0, 0))

draw = ImageDraw.Draw(img)

# Load a TTF font.
# Some other nice fonts to try: http://www.dafont.com/bitmap.php
font = ImageFont.truetype('ChiKareGo.ttf', 16)

draw.text((10, 10), "May I take your photo?", font=font, fill=(220, 220, 255))



# Define a function to create rotated text.  Unfortunately PIL doesn't have good
# native support for rotated fonts, but this function can be used to make a
# text image and rotate it so it's easy to paste in the buffer.
def draw_rotated_text(image, text, position, angle, font, fill=(255, 255, 255)):
    # Get rendered font width and height.
    draw = ImageDraw.Draw(image)
    width, height = draw.textsize(text, font=font)
    # Create a new image with transparent background to store the text.
    textimage = Image.new('RGBA', (width, height), (0, 0, 0, 0))
    # Render the text.
    textdraw = ImageDraw.Draw(textimage)
    textdraw.text((0, 0), text, font=font, fill=fill)
    # Rotate the text image.
    rotated = textimage.rotate(angle, expand=1)
    # Paste the text into the image, using it as a mask for transparency.
    image.paste(rotated, position, rotated)


# Write two lines of white text on the buffer, rotated 90 degrees counter clockwise.
#draw_rotated_text(img, 'Hello World!', (0, 0), 90, font, fill=(255, 255, 255))
#draw_rotated_text(img, 'This is a line of text.', (10, HEIGHT - 10), 0, font, fill=(255, 255, 255))

# Write buffer to display hardware, must be called to make things visible on the
# display!
disp.display(img)

print("hellote")