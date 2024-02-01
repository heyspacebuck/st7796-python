# Python ST7796

Python library to control ST7796 TFT LCD displays. Forked from Pimoroni's Raspi implementation for ST7789, incorporating changes from [this Arduino driver](https://github.com/prenticedavid/Adafruit_ST7796S_kbv)

# Installation

````bash
sudo apt update
sudo apt install python3-rpi.gpio python3-spidev python3-pip python3-pil python3-numpy
````

You will also need to make sure I2C(really?) and SPI are enabled in raspi-config (`sudo raspi-config`) - you can find them under Interface Options. 

# Licensing & History

This library is a modification of one that had this history:

> This library is a modification of a modification of code originally written by Tony DiCola for Adafruit Industries, and modified to work with the ST7735 by Clement Skau.
> 
> To create this ST7789 driver, it has been hard-forked from st7735-python which was originally modified by Pimoroni to include support for their 160x80 SPI LCD breakout.
> 
> ## Modifications include:
> 
> * PIL/Pillow has been removed from the underlying display driver to separate concerns- you should create your own PIL image and display it using `display(image)`
> * `width`, `height`, `rotation`, `invert`, `offset_left` and `offset_top` parameters can be passed into `__init__` for alternate displays
> * `Adafruit_GPIO` has been replaced with `RPi.GPIO` and `spidev` to closely align with our other software (IE: Raspberry Pi only)
> * Test fixtures have been added to keep this library stable
> 
> Pimoroni invests time and resources forking and modifying this open source code, please support Pimoroni and open-source software by purchasing products from us, too!
> 
> Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!
> 
> Modified from 'Modified from 'Adafruit Python ILI9341' written by Tony DiCola for Adafruit Industries.' written by Clement Skau.
> 
> MIT license, all text above must be included in any redistribution
