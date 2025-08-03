# Automatic Turkish Ice Cream Man (ATICM)

Design using servo motors and computer vision model to track approaching hands and dodge them.

<img src=assets/full.png alt="full Machine" width="500"/>

[Demo video](https://www.youtube.com/shorts/_B0X4ihoTbQ)


## Features:
- 3D printed case
- Full 2D range of motion
- CV Hand Tracking
- All firmware in microPython

## Hardware:
Everything fits together using 4 servo screws and hot glue, used to connect the motors to the case.

It has 3 separate printed pieces: the bottom section, the pan section, and the tilt section. These sections fit together using the screws mentioned above. These designs were created on Onshape, with fit tests done for each of the parts.

The rod is a quarter inch round dowel, attached to the CAD model with hot glue. The motors are similarly connected to the CAD with the four servo screws and hot glue.

<img src=assets/hardware1.png alt="Hardware Part 1" width="500"/>
<img src=assets/hardware2.png alt="Hardware Part 2" width="500"/>

## Electronics:
Below is our circuit schematic. We designed the schematic to streamline the necessary transfer of power and signals to each of the parts. Basic connection techniques like soldering and breadboards/perfboards were used. After a little research, we found that using electrical tape improved safety and conductivity. In the schematic, you can see the specific parts we used.

<img src=assets/schematic.png alt="Schematic" width="500"/>

## Computer Vision (CV):
We built a computer vision model that processes video inputs from a HD webcam using OpenCV and then processes that video stream with Google's MediaPipe library, allowing us to detect and track hands in each of the frames. This information is then sent along to the firmware to control the motors.

## Firmware:
This hackpad uses microPython firmware for everything, from moving the rod to pulling from the CV program. The firmware codes in connecting frequencies and angle ratios for each of the servo motors to ensure python commands from the CV model are able to be transferred to real-life motor outputs.

## BOM:

Here is everything you need to make this design

- 1/4 in wooden dowel
- 4x Servo Screws
- 3x MG996R Servo Motors
- 1x Orpheus Pico
- 1x HD USB Webcam
- 1x Case (3 printed parts)
