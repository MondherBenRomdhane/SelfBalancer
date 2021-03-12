# SelfBalancer
SelfBalancer Project

![ROB5](https://user-images.githubusercontent.com/45209418/110980777-83702600-8366-11eb-9b4a-7ef50c1355fd.png)


The Self Balancing Robot Project:
October 2019 - Ongoing
A camera vision based robot perpetually balancing on two wheels capable of navigation and path planning. This project was built from scratch and still a work in progress. It features multiple design layers.
Video:    https://www.youtube.com/channel/UC1xsE1nzPSHUBSwCJd2xYDg

Microcontroller & Sensor:
Firmware implemented on the STM32F3 Discovery Board running on top of FreeRTOS.
- GANTT diagram analysis with logic analyser to estimate internal tasks execution, priority and synchronisation.
- Developed a CLI (command line interface) via UART XBEE allowing to change some key parameters while running.
- Implementing and tuning balancer PID algorithm.
- IMU sensor interfacing accelerometer/Gyroscope data acquisition via SPI/I2C.
- Gyro offset calibration and drift/noise filtering.
- Kalman filter analysis and complementary filter implementation.
- Stepper motor driver developpement and speed curve linearisation.
- Server Side GUI using Labview to monitor in real time IMU values and plotting curves.

Keywords: Embedded C, FreeRTOS, STM32, IMU, Sensor-Fusion, Timers, Interrupts, Multithreading, debugging, LabView.

High Level Processing:
Firmware implemented on the Raspberry PI. Featuring mainly motion algorithms, computer vision and device interfaces.
- Bluetooth controller (DS4) interface with raspberryPI and python (PYgames)
- QT5 and XServer application context setup (YOCTO proof of concept).
- Custom boot sequence & Daemon configuration.
- ROS and OpenCV integration.
- CV Segmentation & feature extraction algorithms.

Keywords: Linux, Python, OpenCV, ROS, RaspberryPI, QT5, Bash.




