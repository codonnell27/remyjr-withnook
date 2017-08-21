# remyjr-withnook
Remy Jr autonomous robot using raspberry pi, nook, and various sensors

## Launching 

If launching on the original remyJr, these are the only necessary steps. If launching on a different system, Dependencies & Setup (see below) has a list of packages that need to be on the raspberry pi and nook, while System Dependent Variables (see below) has a list of variables in the remyJr package code that might need to be changed.

 There are two launch files:
 
 driveremy.launch - Launch file for driving without sounds.
 
 loud_drive.launch - Launch file for driving with sounds. Before launching the output speaker must be manually set. For example:
 
 ```bash
pacmd list-sources | grep -e 'name:'
  name: <alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo.monitor>
pacmd set-default-source alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo.monitor
pacmd list-sinks | grep -e 'name:'
  name: <alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo>
pacmd set-default-sink alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo

 ```

To launch one of these files (launch from the nook):
```bash
source devel/setup.bash
roslaunch remyJr driveremy.launch
```


## Dependencies & Setup

The remyJr package needs to be on both the nook and the raspberry pi.

It depends on the ros packages rospy, std-msgs, and sound_play. It also has the following non-ros dependencies for the sensors:

##### BNO055 

This package needs to be on the raspberry pi.

The package used can be found here: https://github.com/adafruit/Adafruit_Python_BNO055    

Adafruit's installation/setup instructions can be found here: https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black/overview

##### PCA9685 

This package needs to be on the raspberry pi.

The package used can be found here: https://github.com/adafruit/Adafruit_Python_PCA9685  

Adafruit's setup/installation instructions can be found here: https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi/overview

##### MCP3008  

This package needs to be on the raspberry pi.

The package used can be found here: https://github.com/adafruit/Adafruit_Python_MCP3008

Adafruit's setup/installation instructions can be found here: https://learn.adafruit.com/raspberry-pi-analog-to-digital-converters/mcp3008

In this project, the software SPI connection from Adafruit's setup instructions was used.

##### VL53L0X 

This package needs to be on the raspberry pi.

The package used can be found here, as well as installation instructions: https://github.com/johnbryanmoore/VL53L0X_rasp_python  

The raspberry pi initially had trouble locating the vl53l0x_python.so file. Creating the directory /home/pi/bin and copying the vl53l0x_python.so file into it fixed the problem.

##### Sounds

Sounds used in the project are stored in the misc folder of remyJr. They must be moved to the sound_play/sounds in the sound_play package on the nook to be played. 

##### Raspberry Pi
 
 SSH and i2c must be enabled on the raspberry pi.

## System Dependent Variables

If this is build on another system, the following variables and files must be changed to reflect the new system's connections:

remyJr/launch/driveremy.launch - The address, username, address to env.sh, and password of the raspberry pi may need to be updated.

remyJr/misc/env.sh - ROS_IP and ROS_MASTER_URI must be changed to reflect the address of the nook. ROS_HOSTNAME must be changed to reflect the address of the pi. This file must also be moved to the the devel directory in the workspace directory on the raspberry pi (ex: ~/catkin_ws/devel).

remyJr/scripts/tof_watch.py - f_shutdown_pin and b_shutdown_pin must be changed to the shutdown pins of the time-of-flight sensors. The floor distance variables may also need to be changed depending on the location of the tof sensors.

remyJr/scripts/sonar_watch.py - The software SPI connection pins must be change to reflect the connections between the raspberry pi and the ADC. ADC channel numbers must be changed to reflect channel connections of the sonars and orientation of the array. Finally, sonar_array_trig_pin must be changed to reflect the raspberry pi pin it is connected to.

remyJr/scripts/imu_watch.py - The reset_pin, level_grav_x, and jump_threshold must be changed to reflect the new setup. 

remyJr/scripts/control_motors.py - drivePin and turnPin must be changed to reflect the connections between the motor drivers and the PCA9658.

remyJr/scripts/bump_watch.py - The software SPI connection pins must be change to reflect the connections between the raspberry pi and the ADC. ADC channel numbers must be changed to reflect channel connections of the Joystick.

## Electrical Components used:

1 Raspberry Pi - used to read sensor data and drive motors

1 Nook - used for decision making and graphic sensor and motion data, since the raspberry pi has limited processing power

8 LV-MaxSonar_EZ1 Sonar sensors - used for object detection and to inform descisions as to which direction the robot should drive

2 MCP3008 ADC - One is used in conjunction with the sonars to convert data into a form readable by the raspberry pi; the other does the same for the joystick

2 VL53L0X time-of-flight sensors - used for cliff detection directly in front and behind the robot

1 BNO055 IMU with sensor fustion - used to determine if the robot is level

1 PCA9685 Servo Motor Driver - used to fake an RC signal that is used to command the motor drivers

1 Sunfounder Joystick - used in conjunction with bump skirt to detect if the robot has hit anything

1 USB speaker (optional) - used to make a squeaky toy noise when the robot hits something

## Circuit 

![remyjr circuit diagram](https://github.com/codonnell27/remyjr-withnook/blob/master/remyjr.JPG)

The joystick pictured above is not the joystick used, but it has the same connections.

The sonar array must be arranged so that the sonars facing front, left, right, and back are triggered by the same signal and the sonars facing front right, front left, back left, and back right are triggered by a different signal. This means that no sonar with a purple wire to it's RX pin in the above image should be next to another sonar with a purple RX wire. The same holds true for pink RX wires.

The imu should be placed so that the breakout board is level with the base of the robot and longest sides are on the left and right. (There doesn't seem to be any axis markings on the chip...)

If the raspberry pi is not connected to power via micro usb but the motor drivers are powered, the raspberry pi will power itself via gpio, which leaved it open to frying itself. So be careful when testing the motor drivers that the raspberry pi and PCM9685 is not connected to the motor drivers!
