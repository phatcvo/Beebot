# Prerequisites:
  - ROS Kinetic or Melodic
  - C/C++ Compiler: GCC 5.4.0 or MSVC 14.0
  - C++11

# Building:
sudo chmod o+rw xsens_mti_driver
  cd ../lxscontroller && make clean && make
  cd ../xscommon && make clean && make
  cd ../xstypes   && make clean && make
  cd <your_ws>
  catkin_make clean
  catkin_make

# Running:
  - Configure your MTi device to output desired data (e.g. for display example - orientation output) via MT Manager

   - Launch the Xsens MTi driver from your catkin workspace:


    $ roslaunch xsens_mti_driver xsens_mti_node.launch

  or with rviz visualization:

    $ roslaunch xsens_mti_driver display.launch


# Notes:
  - ROS timestamps
      The data messages from devices are time stamped on arrival in the ROS driver.
      When collecting data at higher rates, eg 100 Hz, the times between reads can differ from the configured output rate in the device.
      This is caused by possible buffering in the USB/FTDI driver.

      For instance:
      10 us, 10 us, 10 us, 45 ms, 10 us, 10 us, 10 us, 39 ms, 10 us, etc.
      instead of
      10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, etc.

      Work-around: for accurate and stable time stamp information, users can make use of the sensor reported time stamp (/imu/time_ref) instead.

# Troubleshooting
  - The Mti1 (Motion Tracker Development Board) is not recognized.

      Support for the Development Board is present in recent kernels. (Since June 12, 2015).
      If your kernel does not support the Board, you can add this manually

      $ sudo /sbin/modprobe ftdi_sio
      $ echo 2639 0300 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id


  - The device is recognized, but I cannot ever access the device -

      Make sure you are in the correct group (often dialout or uucp) in order to
      access the device. You can test this with

          $ ls -l /dev/ttyUSB0
          crw-rw---- 1 root dialout 188, 0 May  6 16:21 /dev/ttyUSB0
          $ groups
          dialout audio video usb users plugdev

      If you aren't in the correct group, you can fix this in two ways.

      1. Add yourself to the correct group
          You can add yourself to it by using your distributions user management
          tool, or call

              $ sudo usermod -G dialout -a $USER

          Be sure to replace dialout with the actual group name if it is
          different. After adding yourself to the group, either relogin to your
          user, or call

              $ newgrp dialout

          to add the current terminal session to the group.

      2. Use udev rules
          Alternatively, put the following rule into /etc/udev/rules.d/99-custom.rules

              SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ACTION=="add", GROUP="$GROUP", MODE="0660"

          Change $GROUP into your desired group (e.g. adm, plugdev, or usb).


  - The device is inaccessible for a while after plugging it in -

      When having problems with the device being busy the first 20 seconds after
      plugin, purge the modemmanager application.

  - RViz doesn't show an MTi model.

      It is a known issue with urdfdom in ROS Melodic. A workaround is to unset/modify the LC_NUMERIC environment variable:

      $ LC_NUMERIC="en_US.UTF-8"
