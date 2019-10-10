.. _openAMP_sample:

OpenAMP Sample Application
##########################

Overview
********

This application demonstrates how to use rpmsg on a UART serial link
to expose to the remote  several services.
This demo is only a prove of concept as not implement the rpmsg through the 
OpenAMP library. The proper way to implement this exemple should be to update
the OpenAMP library to add the UART backend in adittion to the virtio one.

requested Hardware
******************

- l475 iot1 disco board
- SSD1306 display connected to the iot board on I2C arduino connectors:
- STM32MP115c-dk2 board connected to the UART arduno connector (RX and TX as to be inversed between disco_l475_iot1 and STM32MP115c-dk2 arduino connectors)

STM32MP115c-dk2 board firmware
*******************************

The linux firmware has to be updated with the Kernel image based on following
branch: https://github.com/arnopo/tree/ELCE_demos

Building the application
*************************

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/ipc/openamp_o_uart
   :board: disco_l475_iot1
   :goals: debug

Flash the board 
.. code-block:: console

  west flash

Open a serial Linux terminal (minicom, putty, etc.) and connect the board with the
following settings:

- Speed: 115200
- Data: 8 bits
- Parity: odd
- Stop bits: 1


Load and start the firmware:

.. code-block:: console

  echo -n <firmware_name.elf> > /sys/class/remoteproc/remoteproc0/firmware
  echo start >/sys/class/remoteproc/remoteproc0/state


This is the Linux console:

Information on console related to the rpmsg channels creation:

.. code-block:: console

  root@stm32mp1:~# [   33.394946] Weston already configured on pixman
  [   99.196227] uart_rpmsg serial1-0: creating channel rpmsg-button addr 0x0
  [   99.202103] input: rpmsg virtual button as /devices/virtual/input/input1
  [   99.262179] uart_rpmsg serial1-0: creating channel rpmsg-vl5310x addr 0x1
  [   99.491174] uart_rpmsg serial1-0: creating channel rpmsg-tty-channel addr 0x2
  [   99.497605] rpmsg_tty serial1-0.rpmsg-tty-channel.-1.2: new channel: 0x402 -> 0x2 : ttyRPMSG0

Read the distance value provided by the Tof sensor on thel475 iot1 disco board:

.. code-block:: console

  root@stm32mp1:~# cat /sys/bus/iio/devices/iio\:device0/in_distance_raw 
  8190

Detect proxy event provided by the Tof sensor on the l475 iot1 disco board:

.. code-block:: console

  root@stm32mp1:~# evtest
  No device specified, trying to scan all of /dev/input/event*
  Available devices:
  /dev/input/event0:      pmic_onkey
  /dev/input/event1:      rpmsg virtual button
  Select the device event number [0-1]: 1
  Input driver version is 1.0.1
  Input device ID: bus 0x0 vendor 0x0 product 0x0 version 0x0
  Input device name: "rpmsg virtual button"
  Supported events:
    Event type 0 (EV_SYN)
    Event type 1 (EV_KEY)
      Event code 256 (BTN_0)
  Properties:
  Testing ... (interrupt to exit)
  Event: time 1548326283.037408, type 1 (EV_KEY), code 256 (BTN_0), value 1
  Event: time 1548326283.037408, -------------- SYN_REPORT ------------
  Event: time 1548326283.813357, type 1 (EV_KEY), code 256 (BTN_0), value 0
  Event: time 1548326283.813357, -------------- SYN_REPORT ------------

End the demo application on the l475 iot1 disco board:

.. code-block:: console

  root@stm32mp1:~# echo stop >/dev/ttyRPMSG0
  root@stm32mp1:~# [  897.118198] uart_rpmsg serial1-0: destroying channel rpmsg-button addr 0x0
  [  897.155425] rpmsg_button serial1-0.rpmsg-button.-1.0: rpmsg button device is removed
  [  897.247167] uart_rpmsg serial1-0: destroying channel rpmsg-vl5310x addr 0x1
  [  897.253175] rpmsg_vl5310x serial1-0.rpmsg-vl5310x.-1.1: rpmsg vl5310x device is removed
  [  897.376171] uart_rpmsg serial1-0: destroying channel rpmsg-tty-channel addr 0x2
  [  897.382921] rpmsg_tty serial1-0.rpmsg-tty-channel.-1.2: rpmsg tty device 0 is removed
