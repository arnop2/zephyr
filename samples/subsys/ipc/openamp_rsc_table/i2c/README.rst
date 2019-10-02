.. _openAMP_sample:

OpenAMP Sample Application
##########################

Overview
********

This application demonstrates how to use OpenAMP to share an I2C bus between
Zephyr and a remote processor. It is designed to implementa proxy to mux I2C
communication 
demonstrate how to integrate OpenAMP with Zephyr both from a build perspective
and code.  Currently this integration is specific to the LPC54114 SoC.

requested Hardware
*************************

- STM32MP115c-dk2 board
- Nucleo shield ISK01A2
- 2 SSD1306 displays connected on I2C arduino connectors:

  - display @0x3C: managed by Zephyr
  - display @0x3D: managed by Remote processor

Building the application
*************************

.. zephyr-app-commands::
   :zephyr-app: samples/subsys/ipc/openamp_rsc_table/i2c
   :board: stm32mp157c_dk2
   :goals: debug

Copy the binary file on the SDCard

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

.. code-block:: console

  root@stm32mp1:~# i2cdetect -l
  i2c-3   i2c             i2c-0-mux (chan_id 0)                   I2C adapter
  i2c-1   i2c             STM32F7 I2C(0x5c002000)                 I2C adapter
  i2c-2   i2c             RPMSG I2C adapter                       I2C adapter
  i2c-0   i2c             STM32F7 I2C(0x40012000)                 I2C adapter
  root@stm32mp1:~# i2cdetect -y 2
       0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
  00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
  10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
  20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
  30: -- -- -- -- -- -- -- -- -- -- -- -- -- 3d -- -- 
  40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
  50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
  60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
  70: -- -- -- -- -- -- -- --                         
