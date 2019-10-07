cmake_minimum_required(VERSION 3.13.1)
# Copyright (c) 2019  STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#

#set(SHIELD x_nucleo_iks01a2)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)

project(tof_on_display)

target_sources(app PRIVATE src/main.c)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
