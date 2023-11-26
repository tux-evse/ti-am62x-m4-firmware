# Copyright (C) 2015-2022 IoT.bzh Company
# Author: Fulup Ar Foll <fulup@iot.bzh>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#

# Makefile config for ti-cgt-armllvm_3.2.0.LTS with out-of-tree source files


# Project sources and compiler location
PROJECT= $(abspath .)
TOOLS_PATH?= $(abspath ../Ti)

# syconfig, ti-clang & ti-sdk version
SYSCONFIG_VERSION=1.18
CGT_ARM_CLANG_VERSION=3.2.0.LTS
MCU_PLUS_SDK_VERSION=09_00_00_19

# project conf file
export PRJ_NAME=pionix-firmware
export LNK_FILES_common= $(PROJECT)/etc/linker.cmd
export PRJ_SYSCONFIG= $(PROJECT)/etc/project.syscfg

# debug, release
export PROFILE?=release
export DESTDIR?=$(abspath Firmware)


export DEVICE= am62x
export DEVICE_TYPE?=GP

export CGT_TI_ARM_CLANG_PATH= $(TOOLS_PATH)/ti-cgt-armllvm_$(CGT_ARM_CLANG_VERSION)
export MCU_PLUS_SDK_PATH= $(TOOLS_PATH)/mcu_plus_sdk_am62x_$(MCU_PLUS_SDK_VERSION)
export SYSCFG_PATH= $(TOOLS_PATH)/ti-sysconfig_$(SYSCONFIG_VERSION)
SYSCFG_CLI_PATH?= $(SYSCFG_PATH)
SYSCFG_NODE= $(SYSCFG_PATH)/nodejs/node
SYSCFG_NWJS= $(SYSCFG_PATH)/nw/nw
SYSCFG_SDKPRODUCT= $(MCU_PLUS_SDK_PATH)/.metadata/product.json


# default syscfg CPU to use, options on am62x are m4fss0-0
ifeq ($(DEVICE),$(filter $(DEVICE), am62x))
  SYSCFG_DEVICE = AM62x
  SYSCFG_CPU = m4fss0-0
endif

# Linux only
export MKDIR=mkdir -p
export RMDIR=rm -rf
export RM=rm -f
export COPY=cp
export TOUCH=touch
export PATHSEP=/
export CHMOD=chmod
export PYTHON=python3

SYSCFG_CLI_PATH ?= $(SYSCFG_PATH)
SYSCFG_NODE = $(SYSCFG_PATH)/nodejs/node
SYSCFG_NWJS = $(SYSCFG_PATH)/nw/nw
SYSCFG_SDKPRODUCT=$(MCU_PLUS_SDK_PATH)/.metadata/product.json