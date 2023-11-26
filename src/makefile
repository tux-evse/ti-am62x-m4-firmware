# Copyright (C) 2015-2022 IoT.bzh Company
# Author: Fulup Ar Foll <fulup@iot.bzh>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#


include $(MCU_PLUS_SDK_PATH)/imports.mak
include $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.mak

OUTNAME:=$(PRJ_NAME).$(PROFILE).out

CG_TOOL_ROOT=$(CGT_TI_ARM_CLANG_PATH)
CC=$(CG_TOOL_ROOT)/bin/tiarmclang
LNK=$(CG_TOOL_ROOT)/bin/tiarmclang
STRIP=$(CG_TOOL_ROOT)/bin/tiarmstrip
OBJCOPY=$(CG_TOOL_ROOT)/bin/tiarmobjcopy

ConfigName:=$(PROFILE)

BOOTIMAGE_NAME:=$(PRJ_NAME).$(PROFILE).appimage
BOOTIMAGE_NAME_XIP:=$(PRJ_NAME).$(PROFILE).appimage_xip
BOOTIMAGE_NAME_SIGNED:=$(PRJ_NAME).$(PROFILE).appimage.signed
BOOTIMAGE_RPRC_NAME:=$(PRJ_NAME).$(PROFILE).rprc
BOOTIMAGE_RPRC_NAME_XIP:=$(PRJ_NAME).$(PROFILE).rprc_xip
BOOTIMAGE_RPRC_NAME_TMP:=$(PRJ_NAME).$(PROFILE).rprc_tmp
BOOTIMAGE_NAME_HS:=$(PRJ_NAME).$(PROFILE).appimage.hs
BOOTIMAGE_NAME_HS_FS:=$(PRJ_NAME).$(PROFILE).appimage.hs_fs
TARGETS := $(BOOTIMAGE_NAME)
ifeq ($(DEVICE_TYPE), HS)
   TARGETS += $(BOOTIMAGE_NAME_HS)
endif

FILES_cpp_common := \
	entry.cpp \
	main.cpp \
	iec61851_fsm.cpp \
    control_pilot.cpp \
	power_switch.cpp  \
	rpmsg.cpp \
	sampler.cpp \

FILES_common := \
	low_to_high.pb.c \
	high_to_low.pb.c \
	pb_common.c \
	pb_decode.c \
	pb_encode.c \
	ti_drivers_config.c \
	ti_drivers_open_close.c \
	ti_board_config.c \
	ti_board_open_close.c \
	ti_dpl_config.c \
	ti_pinmux_config.c \
	ti_power_clock_config.c \

FILES_PATH_cpp_common = \
	. \
	lib \
	bsp_drivers \

FILES_PATH_common = \
	. \
	pb_codec \
	pb_codec/nanopb \
	generated \

INCLUDES_common := \
	-I${CG_TOOL_ROOT}/include/c \
	-I${MCU_PLUS_SDK_PATH}/source \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/FreeRTOS-Kernel/include \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CM4F \
	-I${MCU_PLUS_SDK_PATH}/source/kernel/freertos/config/am62x/m4f \
	-Ibsp_drivers \
	-Ilib \
	-Ipb_codec \
	-Igenerated \

DEFINES_common := \
	-DSOC_AM62X \

CFLAGS_common := \
	-mcpu=cortex-m4 \
	-mfloat-abi=hard \
	-mthumb \
	-Wall \
	-Werror \
	-g \
	-Wno-gnu-variable-sized-type-not-at-end \
	-Wno-unused-function \

CFLAGS_cpp_common := \
	-Wno-c99-designator \
	-Wno-extern-c-compat \
	-Wno-c++11-narrowing \
	-Wno-reorder-init-list \
	-Wno-deprecated-register \
	-Wno-writable-strings \
	-Wno-enum-compare \
	-Wno-reserved-user-defined-literal \
	-Wno-unused-const-variable \
	-x c++ \

CFLAGS_debug := -D_DEBUG_=1

CFLAGS_release := -Os


LIBS_PATH_common = \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/board/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/sciserver/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/rm_pm_hal/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/sciclient_direct/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/self_reset/lib \
	-Wl,-i${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/dm_stub/lib \
	-Wl,-i${CG_TOOL_ROOT}/lib \

LIBS_common = \
	-lfreertos.am62x.m4f.ti-arm-clang.${ConfigName}.lib \
	-ldrivers.am62x.m4f.ti-arm-clang.${ConfigName}.lib \
	-lboard.am62x.m4f.ti-arm-clang.${ConfigName}.lib \
	-llibc.a \
	-llibsysbm.a \

LFLAGS_common = \
	-Wl,--ram_model \
	-Wl,--reread_libs \


LIBS_NAME = \
	freertos.am62x.m4f.ti-arm-clang.${ConfigName}.lib \
	drivers.am62x.m4f.ti-arm-clang.${ConfigName}.lib \
	board.am62x.m4f.ti-arm-clang.${ConfigName}.lib \
	libc.a \
	libsysbm.a \

LIBS_PATH_NAME = \
	${MCU_PLUS_SDK_PATH}/source/kernel/freertos/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/lib \
	${MCU_PLUS_SDK_PATH}/source/board/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/sciserver/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/rm_pm_hal/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/sciclient_direct/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/self_reset/lib \
	${MCU_PLUS_SDK_PATH}/source/drivers/device_manager/dm_stub/lib \
	${CG_TOOL_ROOT}/lib \

FILES := $(FILES_common) $(FILES_$(PROFILE))
CPPFILES := $(FILES_cpp_common) $(FILES_cpp_$(PROFILE))
ASMFILES := $(ASMFILES_common) $(ASMFILES_$(PROFILE))
FILES_PATH := $(FILES_PATH_common) $(FILES_PATH_$(PROFILE))
CFLAGS := $(CFLAGS_common) $(CFLAGS_$(PROFILE))
CPPFILES_PATH := $(FILES_PATH_cpp_common) $(FILES_PATH_$(PROFILE))
CPPFLAGS := $(CFLAGS_common) $(CFLAGS_$(PROFILE)) $(CFLAGS_cpp_common)
DEFINES := $(DEFINES_common) $(DEFINES_$(PROFILE))
INCLUDES := $(INCLUDES_common) $(INCLUDE_$(PROFILE))
LIBS := $(LIBS_common) $(LIBS_$(PROFILE))
LIBS_PATH := $(LIBS_PATH_common) $(LIBS_PATH_$(PROFILE))
LFLAGS := $(LFLAGS_common) $(LFLAGS_$(PROFILE))
LNKOPTFLAGS := $(LNKOPTFLAGS_common) $(LNKOPTFLAGS_$(PROFILE))
LNK_FILES := $(LNK_FILES_common) $(LNK_FILES_$(PROFILE))

OBJDIR := obj/$(PROFILE)/
OBJS := $(FILES:%.c=%.obj)
OBJS += $(CPPFILES:%.cpp=%.obj)
OBJS += $(ASMFILES:%.S=%.obj)
DEPS := $(FILES:%.c=%.d)

vpath %.obj $(OBJDIR)
vpath %.cpp $(CPPFILES_PATH)
vpath %.c $(FILES_PATH)
vpath %.S $(FILES_PATH)
vpath %.lib $(LIBS_PATH_NAME)
vpath %.a $(LIBS_PATH_NAME)

$(OBJDIR)/%.obj %.obj: %.c
	@echo  Compiling: am62x:m4fss0-0:freertos:ti-arm-clang $(OUTNAME): $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $(DEFINES) -MMD -o $(OBJDIR)/$@ $<

$(OBJDIR)/%.obj %.obj: %.cpp
	@echo  Compiling: am62x:m4fss0-0:freertos:ti-arm-clang $(OUTNAME): $<
	$(CC) -c $(CPPFLAGS) $(INCLUDES) $(DEFINES) -MMD -o $(OBJDIR)/$@ $<

$(OBJDIR)/%.obj %.obj: %.S
	@echo  Compiling: am62x:m4fss0-0:freertos:ti-arm-clang $(LIBNAME): $<
	$(CC) -c $(CFLAGS) -o $(OBJDIR)/$@ $<

all: $(TARGETS)

SYSCFG_GEN_FILES=generated/ti_drivers_config.c generated/ti_drivers_config.h
SYSCFG_GEN_FILES+=generated/ti_drivers_open_close.c generated/ti_drivers_open_close.h
SYSCFG_GEN_FILES+=generated/ti_dpl_config.c generated/ti_dpl_config.h
SYSCFG_GEN_FILES+=generated/ti_pinmux_config.c generated/ti_power_clock_config.c
SYSCFG_GEN_FILES+=generated/ti_board_config.c generated/ti_board_config.h
SYSCFG_GEN_FILES+=generated/ti_board_open_close.c generated/ti_board_open_close.h

$(OUTNAME): syscfg $(SYSCFG_GEN_FILES) $(OBJS) $(LNK_FILES) $(LIBS_NAME)
	@echo  .
	@echo  Linking: am62x:m4fss0-0:freertos:ti-arm-clang $@ ...
	$(MKDIR) $(DESTDIR)
	$(LNK) $(LNKOPTFLAGS) $(LFLAGS) $(LIBS_PATH) -Wl,-m=$(basename $@).map -o $@ $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) $(LNK_FILES)
	$(RM) $(DESTDIR)/am62-mcu-m4f0_0-fw
	$(STRIP) -p $@ -o=$(DESTDIR)/am62-mcu-m4f0_0-fw
	@echo  Linking: am62x:m4fss0-0:freertos:ti-arm-clang $@ Done !!!
	@echo  .

clean: scrub
	@echo  Cleaning: am62x:m4fss0-0:freertos:ti-arm-clang $(OUTNAME) ...
	$(RMDIR) $(OBJDIR)
	$(RM) $(DESTDIR)/$(OUTNAME)
	$(RM) $(DESTDIR)/am62-mcu-m4f0_0-fw
	$(RM) $(BOOTIMAGE_NAME)
	$(RM) $(BOOTIMAGE_NAME_XIP)
	$(RM) $(BOOTIMAGE_NAME_SIGNED)
	$(RM) $(BOOTIMAGE_NAME_HS)
	$(RM) $(BOOTIMAGE_NAME_HS_FS)
	$(RM) $(BOOTIMAGE_RPRC_NAME)
	$(RM) $(BOOTIMAGE_RPRC_NAME_XIP)
	$(RMDIR) generated/

scrub:
	@echo  Scrubing: am62x:m4fss0-0:freertos:ti-arm-clang $(PRJ_NAME) ...
	$(RMDIR) obj
	$(RM) *.out
	$(RM) *.map
	$(RM) *.appimage*
	$(RM) *.rprc*
	$(RM) *.tiimage*
	$(RM) *.bin
	$(RMDIR) generated

$(OBJS): | $(OBJDIR)

$(OBJDIR):
	$(MKDIR) $@

.NOTPARALLEL:

.INTERMEDIATE: syscfg
$(SYSCFG_GEN_FILES): syscfg

syscfg: $(PRJ_SYSCONFIG)
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context m4fss0-0 --part Default --package ALW --output generated/ $(PRJ_SYSCONFIG)

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device AM62x --context m4fss0-0 --part Default --package ALW --output generated/  $(PRJ_SYSCONFIG)

#
# Generation of boot image which can be loaded by Secondary Boot Loader (SBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif
ifeq ($(OS),Windows_NT)
  BOOTIMAGE_CERT_GEN_CMD=powershell -executionpolicy unrestricted -command $(MCU_PLUS_SDK_PATH)/tools/boot/signing/x509CertificateGen.ps1
else
  BOOTIMAGE_CERT_GEN_CMD=$(MCU_PLUS_SDK_PATH)/tools/boot/signing/x509CertificateGen.sh
endif
BOOTIMAGE_TEMP_OUT_FILE=temp_stdout_$(PROFILE).txt

BOOTIMAGE_CERT_KEY=$(APP_SIGNING_KEY)

BOOTIMAGE_CORE_ID_a53ss0-0 = 0
BOOTIMAGE_CORE_ID_a53ss0-1 = 1
BOOTIMAGE_CORE_ID_a53ss1-0 = 2
BOOTIMAGE_CORE_ID_a53ss1-1 = 3
BOOTIMAGE_CORE_ID_r5fss0-0 = 4
BOOTIMAGE_CORE_ID_m4fss0-0 = 5
BOOTIMAGE_CORE_ID_hsm-m4fss0-0 = 6
SBL_RUN_ADDRESS=0x43C00000
SBL_DEV_ID=55

MULTI_CORE_IMAGE_GEN = $(SYSCFG_NODE) $(MCU_PLUS_SDK_PATH)/tools/boot/multicoreImageGen/multicoreImageGen.js
OUTRPRC_CMD = $(SYSCFG_NODE) $(MCU_PLUS_SDK_PATH)/tools/boot/out2rprc/elf2rprc.js
APP_IMAGE_SIGN_CMD = $(MCU_PLUS_SDK_PATH)/tools/boot/signing/appimage_x509_cert_gen.py
XIPGEN_CMD=$(MCU_PLUS_SDK_PATH)/tools/boot/xipGen/xipGen.out

MULTI_CORE_IMAGE_PARAMS = \
	$(BOOTIMAGE_RPRC_NAME)@$(BOOTIMAGE_CORE_ID_m4fss0-0) \

MULTI_CORE_IMAGE_PARAMS_XIP = \
	$(BOOTIMAGE_RPRC_NAME_XIP)@$(BOOTIMAGE_CORE_ID_m4fss0-0) \

$(BOOTIMAGE_NAME): $(OUTNAME)
	@echo  Boot image: am62x:m4fss0-0:freertos:ti-arm-clang $@ ...
	$(OUTRPRC_CMD) $(OUTNAME) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(COPY) $(BOOTIMAGE_RPRC_NAME) $(BOOTIMAGE_RPRC_NAME_TMP)
	$(RM) $(BOOTIMAGE_RPRC_NAME)
	$(XIPGEN_CMD) -i $(BOOTIMAGE_RPRC_NAME_TMP) -o $(BOOTIMAGE_RPRC_NAME) -x $(BOOTIMAGE_RPRC_NAME_XIP) --flash-start-addr 0x60000000 -v > $(BOOTIMAGE_TEMP_OUT_FILE)
	$(MULTI_CORE_IMAGE_GEN) --devID $(SBL_DEV_ID) --out $(BOOTIMAGE_NAME) $(MULTI_CORE_IMAGE_PARAMS) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(MULTI_CORE_IMAGE_GEN) --devID $(SBL_DEV_ID) --out $(BOOTIMAGE_NAME_XIP) $(MULTI_CORE_IMAGE_PARAMS_XIP) >> $(BOOTIMAGE_TEMP_OUT_FILE)
# Sign the appimage for HS-FS using appimage signing script
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --authtype 2 --key $(APP_SIGNING_KEY) --output $(BOOTIMAGE_NAME_HS_FS)
	$(RM) $(BOOTIMAGE_RPRC_NAME_TMP)
	$(RM) $(BOOTIMAGE_TEMP_OUT_FILE)
	@echo  Boot image: am62x:m4fss0-0:freertos:ti-arm-clang $@ Done !!!
	@echo  .
	@echo  Boot image: am62x:m4fss0-0:freertos:ti-arm-clang $(BOOTIMAGE_NAME_HS_FS) Done !!!
	@echo  .

$(BOOTIMAGE_NAME_HS): $(BOOTIMAGE_NAME)
ifeq ($(DEVICE_TYPE), HS)
# Sign the appimage using appimage signing script
ifeq ($(ENC_ENABLED),no)
	@echo Boot image signing: Encryption is disabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --authtype 2 --key $(APP_SIGNING_KEY) --output $(BOOTIMAGE_NAME_HS)
else
	@echo Boot image signing: Encryption is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --authtype 2 --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --output $(BOOTIMAGE_NAME_HS)
	$(RM) $(BOOTIMAGE_NAME)-enc
endif
	@echo  Boot image: am62x:m4fss0-0:freertos:ti-arm-clang $(BOOTIMAGE_NAME_HS) Done !!!
	@echo  .
endif
-include $(addprefix $(OBJDIR)/, $(DEPS))
