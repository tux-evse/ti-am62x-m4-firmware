export # Copyright (C) 2015-2022 IoT.bzh Company
# Author: Fulup Ar Foll <fulup@iot.bzh>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#

include etc/imports.mak

all:
	@echo $(MAKE)  -C src all
	$(MAKE)  -C src all

scrub:
	$(MAKE) -s -C src scrub

clean:
	$(MAKE) -s -C src clean

help:

help:
	@echo  Notes,
	@echo  - Use -j to invoke parallel builds
	@echo  - Use PROFILE=debug or PROFILE=release [default] to build in debug or release profile
	@echo  .
	@echo  Overall build targets,
	@echo  ======================
	@echo  $(MAKE) all   # build evrything
	@echo  $(MAKE) clean # delete's tmp files for current profile
	@echo  $(MAKE) scrub # delete's all tmp files and folders for all profiles
	@echo  $(MAKE) syscfg-gui # configure sysconfig
	@echo  $(MAKE) devconfig  # configure device

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device $(SYSCFG_DEVICE) --context $(SYSCFG_CPU)

devconfig:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.json --device $(SYSCFG_DEVICE) --context $(SYSCFG_CPU) --output devconfig/ $(MCU_PLUS_SDK_PATH)/devconfig/devconfig.syscfg

.PHONY: syscfg-gui
.PHONY: devconfig
.PHONY: all clean
.PHONY: libs libs-clean libs-scrub
