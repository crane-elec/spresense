############################################################################
# externals/nrf52/LibIncludes.mk
#
#   Copyright 2022 Sony Semiconductor Solutions Corporation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name of Sony Semiconductor Solutions Corporation nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

ifeq ($(CONFIG_EXTERNALS_NRF52),y)
CFLAGS += -DBLE_STACK_SUPPORT_REQD -DNRF_SD_BLE_API_VERSION=$(BLE_API_VERSION) -DSVCALL_AS_NORMAL_FUNCTION -DBLE_SUPPORT_BLE5 -std=c99
CFLAGS += -DBLE_ENABLE_NORDIC_ORIGINAL
endif

ifeq ($(CONFIG_EXTERNALS_NRF52),y)
#NRF52_TOP := ./nRF5_SDK_17.1.0_ddde560
#NRF52_H_DIRS := $(sort $(dir $(shell find $(NRF52_TOP) -name "*.h")))
#NRF52_INCLUDES := $(addprefix -I, $(NRF52_H_DIRS))
#CFLAGS += $(INCLUDES) $(BLE_INCLUDES)
endif

