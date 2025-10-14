# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Akashi Temac Ethernet device driver.
#

# Exclusion if the driver is integrated into the kernel source tree
ifeq ($(CONFIG_AKASHI_KBUILD), y)
	ccflags-y += -include $(KBUILD_PATH)/include/generated/autoconf.h
endif

ccflags-y += -DLIB_SWITCH_ZYNQ_PLATFORM -DSWITCH_LIB_LINUX -D__AUD_ZYNQ_KERNEL__

obj-$(CONFIG_AKASHI_TEMAC) += akashi-temac.o

akashi-temac-$(CONFIG_AKASHI_TEMAC) := akashi_temac.o zynq_interface.o switch_lib_linux.o switch_lib_shared.o switch_lib_uboot.o switch_lib_utils.o
akashi-temac-$(CONFIG_AKASHI_PS_MAC) := zynq_ps_mac.o
