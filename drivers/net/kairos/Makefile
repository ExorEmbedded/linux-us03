# Comment/uncomment the following line to disable/enable debugging
#DEBUG = y

# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
  DEBFLAGS = -O -g # "-O" is needed to expand inlines
else
  DEBFLAGS = -O2
endif

EXTRA_CFLAGS += $(DEBFLAGS)

ifneq ($(KERNELRELEASE),)
# call from kernel build system

ccflags-y += -Wall
obj-m += kairos.o
kairos-y := kairos_main.o \
	kairos_netdev.o kairos_netdevif.o \
	kairos_ioctl.o kairos_hw.o kairos_ethtool.o \
	kairos_adapter.o \
	kairos_clock_main.o deipce_clock_nco.o 

#deipce_sfp.o deipce_preempt.o
#	kairos_fpts_main.o \
#	kairos_fsc_main.o deipce_fsc_hw.o deipce_ibc_main.o \
#	kairos_shaper.o deipce_time.o deipce_mstp.o

kairos-$(CONFIG_NET_SWITCHDEV) += kairos_switchdev.o
kairos-$(CONFIG_PTP_1588_CLOCK) += kairos_clock_ptp.o
#deipce-$(CONFIG_SYSFS) += deipce_fsc_sysfs.o deipce_fqtss_sysfs.o \
#	deipce_port_sysfs.o deipce_edgex_sysfs.o deipce_phys_sysfs.o \
#	deipce_preempt_sysfs.o
#ifdef CONFIG_NET_SWITCHDEV
#deipce-$(CONFIG_SYSFS) += deipce_qbridge_sysfs.o \
#	deipce_bridge_sysfs.o deipce_mstp_sysfs.o
#endif

else

KDIR ?= /lib/modules/$(shell uname -r)/build

modules modules_install clean:
	$(MAKE) -C $(KDIR) M=$$PWD $@

endif

