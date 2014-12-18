#
# Makefile for PNET.
#

obj-$(CONFIG_PNET)		+= pnet_core.o
obj-$(CONFIG_PNET_NS_SPI)	+= pnet_ns_spi.o
obj-$(CONFIG_PNET_LOOPBACK)	+= pnet_loopback.o
obj-$(CONFIG_PNET_LOCAL)	+= pnet_local.o
obj-$(CONFIG_PNET_FIFO)		+= pnet_fifo.o
obj-$(CONFIG_PNET_SERIAL)	+= pnet_serial.o
obj-$(CONFIG_PNET_TS)		+= pnet_ts.o
obj-$(CONFIG_PNET_RDEV)		+= pnet_rdev.o
obj-$(CONFIG_PNET_SYSTEM)	+= pnet_system.o
obj-$(CONFIG_PNET_CAN)		+= pnet_can.o
obj-$(CONFIG_PNET_GE_CAN)	+= pnet_ge_can.o

ifeq ($(CONFIG_PNET_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
