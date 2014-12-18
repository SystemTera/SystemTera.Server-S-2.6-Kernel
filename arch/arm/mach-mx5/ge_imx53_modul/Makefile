#
# Makefile for the linux kernel.
#

# Object file lists.

# module code
obj-y   := ge_imx53_modul.o ge_imx53_modul_pmic_ltc3589.o ge_imx53_gpio_keys.o

# board specific code
obj-$(CONFIG_MACH_GE_IMX53_TESTBOARD) += ge_imx53_testboard.o
obj-$(CONFIG_MACH_GE_IMX53_DEVBOARD) += ge_imx53_devboard.o
obj-$(CONFIG_MACH_GE_IMX53_BEP2) += ge_imx53_bep2.o
obj-$(CONFIG_MACH_GE_IMX53_MMI) += ge_imx53_mmi.o
obj-$(CONFIG_MACH_GE_IMX53_BT) += ge_imx53_bt.o
obj-$(CONFIG_MACH_GE_IMX53_TERA_COM) += ge_imx53_tera_com.o
obj-$(CONFIG_MACH_GE_IMX53_SIGMA2) += ge_imx53_sigma2.o

