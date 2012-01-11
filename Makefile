cw1200_core-y := \
		fwio.o \
		txrx.o \
		main.o \
		queue.o \
		hwio.o \
		bh.o \
		wsm.o \
		sta.o \
		ap.o \
		scan.o
cw1200_core-$(CONFIG_CW1200_DEBUGFS)	+= debug.o
cw1200_core-$(CONFIG_CW1200_PM)		+= pm.o

cw1200_wlan_sdio-y := cw1200_sdio.o
cw1200_wlan_spi-y := cw1200_spi.o

obj-$(CONFIG_CW1200) += cw1200_core.o
obj-$(CONFIG_CW1200_WLAN_SDIO) += cw1200_wlan_sdio.o
obj-$(CONFIG_CW1200_WLAN_SPI) += cw1200_wlan_spi.o


