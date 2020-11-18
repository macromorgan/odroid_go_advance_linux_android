MODNAME = esp8089

EXTRA_CFLAGS += -DDEBUG -DSIP_DEBUG -DFAST_TX_STATUS \
    -DKERNEL_IV_WAR -DRX_SENDUP_SYNC -DDEBUG_FS \
    -DSIF_DSR_WAR -DHAS_INIT_DATA -DHAS_FW

EXTRA_CFLAGS += -DESP_USE_SDIO -DANDROID -DP2P_CONCURRENT

ifdef ANDROID
EXTRA_CFLAGS += -DANDROID
endif

ifdef P2P_CONCURRENT
EXTRA_CFLAGS += -DP2P_CONCURRENT
endif

ifdef TEST_MODE
EXTRA_CFLAGS += -DTEST_MODE
endif

OBJS = esp_debug.o sdio_sif_esp.o spi_sif_esp.o esp_io.o \
    esp_file.o esp_main.o esp_sip.o esp_ext.o esp_ctrl.o \
    esp_mac80211.o esp_debug.o esp_utils.o esp_pm.o testmode.o

MODULE := $(MODNAME).ko
obj-$(CONFIG_ESP8089) := $(MODNAME).o

$(MODNAME)-objs := $(OBJS)

