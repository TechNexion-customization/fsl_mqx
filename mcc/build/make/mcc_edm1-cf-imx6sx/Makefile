#----------------------------------------------------------
# setup variables
#----------------------------------------------------------
MQX_ROOTDIR             = $(abspath ../../../..)
TYPE                    = library
NAME                    = mcc
BOARD                   = edm1-cf-imx6sx
LIBRARY_ROOTDIR         = $(MQX_ROOTDIR)/lib/$(BOARD).$(TOOL)
LIBRARY_DIR             = $(LIBRARY_ROOTDIR)/$(CONFIG)/mcc
LIBRARY_FILE            = $(LIBRARY_DIR)/mcc.$(LIBRARY_EXT)
POSTBUILD_CMD           = $(if $(findstring $(HOSTENV),WINDOWS),$(MQX_ROOTDIR)/mcc/build/bat/mcc_edm1-cf-imx6sx.bat $(MQX_ROOTDIR) $(LIBRARY_DIR) $(TOOL),$(MQX_ROOTDIR)/mcc/build/bat/mcc_edm1-cf-imx6sx.sh $(MQX_ROOTDIR) $(LIBRARY_DIR) $(TOOL))


#-----------------------------------------------------------
# load tool settings
#-----------------------------------------------------------
-include tools/$(TOOL).mak


#-----------------------------------------------------------
# search paths
#-----------------------------------------------------------
ifeq ($(CONFIG),debug)
INCLUDE += $(MQX_ROOTDIR)/mcc/source/include
INCLUDE += $(MQX_ROOTDIR)/mqx/source/bsp/edm1-cf-imx6sx
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/cm
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/cm/imx6sx
INCLUDE += $(MQX_ROOTDIR)/config/edm1-cf-imx6sx
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/core_mutex
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/can/flexcan
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/hwtimer
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/i2c
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/i2c/polled
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/i2c/int
INCLUDE += $(MQX_ROOTDIR)/config/common
INCLUDE += $(MQX_ROOTDIR)/mqx/source/include
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/pipe
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/lpm/imx6sx
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/lpm
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/lpm_mcore
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/lwadc
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/lwgpio
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/sensor/mag3110
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/sensor/mma8451q
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/mu
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/serial
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/spi
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/spi_slave
INCLUDE += $(MQX_ROOTDIR)/mqx/source/psp/cortex_m/cpu
INCLUDE += $(MQX_ROOTDIR)/mqx/source/psp/cortex_m
INCLUDE += $(MQX_ROOTDIR)/mqx/source/psp/cortex_m/core/M4
INCLUDE += $(MQX_ROOTDIR)/mqx/source/tad
endif
ifeq ($(CONFIG),release)
INCLUDE += $(MQX_ROOTDIR)/mcc/source/include
INCLUDE += $(MQX_ROOTDIR)/mqx/source/bsp/edm1-cf-imx6sx
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/cm
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/cm/imx6sx
INCLUDE += $(MQX_ROOTDIR)/config/edm1-cf-imx6sx
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/core_mutex
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/can/flexcan
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/hwtimer
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/i2c
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/i2c/polled
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/i2c/int
INCLUDE += $(MQX_ROOTDIR)/config/common
INCLUDE += $(MQX_ROOTDIR)/mqx/source/include
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/pipe
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/lpm/imx6sx
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/lpm
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/lpm_mcore
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/lwadc
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/lwgpio
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/sensor/mag3110
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/sensor/mma8451q
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/mu
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/serial
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/spi
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/spi_slave
INCLUDE += $(MQX_ROOTDIR)/mqx/source/psp/cortex_m/cpu
INCLUDE += $(MQX_ROOTDIR)/mqx/source/psp/cortex_m
INCLUDE += $(MQX_ROOTDIR)/mqx/source/psp/cortex_m/core/M4
INCLUDE += $(MQX_ROOTDIR)/mqx/source/tad
endif



#-----------------------------------------------------------
# 'source' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mcc/source/mcc_imx6sx.c
SOURCES += $(MQX_ROOTDIR)/mcc/source/mcc_api.c
SOURCES += $(MQX_ROOTDIR)/mcc/source/mcc_common.c
SOURCES += $(MQX_ROOTDIR)/mcc/source/mqx/mcc_mqx.c


#----------------------------------------------------------
# load global settings
#----------------------------------------------------------
include $(MQX_ROOTDIR)/build/common/make/global.mak
-include $(MQX_ROOTDIR)/build/$(BOARD)/make/tools/$(TOOL).mak
include $(MQX_ROOTDIR)/build/common/make/verify.mak
include $(MQX_ROOTDIR)/build/common/make/lib-process.mak

