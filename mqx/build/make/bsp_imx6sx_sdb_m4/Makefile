#----------------------------------------------------------
# setup variables
#----------------------------------------------------------
MQX_ROOTDIR             = $(abspath ../../../..)
TYPE                    = library
NAME                    = bsp
BOARD                   = imx6sx_sdb_m4
LIBRARY_ROOTDIR         = $(MQX_ROOTDIR)/lib/$(BOARD).$(TOOL)
LIBRARY_DIR             = $(LIBRARY_ROOTDIR)/$(CONFIG)/bsp
LIBRARY_FILE            = $(LIBRARY_DIR)/bsp.$(LIBRARY_EXT)
POSTBUILD_CMD           = $(if $(findstring $(HOSTENV),WINDOWS),$(MQX_ROOTDIR)/mqx/build/bat/bsp_imx6sx_sdb_m4.bat $(MQX_ROOTDIR) $(LIBRARY_DIR) $(TOOL),$(MQX_ROOTDIR)/mqx/build/bat/bsp_imx6sx_sdb_m4.sh $(MQX_ROOTDIR) $(LIBRARY_DIR) $(TOOL))


#-----------------------------------------------------------
# load tool settings
#-----------------------------------------------------------
-include tools/$(TOOL).mak


#-----------------------------------------------------------
# search paths
#-----------------------------------------------------------
ifeq ($(CONFIG),debug)
INCLUDE += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/cm
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/cm/imx6sx
INCLUDE += $(MQX_ROOTDIR)/config/imx6sx_sdb_m4
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
INCLUDE += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/cm
INCLUDE += $(MQX_ROOTDIR)/mqx/source/io/cm/imx6sx
INCLUDE += $(MQX_ROOTDIR)/config/imx6sx_sdb_m4
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
# 'cm' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/bsp_clk_shared_mgmt.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/cm/imx6sx/clk_nodes.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/cm/clk_api.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/cm/cm.c

#-----------------------------------------------------------
# 'core-mutex' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/core_mutex/sema4_imx6sx.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/core_mutex/core_mutex_sema4.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/core_mutex/core_mutex.c

#-----------------------------------------------------------
# 'flexcan' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/can/flexcan/fsl_flexcan_imx6sx.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/can/flexcan/fsl_flexcan_hal.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/can/flexcan/fsl_flexcan_driver.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/can/flexcan/fsl_flexcan_int.c

#-----------------------------------------------------------
# 'hwtimer' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/hwtimer/hwtimer_imx6sx_m4.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/hwtimer/hwtimer_systick.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/hwtimer/hwtimer_epit.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/hwtimer/hwtimer.c

#-----------------------------------------------------------
# 'i2c' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/i2c/i2c_imx6sx_m4.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/i2c/polled/i2c_pol_imx.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/i2c/int/i2c_int_imx.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/i2c/polled/i2c_pol.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/i2c/int/i2c_int.c

#-----------------------------------------------------------
# 'init' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/bsp_cm.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/vectors.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/init_gpio.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/init_hw.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/init_i2c.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/init_lpm.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/init_sci.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/init_lwadc.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/init_spi.c

#-----------------------------------------------------------
# 'init-common' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/init_bsp.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/mqx_init.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/mqx_main.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/bsp/imx6sx_sdb_m4/init_core_mutex.c

#-----------------------------------------------------------
# 'iopipe' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/pipe/io_pipe.c

#-----------------------------------------------------------
# 'lpm' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/lpm/imx6sx/lpm_imx6sx.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/lpm/imx6sx/imx6sx_wkpu.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/lpm/lpm.c

#-----------------------------------------------------------
# 'lpm_mcore' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/lpm_mcore/lpm_mcore_imx6sx.c

#-----------------------------------------------------------
# 'lwadc' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/lwadc/lwadc_imx6sx.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/lwadc/lwadc_imxadc.c

#-----------------------------------------------------------
# 'lwgpio' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/lwgpio/lwgpio_igpio_imx6sx.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/lwgpio/lwgpio_igpio.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/lwgpio/lwgpio.c

#-----------------------------------------------------------
# 'mag3110' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/sensor/mag3110/mag3110_basic.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/sensor/mag3110/mag3110_fun.c

#-----------------------------------------------------------
# 'mma8451q' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/sensor/mma8451q/mma8451q_basic.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/sensor/mma8451q/mma8451q_generic.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/sensor/mma8451q/mma8451q_lapo.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/sensor/mma8451q/mma8451q_ff_mt.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/sensor/mma8451q/mma8451q_tran.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/sensor/mma8451q/mma8451q_pulse.c

#-----------------------------------------------------------
# 'mu' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/mu/mu_imx6sx_m4.c

#-----------------------------------------------------------
# 'serial' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/serial/serl_imx6sx.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/serial/polled/serl_pol_imx_uart.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/serial/int/serl_int_imx_uart.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/serial/io_sinit.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/serial/polled/serl_pol.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/serial/int/serl_int.c

#-----------------------------------------------------------
# 'spi' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/spi/spi_imx6sx_m4.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/spi/spi_ecspi.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/spi/spi.c

#-----------------------------------------------------------
# 'spi_slave' sources
#-----------------------------------------------------------
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/spi_slave/spi_slave_imx6sx_m4.c
SOURCES += $(MQX_ROOTDIR)/mqx/source/io/spi_slave/spi_slave_ecspi.c


#----------------------------------------------------------
# load global settings
#----------------------------------------------------------
include $(MQX_ROOTDIR)/build/common/make/global.mak
-include $(MQX_ROOTDIR)/build/$(BOARD)/make/tools/$(TOOL).mak
include $(MQX_ROOTDIR)/build/common/make/verify.mak
include $(MQX_ROOTDIR)/build/common/make/lib-process.mak

