#!/usr/bin/env bash

# expect forward slash paths
ROOTDIR="${1}"
OUTPUTDIR="${2}"
TOOL="${3}"


# copy common files
mkdir -p "${OUTPUTDIR}"
mkdir -p "${OUTPUTDIR}/"
mkdir -p "${OUTPUTDIR}/.."
cp -f "${ROOTDIR}/mqx/source/io/sensor/mag3110/mag3110.h" "${OUTPUTDIR}/mag3110.h"
cp -f "${ROOTDIR}/mqx/source/io/lwgpio/lwgpio.h" "${OUTPUTDIR}/lwgpio.h"
cp -f "${ROOTDIR}/mqx/source/io/cm/clk_api.h" "${OUTPUTDIR}/clk_api.h"
cp -f "${ROOTDIR}/mqx/source/io/sensor/mma8451q/mma8451q_basic.h" "${OUTPUTDIR}/mma8451q_basic.h"
cp -f "${ROOTDIR}/mqx/source/io/can/flexcan/fsl_flexcan_hal.h" "${OUTPUTDIR}/fsl_flexcan_hal.h"
cp -f "${ROOTDIR}/mqx/source/io/sensor/mag3110/mag3110_basic.h" "${OUTPUTDIR}/mag3110_basic.h"
cp -f "${ROOTDIR}/mqx/source/io/sensor/mag3110/mag3110_reg.h" "${OUTPUTDIR}/mag3110_reg.h"
cp -f "${ROOTDIR}/mqx/source/io/hwtimer/hwtimer_systick.h" "${OUTPUTDIR}/hwtimer_systick.h"
cp -f "${ROOTDIR}/mqx/source/io/lpm/imx6sx/lpm_imx6sx.h" "${OUTPUTDIR}/lpm_imx6sx.h"
cp -f "${ROOTDIR}/mqx/source/io/cm/imx6sx/clk_name.h" "${OUTPUTDIR}/clk_name.h"
cp -f "${ROOTDIR}/mqx/source/io/lwgpio/lwgpio_igpio.h" "${OUTPUTDIR}/lwgpio_igpio.h"
cp -f "${ROOTDIR}/mqx/source/io/lwadc/lwadc.h" "${OUTPUTDIR}/lwadc.h"
cp -f "${ROOTDIR}/mqx/source/include/mqx.h" "${OUTPUTDIR}/mqx.h"
cp -f "${ROOTDIR}/mqx/source/io/sensor/mma8451q/mma8451q_reg.h" "${OUTPUTDIR}/mma8451q_reg.h"
cp -f "${ROOTDIR}/mqx/source/bsp/imx6sx_sdb_m4/init_lpm.h" "${OUTPUTDIR}/init_lpm.h"
cp -f "${ROOTDIR}/mqx/source/io/cm/cm_imx6sx.h" "${OUTPUTDIR}/cm_imx6sx.h"
cp -f "${ROOTDIR}/mqx/source/io/core_mutex/core_mutex.h" "${OUTPUTDIR}/core_mutex.h"
cp -f "${ROOTDIR}/mqx/source/io/sensor/mma8451q/mma8451q_generic.h" "${OUTPUTDIR}/mma8451q_generic.h"
cp -f "${ROOTDIR}/mqx/source/io/lpm_mcore/lpm_mcore.h" "${OUTPUTDIR}/lpm_mcore.h"
cp -f "${ROOTDIR}/mqx/source/io/spi/spi.h" "${OUTPUTDIR}/spi.h"
cp -f "${ROOTDIR}/mqx/source/io/serial/serial.h" "${OUTPUTDIR}/serial.h"
cp -f "${ROOTDIR}/mqx/source/io/cm/cm.h" "${OUTPUTDIR}/cm.h"
cp -f "${ROOTDIR}/mqx/source/io/core_mutex/sema4.h" "${OUTPUTDIR}/sema4.h"
cp -f "${ROOTDIR}/mqx/source/io/serial/serl_imx_uart.h" "${OUTPUTDIR}/serl_imx_uart.h"
cp -f "${ROOTDIR}/mqx/source/bsp/imx6sx_sdb_m4/imx6sx_sdb_m4.h" "${OUTPUTDIR}/imx6sx_sdb_m4.h"
cp -f "${ROOTDIR}/mqx/source/io/can/flexcan/fsl_flexcan_driver.h" "${OUTPUTDIR}/fsl_flexcan_driver.h"
cp -f "${ROOTDIR}/mqx/source/io/lpm/imx6sx/imx6sx_wkpu.h" "${OUTPUTDIR}/imx6sx_wkpu.h"
cp -f "${ROOTDIR}/mqx/source/io/sensor/mag3110/mag3110_fun.h" "${OUTPUTDIR}/mag3110_fun.h"
cp -f "${ROOTDIR}/config/common/small_ram_config.h" "${OUTPUTDIR}/../small_ram_config.h"
cp -f "${ROOTDIR}/mqx/source/io/lwadc/lwadc_imxadc.h" "${OUTPUTDIR}/lwadc_imxadc.h"
cp -f "${ROOTDIR}/mqx/source/io/sensor/mma8451q/mma8451q_tran.h" "${OUTPUTDIR}/mma8451q_tran.h"
cp -f "${ROOTDIR}/mqx/source/bsp/imx6sx_sdb_m4/bsp_rev.h" "${OUTPUTDIR}/bsp_rev.h"
cp -f "${ROOTDIR}/mqx/source/io/sensor/mma8451q/mma8451q_lapo.h" "${OUTPUTDIR}/mma8451q_lapo.h"
cp -f "${ROOTDIR}/mqx/source/io/pipe/io_pipe.h" "${OUTPUTDIR}/io_pipe.h"
cp -f "${ROOTDIR}/config/common/maximum_config.h" "${OUTPUTDIR}/../maximum_config.h"
cp -f "${ROOTDIR}/config/imx6sx_sdb_m4/user_config.h" "${OUTPUTDIR}/../user_config.h"
cp -f "${ROOTDIR}/mqx/source/io/i2c/i2c_imx.h" "${OUTPUTDIR}/i2c_imx.h"
cp -f "${ROOTDIR}/mqx/source/io/sensor/mma8451q/mma8451q.h" "${OUTPUTDIR}/mma8451q.h"
cp -f "${ROOTDIR}/mqx/source/bsp/imx6sx_sdb_m4/bsp.h" "${OUTPUTDIR}/bsp.h"
cp -f "${ROOTDIR}/mqx/source/bsp/imx6sx_sdb_m4/bsp_cm.h" "${OUTPUTDIR}/bsp_cm.h"
cp -f "${ROOTDIR}/mqx/source/io/mu/mu.h" "${OUTPUTDIR}/mu.h"
cp -f "${ROOTDIR}/mqx/source/io/hwtimer/hwtimer_epit.h" "${OUTPUTDIR}/hwtimer_epit.h"
cp -f "${ROOTDIR}/mqx/source/io/lpm/lpm.h" "${OUTPUTDIR}/lpm.h"
cp -f "${ROOTDIR}/mqx/source/io/sensor/mma8451q/mma8451q_pulse.h" "${OUTPUTDIR}/mma8451q_pulse.h"
cp -f "${ROOTDIR}/mqx/source/io/sensor/mma8451q/mma8451q_ff_mt.h" "${OUTPUTDIR}/mma8451q_ff_mt.h"
cp -f "${ROOTDIR}/mqx/source/io/core_mutex/core_mutex_sema4.h" "${OUTPUTDIR}/core_mutex_sema4.h"
cp -f "${ROOTDIR}/mqx/source/io/spi_slave/spi_slave_ecspi.h" "${OUTPUTDIR}/spi_slave_ecspi.h"
cp -f "${ROOTDIR}/mqx/source/io/i2c/i2c.h" "${OUTPUTDIR}/i2c.h"
cp -f "${ROOTDIR}/mqx/source/io/spi/spi_ecspi.h" "${OUTPUTDIR}/spi_ecspi.h"
cp -f "${ROOTDIR}/config/common/verif_enabled_config.h" "${OUTPUTDIR}/../verif_enabled_config.h"
cp -f "${ROOTDIR}/mqx/source/io/can/flexcan/fsl_flexcan_int.h" "${OUTPUTDIR}/fsl_flexcan_int.h"
cp -f "${ROOTDIR}/mqx/source/io/hwtimer/hwtimer.h" "${OUTPUTDIR}/hwtimer.h"
cp -f "${ROOTDIR}/config/common/smallest_config.h" "${OUTPUTDIR}/../smallest_config.h"


# iar files
if [ "${TOOL}" = "iar" ]; then
mkdir -p "${OUTPUTDIR}/"
cp -f "${ROOTDIR}/mqx/source/bsp/imx6sx_sdb_m4/iar/ram.icf" "${OUTPUTDIR}/ram.icf"
cp -f "${ROOTDIR}/mqx/source/bsp/imx6sx_sdb_m4/iar/extflash.icf" "${OUTPUTDIR}/extflash.icf"
mkdir -p "${OUTPUTDIR}/Generated_Code"
mkdir -p "${OUTPUTDIR}/Sources"
:
fi

# ds5 files
if [ "${TOOL}" = "ds5" ]; then
mkdir -p "${OUTPUTDIR}/"
cp -f "${ROOTDIR}/mqx/source/bsp/imx6sx_sdb_m4/ds5/ram.scf" "${OUTPUTDIR}/ram.scf"
cp -f "${ROOTDIR}/mqx/source/bsp/imx6sx_sdb_m4/ds5/extflash.scf" "${OUTPUTDIR}/extflash.scf"
mkdir -p "${OUTPUTDIR}/Generated_Code"
mkdir -p "${OUTPUTDIR}/Sources"
:
fi

# gcc_arm files
if [ "${TOOL}" = "gcc_arm" ]; then
mkdir -p "${OUTPUTDIR}/"
cp -f "${ROOTDIR}/mqx/source/bsp/imx6sx_sdb_m4/gcc_arm/ram.ld" "${OUTPUTDIR}/ram.ld"
cp -f "${ROOTDIR}/mqx/source/bsp/imx6sx_sdb_m4/gcc_arm/extflash.ld" "${OUTPUTDIR}/extflash.ld"
mkdir -p "${OUTPUTDIR}/Generated_Code"
mkdir -p "${OUTPUTDIR}/Sources"
:
fi


