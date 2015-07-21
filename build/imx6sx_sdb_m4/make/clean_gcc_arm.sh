#!/usr/bin/env bash

status=0

cd ../../../mqx/build/make/bsp_imx6sx_sdb_m4 && ./clean_gcc_arm.sh nopause && cd -
if [ "$?" != "0" ]; then
    status=-1
fi
cd ../../../mcc/build/make/mcc_imx6sx_sdb_m4 && ./clean_gcc_arm.sh nopause && cd -
if [ "$?" != "0" ]; then
    status=-1
fi
cd ../../../mqx/build/make/psp_imx6sx_sdb_m4 && ./clean_gcc_arm.sh nopause && cd -
if [ "$?" != "0" ]; then
    status=-1
fi

if [ "${1}" != "nopause" ]; then
read -p "Press any key to continue... " -n1 -s
fi

exit $status

