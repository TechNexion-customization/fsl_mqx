This folder contains the mass build project for all tool chains supported in MQX RTOS. 

Before you start read following documentation:
  Build, Run on target and debugging
  - /doc/MQX_Getting_Started.pdf file provides general information on MQX build process
  - /doc/<tool>/MQX_<TOOL>_Getting_started.pdf file provides step-by-step guide for your favorite tool chain.
  Board related information  and jumper settings
  - /doc/MQX_Release_Notes.pdf chapter "Installation Instructions"

This folder contains makefiles in following structure: 

build
+---common
|   +---make        ... shared Makefiles with global settings, variables and paths
+---<board1>        ... board-specific folder
|   +---make        ... folder contains mass-build Makefile for all libraries
|       +---tools   ... tool-specific global settings, variables and paths
|   +---iar         ... folder contains workspace with mass-build configuration for MQX libraries IAR EWARM
|   +---ds5         ... folder contains workspace with mass-build configuration for MQX libraries ARM DS5 IDE
+---<board2>  
    +---make
        ...

*** MAKE usage ***
  - See step-by-step guide in /doc/tools/gnu/MQX_GNU_Getting_Started.pdf
  - Navigate to the <mqx_install_dir>/build/<board>/make directory 
  Note: 
  - Please use the mingw32-make version 3.8.2 or higher. Download the latest version from http://sourceforge.net/projects/mingw/. 
  - Prior the build you should specify the path (TOOLCHAIN_ROOTDIR variable) to your build tools in the <mqx_install_dir>/build/common/make/global.mak. To get the default path to the toolchain 
    navigate to the toolchain installation directory and execute "command" on the Windows Command Prompt.
