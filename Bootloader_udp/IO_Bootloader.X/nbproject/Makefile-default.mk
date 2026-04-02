#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/IO_Bootloader.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/IO_Bootloader.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/config/default/bootloader/datastream/datastream_udp.c ../src/config/default/bootloader/bootloader_nvm_interface.c ../src/config/default/bootloader/bootloader_udp.c ../src/config/default/bootloader/bootloader_common.c ../src/config/default/driver/ethphy/src/dynamic/drv_extphy_ksz8863.c ../src/config/default/driver/ethphy/src/dynamic/drv_ethphy.c ../src/config/default/driver/gmac/src/dynamic/drv_gmac_lib_pic32CZ.c ../src/config/default/driver/gmac/src/dynamic/drv_gmac.c ../src/config/default/driver/miim/src/dynamic/drv_miim.c ../src/config/default/driver/miim/src/dynamic/drv_miim_pic32cz.c ../src/config/default/library/tcpip/src/helpers.c ../src/config/default/library/tcpip/src/tcpip_heap_internal.c ../src/config/default/library/tcpip/src/tcpip_heap_alloc.c ../src/config/default/library/tcpip/src/tcpip_manager.c ../src/config/default/library/tcpip/src/ipv4.c ../src/config/default/library/tcpip/src/dns.c ../src/config/default/library/tcpip/src/dhcp.c ../src/config/default/library/tcpip/src/tcpip_helpers.c ../src/config/default/library/tcpip/src/oahash.c ../src/config/default/library/tcpip/src/tcpip_notify.c ../src/config/default/library/tcpip/src/arp.c ../src/config/default/library/tcpip/src/udp.c ../src/config/default/library/tcpip/src/tcpip_packet.c ../src/config/default/library/tcpip/src/hash_fnv.c ../src/config/default/peripheral/clock/plib_clock.c ../src/config/default/peripheral/evsys/plib_evsys.c ../src/config/default/peripheral/fcw/plib_fcw.c ../src/config/default/peripheral/mpu/plib_mpu.c ../src/config/default/peripheral/nvic/plib_nvic.c ../src/config/default/peripheral/port/plib_port.c ../src/config/default/peripheral/rtc/plib_rtc_timer.c ../src/config/default/peripheral/sercom/usart/plib_sercom0_usart.c ../src/config/default/peripheral/wdt/plib_wdt.c ../src/config/default/stdio/xc32_monitor.c ../src/config/default/system/cache/sys_cache.c ../src/config/default/system/console/src/sys_console.c ../src/config/default/system/console/src/sys_console_uart.c ../src/config/default/system/debug/src/sys_debug.c ../src/config/default/system/int/src/sys_int.c ../src/config/default/system/time/src/sys_time.c ../src/config/default/system/sys_time_h2_adapter.c ../src/config/default/initialization.c ../src/config/default/startup_xc32.c ../src/config/default/tasks.c ../src/config/default/interrupts.c ../src/config/default/exceptions.c ../src/config/default/libc_syscalls.c ../src/main.c ../src/app_pic32cz_ca.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1277623251/datastream_udp.o ${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o ${OBJECTDIR}/_ext/1748409734/bootloader_udp.o ${OBJECTDIR}/_ext/1748409734/bootloader_common.o ${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o ${OBJECTDIR}/_ext/444070925/drv_ethphy.o ${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o ${OBJECTDIR}/_ext/1546807373/drv_gmac.o ${OBJECTDIR}/_ext/438849557/drv_miim.o ${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o ${OBJECTDIR}/_ext/1033058136/helpers.o ${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o ${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o ${OBJECTDIR}/_ext/1033058136/tcpip_manager.o ${OBJECTDIR}/_ext/1033058136/ipv4.o ${OBJECTDIR}/_ext/1033058136/dns.o ${OBJECTDIR}/_ext/1033058136/dhcp.o ${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o ${OBJECTDIR}/_ext/1033058136/oahash.o ${OBJECTDIR}/_ext/1033058136/tcpip_notify.o ${OBJECTDIR}/_ext/1033058136/arp.o ${OBJECTDIR}/_ext/1033058136/udp.o ${OBJECTDIR}/_ext/1033058136/tcpip_packet.o ${OBJECTDIR}/_ext/1033058136/hash_fnv.o ${OBJECTDIR}/_ext/1984496892/plib_clock.o ${OBJECTDIR}/_ext/1986646378/plib_evsys.o ${OBJECTDIR}/_ext/60168136/plib_fcw.o ${OBJECTDIR}/_ext/60175264/plib_mpu.o ${OBJECTDIR}/_ext/1865468468/plib_nvic.o ${OBJECTDIR}/_ext/1865521619/plib_port.o ${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o ${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o ${OBJECTDIR}/_ext/60184501/plib_wdt.o ${OBJECTDIR}/_ext/163028504/xc32_monitor.o ${OBJECTDIR}/_ext/1014039709/sys_cache.o ${OBJECTDIR}/_ext/1832805299/sys_console.o ${OBJECTDIR}/_ext/1832805299/sys_console_uart.o ${OBJECTDIR}/_ext/944882569/sys_debug.o ${OBJECTDIR}/_ext/1881668453/sys_int.o ${OBJECTDIR}/_ext/101884895/sys_time.o ${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o ${OBJECTDIR}/_ext/1171490990/initialization.o ${OBJECTDIR}/_ext/1171490990/startup_xc32.o ${OBJECTDIR}/_ext/1171490990/tasks.o ${OBJECTDIR}/_ext/1171490990/interrupts.o ${OBJECTDIR}/_ext/1171490990/exceptions.o ${OBJECTDIR}/_ext/1171490990/libc_syscalls.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1277623251/datastream_udp.o.d ${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o.d ${OBJECTDIR}/_ext/1748409734/bootloader_udp.o.d ${OBJECTDIR}/_ext/1748409734/bootloader_common.o.d ${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o.d ${OBJECTDIR}/_ext/444070925/drv_ethphy.o.d ${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o.d ${OBJECTDIR}/_ext/1546807373/drv_gmac.o.d ${OBJECTDIR}/_ext/438849557/drv_miim.o.d ${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o.d ${OBJECTDIR}/_ext/1033058136/helpers.o.d ${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o.d ${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o.d ${OBJECTDIR}/_ext/1033058136/tcpip_manager.o.d ${OBJECTDIR}/_ext/1033058136/ipv4.o.d ${OBJECTDIR}/_ext/1033058136/dns.o.d ${OBJECTDIR}/_ext/1033058136/dhcp.o.d ${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o.d ${OBJECTDIR}/_ext/1033058136/oahash.o.d ${OBJECTDIR}/_ext/1033058136/tcpip_notify.o.d ${OBJECTDIR}/_ext/1033058136/arp.o.d ${OBJECTDIR}/_ext/1033058136/udp.o.d ${OBJECTDIR}/_ext/1033058136/tcpip_packet.o.d ${OBJECTDIR}/_ext/1033058136/hash_fnv.o.d ${OBJECTDIR}/_ext/1984496892/plib_clock.o.d ${OBJECTDIR}/_ext/1986646378/plib_evsys.o.d ${OBJECTDIR}/_ext/60168136/plib_fcw.o.d ${OBJECTDIR}/_ext/60175264/plib_mpu.o.d ${OBJECTDIR}/_ext/1865468468/plib_nvic.o.d ${OBJECTDIR}/_ext/1865521619/plib_port.o.d ${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o.d ${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o.d ${OBJECTDIR}/_ext/60184501/plib_wdt.o.d ${OBJECTDIR}/_ext/163028504/xc32_monitor.o.d ${OBJECTDIR}/_ext/1014039709/sys_cache.o.d ${OBJECTDIR}/_ext/1832805299/sys_console.o.d ${OBJECTDIR}/_ext/1832805299/sys_console_uart.o.d ${OBJECTDIR}/_ext/944882569/sys_debug.o.d ${OBJECTDIR}/_ext/1881668453/sys_int.o.d ${OBJECTDIR}/_ext/101884895/sys_time.o.d ${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o.d ${OBJECTDIR}/_ext/1171490990/initialization.o.d ${OBJECTDIR}/_ext/1171490990/startup_xc32.o.d ${OBJECTDIR}/_ext/1171490990/tasks.o.d ${OBJECTDIR}/_ext/1171490990/interrupts.o.d ${OBJECTDIR}/_ext/1171490990/exceptions.o.d ${OBJECTDIR}/_ext/1171490990/libc_syscalls.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1277623251/datastream_udp.o ${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o ${OBJECTDIR}/_ext/1748409734/bootloader_udp.o ${OBJECTDIR}/_ext/1748409734/bootloader_common.o ${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o ${OBJECTDIR}/_ext/444070925/drv_ethphy.o ${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o ${OBJECTDIR}/_ext/1546807373/drv_gmac.o ${OBJECTDIR}/_ext/438849557/drv_miim.o ${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o ${OBJECTDIR}/_ext/1033058136/helpers.o ${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o ${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o ${OBJECTDIR}/_ext/1033058136/tcpip_manager.o ${OBJECTDIR}/_ext/1033058136/ipv4.o ${OBJECTDIR}/_ext/1033058136/dns.o ${OBJECTDIR}/_ext/1033058136/dhcp.o ${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o ${OBJECTDIR}/_ext/1033058136/oahash.o ${OBJECTDIR}/_ext/1033058136/tcpip_notify.o ${OBJECTDIR}/_ext/1033058136/arp.o ${OBJECTDIR}/_ext/1033058136/udp.o ${OBJECTDIR}/_ext/1033058136/tcpip_packet.o ${OBJECTDIR}/_ext/1033058136/hash_fnv.o ${OBJECTDIR}/_ext/1984496892/plib_clock.o ${OBJECTDIR}/_ext/1986646378/plib_evsys.o ${OBJECTDIR}/_ext/60168136/plib_fcw.o ${OBJECTDIR}/_ext/60175264/plib_mpu.o ${OBJECTDIR}/_ext/1865468468/plib_nvic.o ${OBJECTDIR}/_ext/1865521619/plib_port.o ${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o ${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o ${OBJECTDIR}/_ext/60184501/plib_wdt.o ${OBJECTDIR}/_ext/163028504/xc32_monitor.o ${OBJECTDIR}/_ext/1014039709/sys_cache.o ${OBJECTDIR}/_ext/1832805299/sys_console.o ${OBJECTDIR}/_ext/1832805299/sys_console_uart.o ${OBJECTDIR}/_ext/944882569/sys_debug.o ${OBJECTDIR}/_ext/1881668453/sys_int.o ${OBJECTDIR}/_ext/101884895/sys_time.o ${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o ${OBJECTDIR}/_ext/1171490990/initialization.o ${OBJECTDIR}/_ext/1171490990/startup_xc32.o ${OBJECTDIR}/_ext/1171490990/tasks.o ${OBJECTDIR}/_ext/1171490990/interrupts.o ${OBJECTDIR}/_ext/1171490990/exceptions.o ${OBJECTDIR}/_ext/1171490990/libc_syscalls.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o

# Source Files
SOURCEFILES=../src/config/default/bootloader/datastream/datastream_udp.c ../src/config/default/bootloader/bootloader_nvm_interface.c ../src/config/default/bootloader/bootloader_udp.c ../src/config/default/bootloader/bootloader_common.c ../src/config/default/driver/ethphy/src/dynamic/drv_extphy_ksz8863.c ../src/config/default/driver/ethphy/src/dynamic/drv_ethphy.c ../src/config/default/driver/gmac/src/dynamic/drv_gmac_lib_pic32CZ.c ../src/config/default/driver/gmac/src/dynamic/drv_gmac.c ../src/config/default/driver/miim/src/dynamic/drv_miim.c ../src/config/default/driver/miim/src/dynamic/drv_miim_pic32cz.c ../src/config/default/library/tcpip/src/helpers.c ../src/config/default/library/tcpip/src/tcpip_heap_internal.c ../src/config/default/library/tcpip/src/tcpip_heap_alloc.c ../src/config/default/library/tcpip/src/tcpip_manager.c ../src/config/default/library/tcpip/src/ipv4.c ../src/config/default/library/tcpip/src/dns.c ../src/config/default/library/tcpip/src/dhcp.c ../src/config/default/library/tcpip/src/tcpip_helpers.c ../src/config/default/library/tcpip/src/oahash.c ../src/config/default/library/tcpip/src/tcpip_notify.c ../src/config/default/library/tcpip/src/arp.c ../src/config/default/library/tcpip/src/udp.c ../src/config/default/library/tcpip/src/tcpip_packet.c ../src/config/default/library/tcpip/src/hash_fnv.c ../src/config/default/peripheral/clock/plib_clock.c ../src/config/default/peripheral/evsys/plib_evsys.c ../src/config/default/peripheral/fcw/plib_fcw.c ../src/config/default/peripheral/mpu/plib_mpu.c ../src/config/default/peripheral/nvic/plib_nvic.c ../src/config/default/peripheral/port/plib_port.c ../src/config/default/peripheral/rtc/plib_rtc_timer.c ../src/config/default/peripheral/sercom/usart/plib_sercom0_usart.c ../src/config/default/peripheral/wdt/plib_wdt.c ../src/config/default/stdio/xc32_monitor.c ../src/config/default/system/cache/sys_cache.c ../src/config/default/system/console/src/sys_console.c ../src/config/default/system/console/src/sys_console_uart.c ../src/config/default/system/debug/src/sys_debug.c ../src/config/default/system/int/src/sys_int.c ../src/config/default/system/time/src/sys_time.c ../src/config/default/system/sys_time_h2_adapter.c ../src/config/default/initialization.c ../src/config/default/startup_xc32.c ../src/config/default/tasks.c ../src/config/default/interrupts.c ../src/config/default/exceptions.c ../src/config/default/libc_syscalls.c ../src/main.c ../src/app_pic32cz_ca.c

# Pack Options 
PACK_COMMON_OPTIONS=-I "${CMSIS_DIR}/CMSIS/Core/Include"



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/IO_Bootloader.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32CZ4010CA80208
MP_LINKER_FILE_OPTION=,--script="..\src\config\default\PIC32CZ4010CA80208.ld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1277623251/datastream_udp.o: ../src/config/default/bootloader/datastream/datastream_udp.c  .generated_files/flags/default/7dc66b5ed459f4babe3167a07d1649958e4791a9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1277623251" 
	@${RM} ${OBJECTDIR}/_ext/1277623251/datastream_udp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1277623251/datastream_udp.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1277623251/datastream_udp.o.d" -o ${OBJECTDIR}/_ext/1277623251/datastream_udp.o ../src/config/default/bootloader/datastream/datastream_udp.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o: ../src/config/default/bootloader/bootloader_nvm_interface.c  .generated_files/flags/default/205a476cef430f3285317566eed660589cbda3fa .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1748409734" 
	@${RM} ${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o.d 
	@${RM} ${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o.d" -o ${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o ../src/config/default/bootloader/bootloader_nvm_interface.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1748409734/bootloader_udp.o: ../src/config/default/bootloader/bootloader_udp.c  .generated_files/flags/default/af884477562a4fb46d0e8ac90eb8c04133b8e2fb .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1748409734" 
	@${RM} ${OBJECTDIR}/_ext/1748409734/bootloader_udp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1748409734/bootloader_udp.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1748409734/bootloader_udp.o.d" -o ${OBJECTDIR}/_ext/1748409734/bootloader_udp.o ../src/config/default/bootloader/bootloader_udp.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1748409734/bootloader_common.o: ../src/config/default/bootloader/bootloader_common.c  .generated_files/flags/default/eacb3bed844abefeed89805772f1cd5f65e1e699 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1748409734" 
	@${RM} ${OBJECTDIR}/_ext/1748409734/bootloader_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/1748409734/bootloader_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1748409734/bootloader_common.o.d" -o ${OBJECTDIR}/_ext/1748409734/bootloader_common.o ../src/config/default/bootloader/bootloader_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o: ../src/config/default/driver/ethphy/src/dynamic/drv_extphy_ksz8863.c  .generated_files/flags/default/2d248637da66f2feef84a4792678aa50fe3b2dd5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/444070925" 
	@${RM} ${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o.d 
	@${RM} ${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o.d" -o ${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o ../src/config/default/driver/ethphy/src/dynamic/drv_extphy_ksz8863.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/444070925/drv_ethphy.o: ../src/config/default/driver/ethphy/src/dynamic/drv_ethphy.c  .generated_files/flags/default/f0ca1fec75b951cae0cff04edd7e01e188054905 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/444070925" 
	@${RM} ${OBJECTDIR}/_ext/444070925/drv_ethphy.o.d 
	@${RM} ${OBJECTDIR}/_ext/444070925/drv_ethphy.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/444070925/drv_ethphy.o.d" -o ${OBJECTDIR}/_ext/444070925/drv_ethphy.o ../src/config/default/driver/ethphy/src/dynamic/drv_ethphy.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o: ../src/config/default/driver/gmac/src/dynamic/drv_gmac_lib_pic32CZ.c  .generated_files/flags/default/4b26290f760689af9001022b327dacb5ce9ea130 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1546807373" 
	@${RM} ${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o.d 
	@${RM} ${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o.d" -o ${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o ../src/config/default/driver/gmac/src/dynamic/drv_gmac_lib_pic32CZ.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1546807373/drv_gmac.o: ../src/config/default/driver/gmac/src/dynamic/drv_gmac.c  .generated_files/flags/default/8a7f6c40f662f4120e2e9fc442db32a68ecffb77 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1546807373" 
	@${RM} ${OBJECTDIR}/_ext/1546807373/drv_gmac.o.d 
	@${RM} ${OBJECTDIR}/_ext/1546807373/drv_gmac.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1546807373/drv_gmac.o.d" -o ${OBJECTDIR}/_ext/1546807373/drv_gmac.o ../src/config/default/driver/gmac/src/dynamic/drv_gmac.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/438849557/drv_miim.o: ../src/config/default/driver/miim/src/dynamic/drv_miim.c  .generated_files/flags/default/a95307764e8310599333469d25e58e69b9b472b0 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/438849557" 
	@${RM} ${OBJECTDIR}/_ext/438849557/drv_miim.o.d 
	@${RM} ${OBJECTDIR}/_ext/438849557/drv_miim.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/438849557/drv_miim.o.d" -o ${OBJECTDIR}/_ext/438849557/drv_miim.o ../src/config/default/driver/miim/src/dynamic/drv_miim.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o: ../src/config/default/driver/miim/src/dynamic/drv_miim_pic32cz.c  .generated_files/flags/default/f8b7bbd553f5cfe3ee48d061773e6e8ec24f928e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/438849557" 
	@${RM} ${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o.d 
	@${RM} ${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o.d" -o ${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o ../src/config/default/driver/miim/src/dynamic/drv_miim_pic32cz.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/helpers.o: ../src/config/default/library/tcpip/src/helpers.c  .generated_files/flags/default/3cc52ed2792645ab5727bdac04383643a2222fb0 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/helpers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/helpers.o.d" -o ${OBJECTDIR}/_ext/1033058136/helpers.o ../src/config/default/library/tcpip/src/helpers.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o: ../src/config/default/library/tcpip/src/tcpip_heap_internal.c  .generated_files/flags/default/d770c4d6091baa6ae4b52b7f218a65bf57d78c5a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o.d" -o ${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o ../src/config/default/library/tcpip/src/tcpip_heap_internal.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o: ../src/config/default/library/tcpip/src/tcpip_heap_alloc.c  .generated_files/flags/default/db376d780e42979ab4ccaa4948c820d1891ee35f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o.d" -o ${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o ../src/config/default/library/tcpip/src/tcpip_heap_alloc.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/tcpip_manager.o: ../src/config/default/library/tcpip/src/tcpip_manager.c  .generated_files/flags/default/27a9004c200fb23388985a8fcfca57f7ec9c4a70 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_manager.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/tcpip_manager.o.d" -o ${OBJECTDIR}/_ext/1033058136/tcpip_manager.o ../src/config/default/library/tcpip/src/tcpip_manager.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/ipv4.o: ../src/config/default/library/tcpip/src/ipv4.c  .generated_files/flags/default/4b9583c1e27d668fb97a08ad14445e5f4e049dec .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/ipv4.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/ipv4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/ipv4.o.d" -o ${OBJECTDIR}/_ext/1033058136/ipv4.o ../src/config/default/library/tcpip/src/ipv4.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/dns.o: ../src/config/default/library/tcpip/src/dns.c  .generated_files/flags/default/6ea93d78d2e73cb40078715bcab35049c39be348 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/dns.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/dns.o.d" -o ${OBJECTDIR}/_ext/1033058136/dns.o ../src/config/default/library/tcpip/src/dns.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/dhcp.o: ../src/config/default/library/tcpip/src/dhcp.c  .generated_files/flags/default/723c93afb5c02c23ff8021ec71032abac144c3b2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/dhcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/dhcp.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/dhcp.o.d" -o ${OBJECTDIR}/_ext/1033058136/dhcp.o ../src/config/default/library/tcpip/src/dhcp.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o: ../src/config/default/library/tcpip/src/tcpip_helpers.c  .generated_files/flags/default/dc20b3bff19039c35f9689e6686fe1b782aab00d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o.d" -o ${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o ../src/config/default/library/tcpip/src/tcpip_helpers.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/oahash.o: ../src/config/default/library/tcpip/src/oahash.c  .generated_files/flags/default/68d058919288c2eef73173f83cc78736df53b075 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/oahash.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/oahash.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/oahash.o.d" -o ${OBJECTDIR}/_ext/1033058136/oahash.o ../src/config/default/library/tcpip/src/oahash.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/tcpip_notify.o: ../src/config/default/library/tcpip/src/tcpip_notify.c  .generated_files/flags/default/b155a91fdc465f86ccf93f87c90f534a748770d5 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_notify.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_notify.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/tcpip_notify.o.d" -o ${OBJECTDIR}/_ext/1033058136/tcpip_notify.o ../src/config/default/library/tcpip/src/tcpip_notify.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/arp.o: ../src/config/default/library/tcpip/src/arp.c  .generated_files/flags/default/adaadf0c061bc01260f377da744df72cf29ce870 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/arp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/arp.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/arp.o.d" -o ${OBJECTDIR}/_ext/1033058136/arp.o ../src/config/default/library/tcpip/src/arp.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/udp.o: ../src/config/default/library/tcpip/src/udp.c  .generated_files/flags/default/e3e274d3c342261b50d2163bcb01478ab92d454e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/udp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/udp.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/udp.o.d" -o ${OBJECTDIR}/_ext/1033058136/udp.o ../src/config/default/library/tcpip/src/udp.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/tcpip_packet.o: ../src/config/default/library/tcpip/src/tcpip_packet.c  .generated_files/flags/default/15cf48add2aec692b86b484bba130218ab0123f9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_packet.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_packet.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/tcpip_packet.o.d" -o ${OBJECTDIR}/_ext/1033058136/tcpip_packet.o ../src/config/default/library/tcpip/src/tcpip_packet.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/hash_fnv.o: ../src/config/default/library/tcpip/src/hash_fnv.c  .generated_files/flags/default/a018a25c77d474f1d50d6cdc0593829d9198f3cf .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/hash_fnv.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/hash_fnv.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/hash_fnv.o.d" -o ${OBJECTDIR}/_ext/1033058136/hash_fnv.o ../src/config/default/library/tcpip/src/hash_fnv.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1984496892/plib_clock.o: ../src/config/default/peripheral/clock/plib_clock.c  .generated_files/flags/default/cf858c68cd6909cebfa3fa004e22363bbc753a7c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1984496892" 
	@${RM} ${OBJECTDIR}/_ext/1984496892/plib_clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1984496892/plib_clock.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1984496892/plib_clock.o.d" -o ${OBJECTDIR}/_ext/1984496892/plib_clock.o ../src/config/default/peripheral/clock/plib_clock.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1986646378/plib_evsys.o: ../src/config/default/peripheral/evsys/plib_evsys.c  .generated_files/flags/default/e869c9dce830e2ed493c8455ead4ef07fd8faa3f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1986646378" 
	@${RM} ${OBJECTDIR}/_ext/1986646378/plib_evsys.o.d 
	@${RM} ${OBJECTDIR}/_ext/1986646378/plib_evsys.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1986646378/plib_evsys.o.d" -o ${OBJECTDIR}/_ext/1986646378/plib_evsys.o ../src/config/default/peripheral/evsys/plib_evsys.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60168136/plib_fcw.o: ../src/config/default/peripheral/fcw/plib_fcw.c  .generated_files/flags/default/50d9dc88e28e0e8ca34a67edddf74eb8e24a235e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/60168136" 
	@${RM} ${OBJECTDIR}/_ext/60168136/plib_fcw.o.d 
	@${RM} ${OBJECTDIR}/_ext/60168136/plib_fcw.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60168136/plib_fcw.o.d" -o ${OBJECTDIR}/_ext/60168136/plib_fcw.o ../src/config/default/peripheral/fcw/plib_fcw.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60175264/plib_mpu.o: ../src/config/default/peripheral/mpu/plib_mpu.c  .generated_files/flags/default/6cb2f89827c0e5ec6eb8b1ef16241bb16cbf3ab0 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/60175264" 
	@${RM} ${OBJECTDIR}/_ext/60175264/plib_mpu.o.d 
	@${RM} ${OBJECTDIR}/_ext/60175264/plib_mpu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60175264/plib_mpu.o.d" -o ${OBJECTDIR}/_ext/60175264/plib_mpu.o ../src/config/default/peripheral/mpu/plib_mpu.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1865468468/plib_nvic.o: ../src/config/default/peripheral/nvic/plib_nvic.c  .generated_files/flags/default/f74ed3cf8a0a2e4a8d50e40aab2beafb78e9f11b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1865468468" 
	@${RM} ${OBJECTDIR}/_ext/1865468468/plib_nvic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865468468/plib_nvic.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1865468468/plib_nvic.o.d" -o ${OBJECTDIR}/_ext/1865468468/plib_nvic.o ../src/config/default/peripheral/nvic/plib_nvic.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1865521619/plib_port.o: ../src/config/default/peripheral/port/plib_port.c  .generated_files/flags/default/a6b7f1e36172e08033a7565ef90ba3211b8c46a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1865521619" 
	@${RM} ${OBJECTDIR}/_ext/1865521619/plib_port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865521619/plib_port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1865521619/plib_port.o.d" -o ${OBJECTDIR}/_ext/1865521619/plib_port.o ../src/config/default/peripheral/port/plib_port.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o: ../src/config/default/peripheral/rtc/plib_rtc_timer.c  .generated_files/flags/default/8b89779032b629d9f9d5577ef38bde7d03441b31 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/60180175" 
	@${RM} ${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o.d" -o ${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o ../src/config/default/peripheral/rtc/plib_rtc_timer.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o: ../src/config/default/peripheral/sercom/usart/plib_sercom0_usart.c  .generated_files/flags/default/f7dd1373a8c5fa8a32171b61be757e9c5e024271 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/504274921" 
	@${RM} ${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o.d" -o ${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o ../src/config/default/peripheral/sercom/usart/plib_sercom0_usart.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60184501/plib_wdt.o: ../src/config/default/peripheral/wdt/plib_wdt.c  .generated_files/flags/default/99efff219a5df8e63523543cb9fa70feda867c79 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/60184501" 
	@${RM} ${OBJECTDIR}/_ext/60184501/plib_wdt.o.d 
	@${RM} ${OBJECTDIR}/_ext/60184501/plib_wdt.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60184501/plib_wdt.o.d" -o ${OBJECTDIR}/_ext/60184501/plib_wdt.o ../src/config/default/peripheral/wdt/plib_wdt.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/163028504/xc32_monitor.o: ../src/config/default/stdio/xc32_monitor.c  .generated_files/flags/default/ecbcaf231cbe3c462fd91fa9e3881a2329212a9a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/163028504" 
	@${RM} ${OBJECTDIR}/_ext/163028504/xc32_monitor.o.d 
	@${RM} ${OBJECTDIR}/_ext/163028504/xc32_monitor.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/163028504/xc32_monitor.o.d" -o ${OBJECTDIR}/_ext/163028504/xc32_monitor.o ../src/config/default/stdio/xc32_monitor.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1014039709/sys_cache.o: ../src/config/default/system/cache/sys_cache.c  .generated_files/flags/default/f45746437ac1317c623d284dccad06f4666918af .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1014039709" 
	@${RM} ${OBJECTDIR}/_ext/1014039709/sys_cache.o.d 
	@${RM} ${OBJECTDIR}/_ext/1014039709/sys_cache.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1014039709/sys_cache.o.d" -o ${OBJECTDIR}/_ext/1014039709/sys_cache.o ../src/config/default/system/cache/sys_cache.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1832805299/sys_console.o: ../src/config/default/system/console/src/sys_console.c  .generated_files/flags/default/4722d592c07d9ee0abfddfafc04b8cd0fad904b4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1832805299" 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console.o.d 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1832805299/sys_console.o.d" -o ${OBJECTDIR}/_ext/1832805299/sys_console.o ../src/config/default/system/console/src/sys_console.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1832805299/sys_console_uart.o: ../src/config/default/system/console/src/sys_console_uart.c  .generated_files/flags/default/ca18ac72311b79319250908fd5aec7f21f9a270b .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1832805299" 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console_uart.o.d 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console_uart.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1832805299/sys_console_uart.o.d" -o ${OBJECTDIR}/_ext/1832805299/sys_console_uart.o ../src/config/default/system/console/src/sys_console_uart.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/944882569/sys_debug.o: ../src/config/default/system/debug/src/sys_debug.c  .generated_files/flags/default/a4bbd15cb6743976e10c674d9a477ccdcf4bec9f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/944882569" 
	@${RM} ${OBJECTDIR}/_ext/944882569/sys_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/944882569/sys_debug.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/944882569/sys_debug.o.d" -o ${OBJECTDIR}/_ext/944882569/sys_debug.o ../src/config/default/system/debug/src/sys_debug.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1881668453/sys_int.o: ../src/config/default/system/int/src/sys_int.c  .generated_files/flags/default/60c39b38d6f16f00f22bf9afa9d0cf550ca31723 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1881668453" 
	@${RM} ${OBJECTDIR}/_ext/1881668453/sys_int.o.d 
	@${RM} ${OBJECTDIR}/_ext/1881668453/sys_int.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1881668453/sys_int.o.d" -o ${OBJECTDIR}/_ext/1881668453/sys_int.o ../src/config/default/system/int/src/sys_int.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/101884895/sys_time.o: ../src/config/default/system/time/src/sys_time.c  .generated_files/flags/default/6297d0044706f7a21f0283210b3648c934736b4c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/101884895" 
	@${RM} ${OBJECTDIR}/_ext/101884895/sys_time.o.d 
	@${RM} ${OBJECTDIR}/_ext/101884895/sys_time.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/101884895/sys_time.o.d" -o ${OBJECTDIR}/_ext/101884895/sys_time.o ../src/config/default/system/time/src/sys_time.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o: ../src/config/default/system/sys_time_h2_adapter.c  .generated_files/flags/default/2f7d51013570daf622ce1ff1437ecd4df10f46d9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/753841488" 
	@${RM} ${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o.d 
	@${RM} ${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o.d" -o ${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o ../src/config/default/system/sys_time_h2_adapter.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/initialization.o: ../src/config/default/initialization.c  .generated_files/flags/default/11dae67f1f0d68d0908c463f5c186168ecf0d55 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/initialization.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/initialization.o.d" -o ${OBJECTDIR}/_ext/1171490990/initialization.o ../src/config/default/initialization.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/startup_xc32.o: ../src/config/default/startup_xc32.c  .generated_files/flags/default/b0e1a6f416411e1492961e8fc2d931809569b93f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/startup_xc32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/startup_xc32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/startup_xc32.o.d" -o ${OBJECTDIR}/_ext/1171490990/startup_xc32.o ../src/config/default/startup_xc32.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/tasks.o: ../src/config/default/tasks.c  .generated_files/flags/default/18266fc2017af0a40a9aed5f811b0d9c1a38c2ca .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/tasks.o.d" -o ${OBJECTDIR}/_ext/1171490990/tasks.o ../src/config/default/tasks.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/interrupts.o: ../src/config/default/interrupts.c  .generated_files/flags/default/4e261684888ffd7f3d5e4f62acbf98161cb02ad8 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/interrupts.o.d" -o ${OBJECTDIR}/_ext/1171490990/interrupts.o ../src/config/default/interrupts.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/exceptions.o: ../src/config/default/exceptions.c  .generated_files/flags/default/d2f86cc76d7a70d5d21fb579993a797a6383e090 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/exceptions.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/exceptions.o.d" -o ${OBJECTDIR}/_ext/1171490990/exceptions.o ../src/config/default/exceptions.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/libc_syscalls.o: ../src/config/default/libc_syscalls.c  .generated_files/flags/default/be380b1d2600d62e4daf43d9aa29740a3b9a3a4a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/libc_syscalls.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/libc_syscalls.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/libc_syscalls.o.d" -o ${OBJECTDIR}/_ext/1171490990/libc_syscalls.o ../src/config/default/libc_syscalls.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  .generated_files/flags/default/70a6ed2f60774247d0e2120a5248aca24a5fcbe .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o: ../src/app_pic32cz_ca.c  .generated_files/flags/default/48688275ff50c7254508ed650132d2491a8eaf3d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o.d" -o ${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o ../src/app_pic32cz_ca.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
else
${OBJECTDIR}/_ext/1277623251/datastream_udp.o: ../src/config/default/bootloader/datastream/datastream_udp.c  .generated_files/flags/default/c3ad713258e3b2e6f76c7f07fc508228c77af1c6 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1277623251" 
	@${RM} ${OBJECTDIR}/_ext/1277623251/datastream_udp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1277623251/datastream_udp.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1277623251/datastream_udp.o.d" -o ${OBJECTDIR}/_ext/1277623251/datastream_udp.o ../src/config/default/bootloader/datastream/datastream_udp.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o: ../src/config/default/bootloader/bootloader_nvm_interface.c  .generated_files/flags/default/fe57f22b6a02e953b733f767e4508f6389dffefd .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1748409734" 
	@${RM} ${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o.d 
	@${RM} ${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o.d" -o ${OBJECTDIR}/_ext/1748409734/bootloader_nvm_interface.o ../src/config/default/bootloader/bootloader_nvm_interface.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1748409734/bootloader_udp.o: ../src/config/default/bootloader/bootloader_udp.c  .generated_files/flags/default/bd7144090d37de63c5bc68bb30934b3c8b59db8a .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1748409734" 
	@${RM} ${OBJECTDIR}/_ext/1748409734/bootloader_udp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1748409734/bootloader_udp.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1748409734/bootloader_udp.o.d" -o ${OBJECTDIR}/_ext/1748409734/bootloader_udp.o ../src/config/default/bootloader/bootloader_udp.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1748409734/bootloader_common.o: ../src/config/default/bootloader/bootloader_common.c  .generated_files/flags/default/59ddec93a329de3b5f839f4574c4e7cd75e778fc .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1748409734" 
	@${RM} ${OBJECTDIR}/_ext/1748409734/bootloader_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/1748409734/bootloader_common.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1748409734/bootloader_common.o.d" -o ${OBJECTDIR}/_ext/1748409734/bootloader_common.o ../src/config/default/bootloader/bootloader_common.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o: ../src/config/default/driver/ethphy/src/dynamic/drv_extphy_ksz8863.c  .generated_files/flags/default/8ddc8e51a204de44b020a6a5ab5ab013e93fc6ae .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/444070925" 
	@${RM} ${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o.d 
	@${RM} ${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o.d" -o ${OBJECTDIR}/_ext/444070925/drv_extphy_ksz8863.o ../src/config/default/driver/ethphy/src/dynamic/drv_extphy_ksz8863.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/444070925/drv_ethphy.o: ../src/config/default/driver/ethphy/src/dynamic/drv_ethphy.c  .generated_files/flags/default/352e459e18cbbdafe50e34150ebc6bfa017d009e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/444070925" 
	@${RM} ${OBJECTDIR}/_ext/444070925/drv_ethphy.o.d 
	@${RM} ${OBJECTDIR}/_ext/444070925/drv_ethphy.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/444070925/drv_ethphy.o.d" -o ${OBJECTDIR}/_ext/444070925/drv_ethphy.o ../src/config/default/driver/ethphy/src/dynamic/drv_ethphy.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o: ../src/config/default/driver/gmac/src/dynamic/drv_gmac_lib_pic32CZ.c  .generated_files/flags/default/5d29bba3f259a525fd46d61a993fff9052661ed4 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1546807373" 
	@${RM} ${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o.d 
	@${RM} ${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o.d" -o ${OBJECTDIR}/_ext/1546807373/drv_gmac_lib_pic32CZ.o ../src/config/default/driver/gmac/src/dynamic/drv_gmac_lib_pic32CZ.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1546807373/drv_gmac.o: ../src/config/default/driver/gmac/src/dynamic/drv_gmac.c  .generated_files/flags/default/385290308726cd7e1697175e2332f10de4d1375 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1546807373" 
	@${RM} ${OBJECTDIR}/_ext/1546807373/drv_gmac.o.d 
	@${RM} ${OBJECTDIR}/_ext/1546807373/drv_gmac.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1546807373/drv_gmac.o.d" -o ${OBJECTDIR}/_ext/1546807373/drv_gmac.o ../src/config/default/driver/gmac/src/dynamic/drv_gmac.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/438849557/drv_miim.o: ../src/config/default/driver/miim/src/dynamic/drv_miim.c  .generated_files/flags/default/52ed5904abaa500d39c9b4c77280da1558f3d066 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/438849557" 
	@${RM} ${OBJECTDIR}/_ext/438849557/drv_miim.o.d 
	@${RM} ${OBJECTDIR}/_ext/438849557/drv_miim.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/438849557/drv_miim.o.d" -o ${OBJECTDIR}/_ext/438849557/drv_miim.o ../src/config/default/driver/miim/src/dynamic/drv_miim.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o: ../src/config/default/driver/miim/src/dynamic/drv_miim_pic32cz.c  .generated_files/flags/default/4337f66255f2dd2c5136b89efc07c207850cb3da .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/438849557" 
	@${RM} ${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o.d 
	@${RM} ${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o.d" -o ${OBJECTDIR}/_ext/438849557/drv_miim_pic32cz.o ../src/config/default/driver/miim/src/dynamic/drv_miim_pic32cz.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/helpers.o: ../src/config/default/library/tcpip/src/helpers.c  .generated_files/flags/default/cdda3a725bcef67fc58219ce7c5896391bd8027f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/helpers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/helpers.o.d" -o ${OBJECTDIR}/_ext/1033058136/helpers.o ../src/config/default/library/tcpip/src/helpers.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o: ../src/config/default/library/tcpip/src/tcpip_heap_internal.c  .generated_files/flags/default/e06e4d9dcc0c1b44f9e140c74efd57f1fd2c0b87 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o.d" -o ${OBJECTDIR}/_ext/1033058136/tcpip_heap_internal.o ../src/config/default/library/tcpip/src/tcpip_heap_internal.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o: ../src/config/default/library/tcpip/src/tcpip_heap_alloc.c  .generated_files/flags/default/12c1a719b1db0e634ae0dccb671ad96720d774a2 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o.d" -o ${OBJECTDIR}/_ext/1033058136/tcpip_heap_alloc.o ../src/config/default/library/tcpip/src/tcpip_heap_alloc.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/tcpip_manager.o: ../src/config/default/library/tcpip/src/tcpip_manager.c  .generated_files/flags/default/91a25fed3643bfdffc2c41f497aaeb24944bc0a0 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_manager.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_manager.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/tcpip_manager.o.d" -o ${OBJECTDIR}/_ext/1033058136/tcpip_manager.o ../src/config/default/library/tcpip/src/tcpip_manager.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/ipv4.o: ../src/config/default/library/tcpip/src/ipv4.c  .generated_files/flags/default/6e3b6a002009dae3663f17158f719ffd9090eb08 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/ipv4.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/ipv4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/ipv4.o.d" -o ${OBJECTDIR}/_ext/1033058136/ipv4.o ../src/config/default/library/tcpip/src/ipv4.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/dns.o: ../src/config/default/library/tcpip/src/dns.c  .generated_files/flags/default/6bb88d4edea917263bec3d94e2ba6119c5ce802d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/dns.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/dns.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/dns.o.d" -o ${OBJECTDIR}/_ext/1033058136/dns.o ../src/config/default/library/tcpip/src/dns.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/dhcp.o: ../src/config/default/library/tcpip/src/dhcp.c  .generated_files/flags/default/16cf11fb6114b89ee63c114c7b5bdb58d1c6ee16 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/dhcp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/dhcp.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/dhcp.o.d" -o ${OBJECTDIR}/_ext/1033058136/dhcp.o ../src/config/default/library/tcpip/src/dhcp.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o: ../src/config/default/library/tcpip/src/tcpip_helpers.c  .generated_files/flags/default/3f3326d5100c9aedab9ef3306685ba3fb204422d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o.d" -o ${OBJECTDIR}/_ext/1033058136/tcpip_helpers.o ../src/config/default/library/tcpip/src/tcpip_helpers.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/oahash.o: ../src/config/default/library/tcpip/src/oahash.c  .generated_files/flags/default/5bd2b0f87dd58085abb81631e4d36141bff2de66 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/oahash.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/oahash.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/oahash.o.d" -o ${OBJECTDIR}/_ext/1033058136/oahash.o ../src/config/default/library/tcpip/src/oahash.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/tcpip_notify.o: ../src/config/default/library/tcpip/src/tcpip_notify.c  .generated_files/flags/default/d84b91e6fb7da45d67d2010f91cab66e4e037337 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_notify.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_notify.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/tcpip_notify.o.d" -o ${OBJECTDIR}/_ext/1033058136/tcpip_notify.o ../src/config/default/library/tcpip/src/tcpip_notify.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/arp.o: ../src/config/default/library/tcpip/src/arp.c  .generated_files/flags/default/75c27459ff57319e2f1d8a4a28cb8f023c97587f .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/arp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/arp.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/arp.o.d" -o ${OBJECTDIR}/_ext/1033058136/arp.o ../src/config/default/library/tcpip/src/arp.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/udp.o: ../src/config/default/library/tcpip/src/udp.c  .generated_files/flags/default/e168288cdbe6cf6393788b215a9243bd9cc94a44 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/udp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/udp.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/udp.o.d" -o ${OBJECTDIR}/_ext/1033058136/udp.o ../src/config/default/library/tcpip/src/udp.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/tcpip_packet.o: ../src/config/default/library/tcpip/src/tcpip_packet.c  .generated_files/flags/default/a58725abfc09b67911a1b65c5b1df95a418a6335 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_packet.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/tcpip_packet.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/tcpip_packet.o.d" -o ${OBJECTDIR}/_ext/1033058136/tcpip_packet.o ../src/config/default/library/tcpip/src/tcpip_packet.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1033058136/hash_fnv.o: ../src/config/default/library/tcpip/src/hash_fnv.c  .generated_files/flags/default/ce38076f9dd3c825c19952ffae24aaf2b2d1d94c .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1033058136" 
	@${RM} ${OBJECTDIR}/_ext/1033058136/hash_fnv.o.d 
	@${RM} ${OBJECTDIR}/_ext/1033058136/hash_fnv.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1033058136/hash_fnv.o.d" -o ${OBJECTDIR}/_ext/1033058136/hash_fnv.o ../src/config/default/library/tcpip/src/hash_fnv.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1984496892/plib_clock.o: ../src/config/default/peripheral/clock/plib_clock.c  .generated_files/flags/default/c68f529275282fc4cd76bbe15113daee4b8f413d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1984496892" 
	@${RM} ${OBJECTDIR}/_ext/1984496892/plib_clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1984496892/plib_clock.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1984496892/plib_clock.o.d" -o ${OBJECTDIR}/_ext/1984496892/plib_clock.o ../src/config/default/peripheral/clock/plib_clock.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1986646378/plib_evsys.o: ../src/config/default/peripheral/evsys/plib_evsys.c  .generated_files/flags/default/70c49961314d0528950936156baf67e13a53af0e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1986646378" 
	@${RM} ${OBJECTDIR}/_ext/1986646378/plib_evsys.o.d 
	@${RM} ${OBJECTDIR}/_ext/1986646378/plib_evsys.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1986646378/plib_evsys.o.d" -o ${OBJECTDIR}/_ext/1986646378/plib_evsys.o ../src/config/default/peripheral/evsys/plib_evsys.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60168136/plib_fcw.o: ../src/config/default/peripheral/fcw/plib_fcw.c  .generated_files/flags/default/53f07ecff1d909360a73804776979b6358932be9 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/60168136" 
	@${RM} ${OBJECTDIR}/_ext/60168136/plib_fcw.o.d 
	@${RM} ${OBJECTDIR}/_ext/60168136/plib_fcw.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60168136/plib_fcw.o.d" -o ${OBJECTDIR}/_ext/60168136/plib_fcw.o ../src/config/default/peripheral/fcw/plib_fcw.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60175264/plib_mpu.o: ../src/config/default/peripheral/mpu/plib_mpu.c  .generated_files/flags/default/dc2953f091611de9d27ddaf6a9993a50e8fc2e45 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/60175264" 
	@${RM} ${OBJECTDIR}/_ext/60175264/plib_mpu.o.d 
	@${RM} ${OBJECTDIR}/_ext/60175264/plib_mpu.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60175264/plib_mpu.o.d" -o ${OBJECTDIR}/_ext/60175264/plib_mpu.o ../src/config/default/peripheral/mpu/plib_mpu.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1865468468/plib_nvic.o: ../src/config/default/peripheral/nvic/plib_nvic.c  .generated_files/flags/default/7b79030318678f35f21a711cdcfc6cea5e158030 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1865468468" 
	@${RM} ${OBJECTDIR}/_ext/1865468468/plib_nvic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865468468/plib_nvic.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1865468468/plib_nvic.o.d" -o ${OBJECTDIR}/_ext/1865468468/plib_nvic.o ../src/config/default/peripheral/nvic/plib_nvic.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1865521619/plib_port.o: ../src/config/default/peripheral/port/plib_port.c  .generated_files/flags/default/215edb80449b746834e77e9f39313499e13d08cc .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1865521619" 
	@${RM} ${OBJECTDIR}/_ext/1865521619/plib_port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1865521619/plib_port.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1865521619/plib_port.o.d" -o ${OBJECTDIR}/_ext/1865521619/plib_port.o ../src/config/default/peripheral/port/plib_port.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o: ../src/config/default/peripheral/rtc/plib_rtc_timer.c  .generated_files/flags/default/538da838ec6202698ca9ab5c11ffba81843fffad .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/60180175" 
	@${RM} ${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o.d" -o ${OBJECTDIR}/_ext/60180175/plib_rtc_timer.o ../src/config/default/peripheral/rtc/plib_rtc_timer.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o: ../src/config/default/peripheral/sercom/usart/plib_sercom0_usart.c  .generated_files/flags/default/e9e797c816618c67aa74a7d707bd7194ddc7b31e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/504274921" 
	@${RM} ${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o.d" -o ${OBJECTDIR}/_ext/504274921/plib_sercom0_usart.o ../src/config/default/peripheral/sercom/usart/plib_sercom0_usart.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/60184501/plib_wdt.o: ../src/config/default/peripheral/wdt/plib_wdt.c  .generated_files/flags/default/a153e1b52d6ffc93601512a11b98bbb2f33d8f84 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/60184501" 
	@${RM} ${OBJECTDIR}/_ext/60184501/plib_wdt.o.d 
	@${RM} ${OBJECTDIR}/_ext/60184501/plib_wdt.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60184501/plib_wdt.o.d" -o ${OBJECTDIR}/_ext/60184501/plib_wdt.o ../src/config/default/peripheral/wdt/plib_wdt.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/163028504/xc32_monitor.o: ../src/config/default/stdio/xc32_monitor.c  .generated_files/flags/default/150187e11096e7903d7a80c4ad6d6eec0f0cc384 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/163028504" 
	@${RM} ${OBJECTDIR}/_ext/163028504/xc32_monitor.o.d 
	@${RM} ${OBJECTDIR}/_ext/163028504/xc32_monitor.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/163028504/xc32_monitor.o.d" -o ${OBJECTDIR}/_ext/163028504/xc32_monitor.o ../src/config/default/stdio/xc32_monitor.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1014039709/sys_cache.o: ../src/config/default/system/cache/sys_cache.c  .generated_files/flags/default/7975f8c5aeb521e1b2f744d8cc958c7ab66e45a7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1014039709" 
	@${RM} ${OBJECTDIR}/_ext/1014039709/sys_cache.o.d 
	@${RM} ${OBJECTDIR}/_ext/1014039709/sys_cache.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1014039709/sys_cache.o.d" -o ${OBJECTDIR}/_ext/1014039709/sys_cache.o ../src/config/default/system/cache/sys_cache.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1832805299/sys_console.o: ../src/config/default/system/console/src/sys_console.c  .generated_files/flags/default/ef7f6b09685c3c01c64b0de42670f19dd669834d .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1832805299" 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console.o.d 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1832805299/sys_console.o.d" -o ${OBJECTDIR}/_ext/1832805299/sys_console.o ../src/config/default/system/console/src/sys_console.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1832805299/sys_console_uart.o: ../src/config/default/system/console/src/sys_console_uart.c  .generated_files/flags/default/9d80396b00fbb6a0beb45d1be63a0d618a679fab .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1832805299" 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console_uart.o.d 
	@${RM} ${OBJECTDIR}/_ext/1832805299/sys_console_uart.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1832805299/sys_console_uart.o.d" -o ${OBJECTDIR}/_ext/1832805299/sys_console_uart.o ../src/config/default/system/console/src/sys_console_uart.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/944882569/sys_debug.o: ../src/config/default/system/debug/src/sys_debug.c  .generated_files/flags/default/66a6417ad5758cc89d22d9992dc6b130c7ef8746 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/944882569" 
	@${RM} ${OBJECTDIR}/_ext/944882569/sys_debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/944882569/sys_debug.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/944882569/sys_debug.o.d" -o ${OBJECTDIR}/_ext/944882569/sys_debug.o ../src/config/default/system/debug/src/sys_debug.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1881668453/sys_int.o: ../src/config/default/system/int/src/sys_int.c  .generated_files/flags/default/4e87a4910b55e4df040b05da590c256304a9e393 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1881668453" 
	@${RM} ${OBJECTDIR}/_ext/1881668453/sys_int.o.d 
	@${RM} ${OBJECTDIR}/_ext/1881668453/sys_int.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1881668453/sys_int.o.d" -o ${OBJECTDIR}/_ext/1881668453/sys_int.o ../src/config/default/system/int/src/sys_int.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/101884895/sys_time.o: ../src/config/default/system/time/src/sys_time.c  .generated_files/flags/default/41b3d63cf7ab79afcc9b123343aaf8e91c9ecd7 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/101884895" 
	@${RM} ${OBJECTDIR}/_ext/101884895/sys_time.o.d 
	@${RM} ${OBJECTDIR}/_ext/101884895/sys_time.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/101884895/sys_time.o.d" -o ${OBJECTDIR}/_ext/101884895/sys_time.o ../src/config/default/system/time/src/sys_time.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o: ../src/config/default/system/sys_time_h2_adapter.c  .generated_files/flags/default/487fa2ea4011a6181a23b26ba731765c9c146366 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/753841488" 
	@${RM} ${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o.d 
	@${RM} ${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o.d" -o ${OBJECTDIR}/_ext/753841488/sys_time_h2_adapter.o ../src/config/default/system/sys_time_h2_adapter.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/initialization.o: ../src/config/default/initialization.c  .generated_files/flags/default/b78a1741cd6eb8edad08b292fc3d335d2f3dd9de .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/initialization.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/initialization.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/initialization.o.d" -o ${OBJECTDIR}/_ext/1171490990/initialization.o ../src/config/default/initialization.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/startup_xc32.o: ../src/config/default/startup_xc32.c  .generated_files/flags/default/723296933a8e268b88ab2aea374be26a11bd5e40 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/startup_xc32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/startup_xc32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/startup_xc32.o.d" -o ${OBJECTDIR}/_ext/1171490990/startup_xc32.o ../src/config/default/startup_xc32.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/tasks.o: ../src/config/default/tasks.c  .generated_files/flags/default/26d473686929413e4c2a7580604b497f9d40401e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/tasks.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/tasks.o.d" -o ${OBJECTDIR}/_ext/1171490990/tasks.o ../src/config/default/tasks.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/interrupts.o: ../src/config/default/interrupts.c  .generated_files/flags/default/802681a4d13ae0f39f2ece6bec08967327113814 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/interrupts.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/interrupts.o.d" -o ${OBJECTDIR}/_ext/1171490990/interrupts.o ../src/config/default/interrupts.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/exceptions.o: ../src/config/default/exceptions.c  .generated_files/flags/default/d8977f2ea2d97dfe0ed846a4f8bbdbf05fcfd6ce .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/exceptions.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/exceptions.o.d" -o ${OBJECTDIR}/_ext/1171490990/exceptions.o ../src/config/default/exceptions.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1171490990/libc_syscalls.o: ../src/config/default/libc_syscalls.c  .generated_files/flags/default/d3b68d8f083609e8b0b0ffa9160a7e79b14e50e .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1171490990" 
	@${RM} ${OBJECTDIR}/_ext/1171490990/libc_syscalls.o.d 
	@${RM} ${OBJECTDIR}/_ext/1171490990/libc_syscalls.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1171490990/libc_syscalls.o.d" -o ${OBJECTDIR}/_ext/1171490990/libc_syscalls.o ../src/config/default/libc_syscalls.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  .generated_files/flags/default/a7d840913eed41d5a3b67c79590820558a1f4aae .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o: ../src/app_pic32cz_ca.c  .generated_files/flags/default/291c38ea939014ec61ade0f3fb7c32111ab93d04 .generated_files/flags/default/da39a3ee5e6b4b0d3255bfef95601890afd80709
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -fdata-sections -O2 -fno-common -I"../src" -I"../src/config/default" -I"../src/config/default/library" -I"../src/config/default/library/tcpip/src" -I"../src/config/default/library/tcpip/src/common" -I"../src/packs/CMSIS/" -I"../src/packs/CMSIS/CMSIS/Core/Include" -I"../src/packs/PIC32CZ4010CA80208_DFP" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o.d" -o ${OBJECTDIR}/_ext/1360937237/app_pic32cz_ca.o ../src/app_pic32cz_ca.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}/CA80" ${PACK_COMMON_OPTIONS} 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/IO_Bootloader.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    ../src/config/default/PIC32CZ4010CA80208.ld
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g   -mprocessor=$(MP_PROCESSOR_OPTION)  -mno-device-startup-code -o ${DISTDIR}/IO_Bootloader.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=_min_heap_size=98304,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",-DRAM_LENGTH=0xffff0,-DRAM_ORIGIN=0x20020010,-DROM_LENGTH=0x10800,-DROM_ORIGIN="BOOT_ROM_ORIGIN + 1024",--memorysummary,${DISTDIR}/memoryfile.xml -mdfp="${DFP_DIR}/CA80"
	
else
${DISTDIR}/IO_Bootloader.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   ../src/config/default/PIC32CZ4010CA80208.ld
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -mno-device-startup-code -o ${DISTDIR}/IO_Bootloader.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=98304,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",-DRAM_LENGTH=0xffff0,-DRAM_ORIGIN=0x20020010,-DROM_LENGTH=0x10800,-DROM_ORIGIN="BOOT_ROM_ORIGIN + 1024",--memorysummary,${DISTDIR}/memoryfile.xml -mdfp="${DFP_DIR}/CA80"
	${MP_CC_DIR}\\xc32-bin2hex ${DISTDIR}/IO_Bootloader.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(wildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
