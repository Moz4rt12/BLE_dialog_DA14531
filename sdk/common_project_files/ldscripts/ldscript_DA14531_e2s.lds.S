/*
 ****************************************************************************************
 *
 * @file ldscript_DA14531.lds.S
 *
 * @brief Common GNU LD linker script file.
 *
 * Copyright (C) 2018-2025 Renesas Electronics Corporation and/or its affiliates.
 * All rights reserved. Confidential Information.
 *
 * This software ("Software") is supplied by Renesas Electronics Corporation and/or its
 * affiliates ("Renesas"). Renesas grants you a personal, non-exclusive, non-transferable,
 * revocable, non-sub-licensable right and license to use the Software, solely if used in
 * or together with Renesas products. You may make copies of this Software, provided this
 * copyright notice and disclaimer ("Notice") is included in all such copies. Renesas
 * reserves the right to change or discontinue the Software at any time without notice.
 *
 * THE SOFTWARE IS PROVIDED "AS IS". RENESAS DISCLAIMS ALL WARRANTIES OF ANY KIND,
 * WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. TO THE
 * MAXIMUM EXTENT PERMITTED UNDER LAW, IN NO EVENT SHALL RENESAS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE, EVEN IF RENESAS HAS BEEN ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGES. USE OF THIS SOFTWARE MAY BE SUBJECT TO TERMS AND CONDITIONS CONTAINED IN
 * AN ADDITIONAL AGREEMENT BETWEEN YOU AND RENESAS. IN CASE OF CONFLICT BETWEEN THE TERMS
 * OF THIS NOTICE AND ANY SUCH ADDITIONAL LICENSE AGREEMENT, THE TERMS OF THE AGREEMENT
 * SHALL TAKE PRECEDENCE. BY CONTINUING TO USE THIS SOFTWARE, YOU AGREE TO THE TERMS OF
 * THIS NOTICE.IF YOU DO NOT AGREE TO THESE TERMS, YOU ARE NOT PERMITTED TO USE THIS
 * SOFTWARE.
 *
 ****************************************************************************************
 */

#include "da1458x_config_basic.h"
#include "da1458x_config_advanced.h"
#include "da1458x_scatter_config.h"

#include "mem_DA14531.lds"

__ER_IROM3_BASE__   = ORIGIN(LR_IROM3);
__ER_IROM3_LENGTH__ = LENGTH(LR_IROM3);

/* Set the entry instruction */
ENTRY(Reset_Handler)

/* Sections layout */
SECTIONS
{
    /* Reset vector at the beginning of RAM */
    ER_IROM1 :
    {
        KEEP(*(.vectors))
        KEEP(*(otp_cs_booter))

    } > LR_IROM1

    ER_IROM2 :
    {
        KEEP(*(patch_table))
    } > LR_IROM2

    ER_IROM3 :
    {
        KEEP(*(.text.disp_heaplog))
        *(.text*)

        . = ALIGN(4);

        KEEP(*(.init))
        KEEP(*(.fini))
        *(.rodata*)
    } > LR_IROM3

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > LR_IROM3

    __exidx_start = .;
    .ARM.exidx :
    {
       *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > LR_IROM3
    __exidx_end = .;

    .copy.table :
    {
        . = ALIGN(4);
        __copy_table_start__ = .;
        LONG (LOADADDR(.data))
        LONG (__data_start__)
        LONG ((__data_end__ - __data_start__)/4)
        __copy_table_end__ = .;
    } > LR_IROM3

    .zero.table :
    {
        . = ALIGN(4);
        __zero_table_start__ = .;
        LONG (__bss_start__)
        LONG ((__bss_end__ - __bss_start__)/4)

        LONG (__ret_data_start__)
        LONG ((__ret_data_end__ - __ret_data_start__)/4)

        __zero_table_end__ = .;
    } > LR_IROM3


    /* Location counter can end up 2byte aligned with narrow Thumb code but
     * __etext is assumed by startup code to be the LMA of a section in RAM
     * which must be 4byte aligned
     */
    __etext = ALIGN(4);

    /* The initialised data section is stored immediately
     * at the end of the text section.
     */
    .data : AT (__etext)
    {
        . = ALIGN(4);
        __data_start__ = .;
        *(vtable)
        *(.data*)

        . = ALIGN(4);
        /* preinit data */
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(4);
        /* init data */
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(4);
        /* finit data */
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        KEEP(*(.jcr*))
        . = ALIGN(4);
        /* All data end */
        __data_end__ = .;
    } > LR_IROM3
    __data_start = ADDR(.data);
    __data_size = 0;
    __data_source = LOADADDR(.data);
    __tls_base = 0;
    __arm32_tls_tcb_offset = 0;

    __code_area_end__ = .;

    .bss :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > LR_IROM3
    __bss_start = __bss_start__;
    __bss_size = SIZEOF(.bss);

    ER_PRODTEST (NOLOAD):
    {
        . = ALIGN(4);
        *(prodtest_uninit)
    } > LR_IROM3

    ER_NZI (NOLOAD) :
    {
        . = ALIGN(4);
        __heap_mem_area_not_ret_start__ = .;
        *jump_table.o(heap_mem_area_not_ret)    /* not retained HEAP */
        __heap_mem_area_not_ret_end__ = .;
    } > LR_IROM3

    /* After the zero-init sections goes the non retained heap and stack */
    .heap (NOLOAD):
    {
        __end__ = .;
        PROVIDE(end = .);
        *(.heap*)
        __HeapLimit = .;
    } > LR_IROM3

    .stack_dummy (NOLOAD):
    {
        __StackLimit = .;
        KEEP(*(stack))
        __StackTop = .;
    } > LR_IROM3

    PROVIDE (__stack = __StackTop);

    /* Execution region needed for stateful hibernation. */
    ER_STATEFUL_HIBERNATION (NOLOAD) :
    {
        KEEP(*(stateful_hibernation))
    } > LR_IROM3

    /* Check if data + heap + stack exceeds RAM limit */
    ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")

    /* Exact address placement to force first section in LR_RETAINED_RAM0 alignment to single word boundary.
     * By default linker aligns it in two-word boundary even when LR_RETAINED_RAM0 starts at a word boundary.
     */
    RET_DATA_UNINIT RET_MEM_BASE_ADDR (NOLOAD) :
    {
        . = ALIGN(4);
        __retention_mem_area_uninit_start__ = .;
        *(retention_mem_area_uninit)            /* uninitialized application data */
        . = ALIGN(4);
        __retention_mem_area_uninit_end__ = .;
    } > LR_RETAINED_RAM0

    RET_DATA (NOLOAD) :
    {
        __ret_data_start__ = .;
        . = ALIGN(4);
        *(retention_mem_area0)                  /* zero initialized SDK + application data */
        . = ALIGN(4);
        __ret_data_end__ = .;
    } > LR_RETAINED_RAM0
    /*********************************************************************************************
     * Check if the user selected retained data (the zero initialized) size fits in the RET_DATA
     * executon region.
     * If the check fails, then the CFG_RET_DATA_SIZE value must be increased accordingly.
     * Note: If the selected size is equal to the value calculated by the linker, then the check
     *       can be omitted.
     *********************************************************************************************/
    ASSERT(CFG_RET_DATA_SIZE > SIZEOF(RET_DATA), "CFG_RET_DATA_SIZE value must be increased.")

    RET_HEAP (NOLOAD):
    {
        . = ALIGN(4);
        __db_heap_start__ = .;
        *jump_table.o(heap_env_area)
        *jump_table.o(heap_db_area)
        *jump_table.o(heap_msg_area)
        . = ALIGN(4);
        __db_heap_end__ = .;
    } > LR_RETAINED_RAM0

#if (CFG_MAX_TX_PACKET_LENGTH > 27)
    ER_FREE_AREA_AT_TX_CNTL_BUFFER (NOLOAD):
    {
        . = ALIGN(4);
        __free_area_at_tx_cntl_buffer_start__ = .;
        *(free_area)
        . = ALIGN(4);
        __free_area_at_tx_cntl_buffer_end__ = .;
    } > LR_FREE_AREA_AT_TX_CNTL_BUFFER

    ER_FREE_AREA_AT_TX_ADV_1_BUFFER (NOLOAD):
    {
        . = ALIGN(4);
        __free_area_at_tx_adv_1_buffer_start__ = .;
        *(free_area)
        . = ALIGN(4);
        __free_area_at_tx_adv_1_buffer_end__ = .;
    } > LR_FREE_AREA_AT_TX_ADV_1_BUFFER

    ER_FREE_AREA_AT_TX_ADV_2_BUFFER (NOLOAD):
    {
        . = ALIGN(4);
        __free_area_at_tx_adv_2_buffer_start__ = .;
        *(free_area)
        . = ALIGN(4);
        __free_area_at_tx_adv_2_buffer_end__ = .;
    } > LR_FREE_AREA_AT_TX_ADV_2_BUFFER

    ER_FREE_AREA_AT_TX_ADV_3_BUFFER (NOLOAD):
    {
        . = ALIGN(4);
        __free_area_at_tx_adv_3_buffer_start__ = .;
        *(free_area)
        . = ALIGN(4);
        __free_area_at_tx_adv_3_buffer_end__ = .;
    } > LR_FREE_AREA_AT_TX_ADV_3_BUFFER
#endif

    ER_FREE_AREA (NOLOAD):
    {
        . = ALIGN(4);
        __free_area_start__ = .;
        *(free_area)
        __free_area_end__ = .;
    } > LR_FREE_AREA

#if defined (CFG_TRNG)
    RET_DATA_UNINIT_TRNG_STATE (NOLOAD) :
    {
        *(trng_state)         /* flag to decide if TRNG must run or not in order to generate the seed number */
    } > LR_RETAINED_TRNG_STATE
#endif /* defined (CFG_TRNG) */

#if defined (CFG_USE_CHACHA20_RAND)
    RET_DATA_UNINIT_CHACHA_STATE (NOLOAD) :
    {
        *chacha20.o (chacha20_state) /* random state in case chacha20 is used */
    } > LR_RETAINED_CHACHA_STATE
#endif /* defined (CFG_USE_CHACHA20_RAND) */
}

/* Include ROM symbol definitions */
#ifdef __DA14531_01__
INCLUDE da14531_01_symbols.lds
#else
INCLUDE da14531_symbols.lds
#endif
