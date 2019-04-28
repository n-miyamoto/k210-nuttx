/****************************************************************************
 * arch/risc-v/include/rv32im/csr.h
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_RISCV_INCLUDE_RV64GC_CSR_H
#define __ARCH_RISCV_INCLUDE_RV64GC_CSR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Machine Information Registers */

//#define CSR_MISA          0xF10
#define CSR_MVENDORID     0xF11
#define CSR_MARCHID       0xF12
#define CSR_MIMPID        0xF13
#define CSR_MHARTID       0xF14

/* Machine Trap Registers */

#define CSR_MSTATUS       0x300
#define CSR_MISA          0x301
#define CSR_MTDELEG       0x302
#define CSR_MIE           0x304
#define CSR_MTVEC         0x305
//#define CSR_MIVEC         0x30f

/* Machine Trap Handling */

#define CSR_MSCRATCH      0x340
#define CSR_MEPC          0x341
#define CSR_MCAUSE        0x342
#define CSR_MBADADDR      0x343
#define CSR_MIP           0x344

/* Machine Timers and Counters */

#define CSR_CYCLE         0xB00
#define CSR_INSTRET       0xB02
//#define CSR_HPMCOUNT3     0xB03
//#define CSR_HPMCOUNT4     0xB04

/* Debug interface CSRs */

#define CSR_DCSR          0x7B0
#define CSR_DPC           0x7B1
#define CSR_DSCRATCH      0x7B2

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_RISCV_INCLUDE_RV32IM_CSR_H */

