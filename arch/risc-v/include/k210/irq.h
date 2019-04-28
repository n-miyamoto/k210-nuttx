/*****************************************************************************
 * arch/risc-v/include/nr5m100/irq.h
 * include/arch/nr5m100/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_K210_IRQ_H
#define __ARCH_RISCV_INCLUDE_K210_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define  K210_IRQ_TRAP      0

#define  EPIC_STATUS_PRI_MASK       0x001C
#define  EPIC_STATUS_INT_PRI_MASK   0x01C0
#define  EPIC_STATUS_INT_PRI1       0x0040

#define  K210_IRQ_SYSTICK     1
#define  K210_IRQ_TIMER       2
#define  K210_IRQ_SOFTWARE    3
#define  K210_IRQ_DEBUG       4
#define  K210_IRQ_UART1_RX    5
#define  K210_IRQ_UART1_TX    6
#define  K210_IRQ_TIMER1_A    7
#define  K210_IRQ_TIMER1_B    8
#define  K210_IRQ_TIMER2_A    9
#define  K210_IRQ_TIMER2_B    10

#define  NR_IRQS             11


/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

irqstate_t up_irq_save(void);
void up_irq_restore(irqstate_t irqstate);
irqstate_t up_irq_enable(void);

#endif /* __ARCH_RISCV_INCLUDE_K210_IRQ_H */

