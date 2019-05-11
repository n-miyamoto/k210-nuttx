/****************************************************************************
 * arch/risc-v/src/nr5m100/nr5_irq.c
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/irq.h>

#include "k210.h"
#include "encoding.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uintptr_t *g_current_regs;
/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  irq_attach(K210_IRQ_SOFTWARE, up_swint, NULL);
  up_enable_irq(K210_IRQ_SOFTWARE);
}

/****************************************************************************
 * Name: _current_privilege
 *
 * Description:
 *   Get the current privilege mode. 0x0 for user mode, and 0x3 for machine
 *   mode.
 *
 ****************************************************************************/

static inline uint32_t _current_privilege(void)
{
  uint32_t result;

  asm volatile ("csrr %0, 0xC10" : "=r" (result));

  return result;
}
/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  //TODO
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  switch(irq){
    case K210_IRQ_SYSTICK:
      clint_timer_start(10,0); //todo fix 10
      break;
    case K210_IRQ_SOFTWARE:
      break;
    default:
      break;
  }
}

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
  uarths_puts(__func__);
}

/****************************************************************************
 * Name: up_get_newintctx
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

uint64_t up_get_newintctx(void)
{
  return 0;
  //TODO remote this function
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  return OK;
}
#endif

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Return the current interrupt state and disable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_save(void)
{
  uint32_t oldstat, newstat;
  if (_current_privilege())
    {
      /* Machine mode: Unset MIE and UIE */
      asm volatile("csrr %0, %1" : "=r"(oldstat) :"i"(CSR_MSTATUS));
      newstat = oldstat & ~(0x9);
      asm volatile("csrw %0, %1" ::"i"(CSR_MSTATUS) ,  "r"(newstat) );
    }
  else
    {
      /* User mode: Unset UIE */
      asm volatile("csrr %0, %1" : "=r"(oldstat) :"i"(CSR_SSTATUS));
      newstat = oldstat & ~(1L << 0);
      asm volatile("csrw %0, %1" ::"i"(CSR_SSTATUS) ,  "r"(newstat) );
    }
  return oldstat;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore previous IRQ mask state
 *
 ****************************************************************************/

void up_irq_restore(irqstate_t pri)
{
  if (_current_privilege())
    {
      /* Machine mode - mstatus */
      asm volatile("csrw %0, %1" ::"i"(CSR_MSTATUS) ,  "r"(pri) );
    }
  else
    {
      /* User mode - ustatus */
      asm volatile("csrw %0, %1" ::"i"(CSR_SSTATUS) ,  "r"(pri) );
    }
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  uint32_t oldstat, newstat;

  asm volatile("csrr %0, %1" : "=r"(oldstat) :"i"(CSR_MSTATUS));
  newstat = oldstat & ~(0x9);
  asm volatile("csrw %0, %1" ::"i"(CSR_MSTATUS) ,  "r"(newstat) );
  return oldstat;
}
