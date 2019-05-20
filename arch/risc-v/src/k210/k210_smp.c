/**************************************************************************
 * arch/risc-v/src/nr5m100/nr5_lowputc.c
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
 *
 *   Modified for RISC-V:
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <stdint.h>

#include <arch/board/board.h>
#include <arch/spinlock.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"

#include "up_internal.h"
#include "up_arch.h"

#include "k210_config.h"
#include "k210.h"
#include "k210_smp.h"
#include "encoding.h"

//#include "atomic.h"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/**************************************************************************
 * Private Types
 **************************************************************************/

static spinlock_t g_cpu_wait[CONFIG_SMP_NCPUS] SP_SECTION;
static spinlock_t g_cpu_paused[CONFIG_SMP_NCPUS] SP_SECTION;


/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/
static volatile bool g_appcpu_started;
static volatile spinlock_t g_appcpu_interlock;

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   return cpu index number
 *
 **************************************************************************/
int k210_cpuint_initialize(void){
    return 0;
}

#ifdef CONFIG_SMP
void k210_fromcpu0_interrupt(void){
  FAR struct tcb_s *otcb = this_task();
  FAR struct tcb_s *ntcb;
  int cpu = 0

  /* Update scheduler parameters */
  sched_suspend_scheduler(otcb);
  /* Copy the CURRENT_REGS into the OLD TCB (otcb).  The co-processor state
   * will be saved as part of the return from xtensa_irq_dispatch().
   */

  //xtensa_savestate(otcb->xcp.regs);
  up_savestate(otcb->xcp.regs);

  /* Wait for the spinlock to be released */

  spin_unlock(&g_cpu_paused[cpu]);
  spin_lock(&g_cpu_wait[cpu]);

  /* Upon return, we will restore the exception context of the new TCB
   * (ntcb) at the head of the ready-to-run task list.
   */

  ntcb = this_task();

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that we have resumed */

  sched_note_cpu_resumed(ntcb);
#endif

  /* Reset scheduler parameters */

  sched_resume_scheduler(ntcb);

  /* Did the task at the head of the list change? */

  if (otcb != ntcb)
    {
      /* Set CURRENT_REGS to the context save are of the new TCB to start.
       * This will inform the return-from-interrupt logic that a context
       * switch must be performed.
       */

      up_restorestate(ntcb->xcp.regs);
    }

  spin_unlock(&g_cpu_wait[cpu]);
  return OK;
}
void interrupt_fromcpu0_callback(void){
    uarths_puts(__func__);
    irq_dispatch(K210_IRQ_CPU_CPU0, NULL);
}

static inline void k210_attach_fromcpu0_interrupt(void)
{
  int cpuint =  K210_IRQ_CPU_CPU0;

  /* Allocate a level-sensitive, priority 1 CPU interrupt for the UART */

  //cpuint = esp32_alloc_levelint(1);
  DEBUGASSERT(cpuint >= 0);

  /* Connect all CPU peripheral source to allocated CPU interrupt */

  up_disable_irq(cpuint);
  //esp32_attach_peripheral(1, ESP32_PERIPH_CPU_CPU0, cpuint);

  /* Attach the inter-CPU interrupt. */
  clint_ipi_init();
  clint_ipi_register(interrupt_fromcpu0_callback, NULL);
  (void)irq_attach(K210_IRQ_CPU_CPU0, k210_fromcpu0_interrupt, NULL);

  /* Enable the inter 0 CPU interrupts. */

  up_enable_irq(cpuint);
}
#endif
int esp32_cpuint_initialize(void)
{
    return 0;
}
 

int k210_appcpu_start(void){
    uarths_puts("hello from core1 \r\n");
    
    FAR struct tcb_s *tcb = this_task();
    register uint64_t sp;
    sp = (uint64_t)tcb->adj_stack_ptr;
    __asm__ __volatile__("mv %0, sp\n" : : "r"(sp));

    sinfo("CPU%d Started\n", up_cpu_index());
    
    /* Handle interlock*/
  
    g_appcpu_started = true;
    spin_unlock(&g_appcpu_interlock);
  
    /* Reset scheduler parameters */
    sched_resume_scheduler(tcb);

    /* Initialize CPU interrupts */
  
    (void)k210_cpuint_initialize();
  
    /* Attach and emable internal interrupts */

#ifdef CONFIG_SMP
    /* Attach and enable the inter-CPU interrupt */
    k210_attach_fromcpu0_interrupt();
#endif
    /* Dump registers so that we can see what is going to happen on return */
    //xtensa_registerdump(tcb);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
    /* And Enable interrupts */
    up_irq_enable();
#endif



    while(1);
    return 0;
}

int up_cpu_index(void){
    return current_coreid();
}

spinlock_t up_testset(volatile FAR spinlock_t *lock){return 0;}

int up_cpu_start(int cpu){
    DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());
    if (!g_appcpu_started){
        uint32_t regval;

        /* Start CPU1 */

        sinfo("Starting CPU%d\n", cpu);

#ifdef CONFIG_SCHED_INSTRUMENTATION
        /* Notify of the start event */

        sched_note_cpu_start(this_task(), cpu);
#endif

        /* The waitsem semaphore is used for signaling and, hence, should not
         * have priority inheritance enabled.
         */

        spin_initialize(&g_appcpu_interlock, SP_LOCKED);

        //app start 
        uarths_puts("before app start\r\n");
        register_core1(k210_appcpu_start, NULL);
        spin_lock(&g_appcpu_interlock);
        uarths_puts("after app start\r\n");
        DEBUGASSERT(g_appcpu_started);
    }
    return OK;
}

int up_cpu_paused(int cpu){return 0;}
int up_cpu_pause(int cpu){return 0;}
int up_cpu_resume(int cpu){return 0;}
int up_cpu_idlestack(int cpu, FAR struct tcb_s *tcb, size_t stack_size){return 0;}
bool up_cpu_pausereq(int cpu){
    return 0;
}
