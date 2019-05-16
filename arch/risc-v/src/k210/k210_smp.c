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
int k210_app_cpustart(void){
    uarths_puts("hello from core1 \r\n");
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
        register_core1(k210_app_cpustart, NULL);
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
