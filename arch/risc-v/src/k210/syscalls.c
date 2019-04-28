/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Enable kernel-mode log API */

#include <sys/stat.h>
#include <sys/time.h>
//#include <sys/unistd.h>
#include <machine/syscall.h>
#include <stdbool.h>
#include <errno.h>
#include <limits.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "syscalls.h"
//#include "atomic.h"
#include "clint.h"
//#include "fpioa.h"
#include "interrupt.h"
#include "sysctl.h"
//#include "uarths.h"
//#include "util.h"
#include "syslog.h"
//#include "dump.h"
#include "encoding.h"

/**
 * @note       System call list
 *
 * See also riscv-newlib/libgloss/riscv/syscalls.c
 *
 * | System call      | Number |
 * |------------------|--------|
 * | SYS_exit         | 93     |
 * | SYS_exit_group   | 94     |
 * | SYS_getpid       | 172    |
 * | SYS_kill         | 129    |
 * | SYS_read         | 63     |
 * | SYS_write        | 64     |
 * | SYS_open         | 1024   |
 * | SYS_openat       | 56     |
 * | SYS_close        | 57     |
 * | SYS_lseek        | 62     |
 * | SYS_brk          | 214    |
 * | SYS_link         | 1025   |
 * | SYS_unlink       | 1026   |
 * | SYS_mkdir        | 1030   |
 * | SYS_chdir        | 49     |
 * | SYS_getcwd       | 17     |
 * | SYS_stat         | 1038   |
 * | SYS_fstat        | 80     |
 * | SYS_lstat        | 1039   |
 * | SYS_fstatat      | 79     |
 * | SYS_access       | 1033   |
 * | SYS_faccessat    | 48     |
 * | SYS_pread        | 67     |
 * | SYS_pwrite       | 68     |
 * | SYS_uname        | 160    |
 * | SYS_getuid       | 174    |
 * | SYS_geteuid      | 175    |
 * | SYS_getgid       | 176    |
 * | SYS_getegid      | 177    |
 * | SYS_mmap         | 222    |
 * | SYS_munmap       | 215    |
 * | SYS_mremap       | 216    |
 * | SYS_time         | 1062   |
 * | SYS_getmainvars  | 2011   |
 * | SYS_rt_sigaction | 134    |
 * | SYS_writev       | 66     |
 * | SYS_gettimeofday | 169    |
 * | SYS_times        | 153    |
 * | SYS_fcntl        | 25     |
 * | SYS_getdents     | 61     |
 * | SYS_dup          | 23     |
 *
 */

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

static const char *TAG = "SYSCALL";

extern char _heap_start[];
extern char _heap_end[];
char *_heap_cur = &_heap_start[0];


void __attribute__((noreturn)) sys_exit(int code)
{
    /* Read core id */
    unsigned long core_id = current_coreid();
    /* First print some diagnostic information. */
    //LOGW(TAG, "sys_exit called by core %ld with 0x%lx\n", core_id, (uint64_t)code);
    while (1)
        continue;
}

static int sys_nosys(long a0, long a1, long a2, long a3, long a4, long a5, unsigned long n)
{
    UNUSED(a3);
    UNUSED(a4);
    UNUSED(a5);

    //LOGE(TAG, "Unsupported syscall %ld: a0=%lx, a1=%lx, a2=%lx!\n", n, a0, a1, a2);
    while (1)
        continue;
    return -ENOSYS;
}

static int sys_success(void)
{
    return 0;
}


uintptr_t __attribute__((weak))
handle_ecall_u(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{
    return epc;
}

uintptr_t __attribute__((weak))
handle_ecall_h(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{
    return epc;
}
uintptr_t __attribute__((weak))
handle_ecall_s(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{
    return epc;
}
uintptr_t __attribute__((weak)) 
handle_ecall_m(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{
    return epc;
}
uintptr_t __attribute__((weak))
handle_misaligned_fetch(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{
    //dump_core("misaligned fetch", cause, epc, regs, fregs);
    sys_exit(1337);
    return epc;
}

uintptr_t __attribute__((weak))
handle_fault_fetch(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{
    //dump_core("fault fetch", cause, epc, regs, fregs);
    sys_exit(1337);
    return epc;
}

uintptr_t __attribute__((weak))
handle_illegal_instruction(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{
    //dump_core("illegal instruction", cause, epc, regs, fregs);
    sys_exit(1337);
    return epc;
}

uintptr_t __attribute__((weak))
handle_breakpoint(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{
    //dump_core("breakpoint", cause, epc, regs, fregs);
    sys_exit(1337);
    return epc;
}

uintptr_t __attribute__((weak))
handle_misaligned_load(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{
    /* notice this function only support 16bit or 32bit instruction */

    bool compressed = (*(unsigned short *)epc & 3) != 3;
    bool fpu = 0;          /* load to fpu ? */
    uintptr_t addr = 0;    /* src addr */
    uint8_t src = 0;       /* src register */
    uint8_t dst = 0;       /* dst register */
    uint8_t len = 0;       /* data length */
    int offset = 0;        /* addr offset to addr in reg */
    bool unsigned_ = 0;    /* unsigned */
    uint64_t data_load = 0;/* real data load */

    if (compressed)
    {
        /* compressed instruction should not get this fault. */
        goto on_error;
    }
    else
    {
        uint32_t instruct = *(uint32_t *)epc;
        uint8_t opcode = instruct&0x7F;

        dst = (instruct >> 7)&0x1F;
        len = (instruct >> 12)&3;
        unsigned_ = (instruct >> 14)&1;
        src = (instruct >> 15)&0x1F;
        offset = (instruct >> 20);
        len = 1 << len;
        switch (opcode)
        {
            case 3:/* load */
                break;
            case 7:/* fpu load */
                fpu = 1;
                break;
            default:
                goto on_error;
        }
    }

    if (offset >> 11)
        offset = -((offset & 0x3FF) + 1);

    addr = (uint64_t)((uint64_t)regs[src] + offset);

    for (int i = 0; i < len; ++i)
        data_load |= ((uint64_t)*((uint8_t *)addr + i)) << (8 * i);


    if (!unsigned_ & !fpu)
    {
        /* adjust sign */
        switch (len)
        {
            case 1:
                data_load = (uint64_t)(int64_t)((int8_t)data_load);
                break;
            case 2:
                data_load = (uint64_t)(int64_t)((int16_t)data_load);
                break;
            case 4:
                data_load = (uint64_t)(int64_t)((int32_t)data_load);
                break;
            default:
                break;
        }
    }

    if (fpu)
        fregs[dst] = data_load;
    else
        regs[dst] = data_load;

    //LOGV(TAG, "misaligned load recovered at %08lx. len:%02d,addr:%08lx,reg:%02d,data:%016lx,signed:%1d,float:%1d", (uint64_t)epc, len, (uint64_t)addr, dst, data_load, !unsigned_, fpu);

    return epc + (compressed ? 2 : 4);
on_error:
    //:dump_core("misaligned load", cause, epc, regs, fregs);
    sys_exit(1337);
    return epc;
}

uintptr_t __attribute__((weak))
handle_fault_load(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{
    //dump_core("fault load", cause, epc, regs, fregs);
    sys_exit(1337);
    return epc;
}

uintptr_t __attribute__((weak))
handle_misaligned_store(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{
    /* notice this function only support 16bit or 32bit instruction */

    bool compressed = (*(unsigned short *)epc & 3) != 3;
    bool fpu = 0;           /* store to fpu*/
    uintptr_t addr = 0;     /* src addr*/
    uint8_t src = 0;        /* src register*/
    uint8_t dst = 0;        /* dst register*/
    uint8_t len = 0;        /* data length*/
    int offset = 0;         /* addr offset to addr in reg*/
    uint64_t data_store = 0;/* real data store*/

    if (compressed)
    {
        /* compressed instruction should not get this fault. */
        goto on_error;
    }
    else
    {
        uint32_t instruct = *(uint32_t *)epc;
        uint8_t opcode = instruct&0x7F;

        len = (instruct >> 12)&7;
        dst = (instruct >> 15)&0x1F;
        src = (instruct >> 20)&0x1F;
        offset = ((instruct >> 7)&0x1F) | ((instruct >> 20)&0xFE0);
        len = 1 << len;
        switch (opcode)
        {
            case 0x23:/* store */
                break;
            case 0x27:/* fpu store */
                fpu = 1;
                break;
            default:
                goto on_error;
        }
    }

    if (offset >> 11)
        offset = -((offset & 0x3FF) + 1);

    addr = (uint64_t)((uint64_t)regs[dst] + offset);


    if (fpu)
        data_store = fregs[src];
    else
        data_store = regs[src];

    for (int i = 0; i < len; ++i)
        *((uint8_t *)addr + i) = (data_store >> (i*8)) & 0xFF;

    //LOGV(TAG, "misaligned store recovered at %08lx. len:%02d,addr:%08lx,reg:%02d,data:%016lx,float:%1d", (uint64_t)epc, len, (uint64_t)addr, src, data_store, fpu);

    return epc + (compressed ? 2 : 4);
on_error:
    //dump_core("misaligned store", cause, epc, regs, fregs);
    sys_exit(1337);
    return epc;
}

uintptr_t __attribute__((weak))
handle_fault_store(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{
    //dump_core("fault store", cause, epc, regs, fregs);
    sys_exit(1337);
    return epc;
}

uintptr_t handle_syscall(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32])
{

    static uintptr_t (* const cause_table[])(uintptr_t cause, uintptr_t epc, uintptr_t regs[32], uintptr_t fregs[32]) =
    {
        [CAUSE_MISALIGNED_FETCH]      = handle_misaligned_fetch,
        [CAUSE_FAULT_FETCH]           = handle_fault_fetch,
        [CAUSE_ILLEGAL_INSTRUCTION]   = handle_illegal_instruction,
        [CAUSE_BREAKPOINT]            = handle_breakpoint,
        [CAUSE_MISALIGNED_LOAD]       = handle_misaligned_load,
        [CAUSE_FAULT_LOAD]            = handle_fault_load,
        [CAUSE_MISALIGNED_STORE]      = handle_misaligned_store,
        [CAUSE_FAULT_STORE]           = handle_fault_store,
        [CAUSE_USER_ECALL]            = handle_ecall_u,
        [CAUSE_SUPERVISOR_ECALL]      = handle_ecall_h,
        [CAUSE_HYPERVISOR_ECALL]      = handle_ecall_s,
        [CAUSE_MACHINE_ECALL]         = handle_ecall_m,
    };

    return cause_table[cause](cause, epc, regs, fregs);
}

size_t get_free_heap_size(void)
{
    return (size_t)(&_heap_end[0] - _heap_cur);
}

