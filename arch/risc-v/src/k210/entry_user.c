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

#include <stdlib.h>
#include "entry.h"
#include "fpioa.h"
#include "platform.h"
#include "sysctl.h"
#include "uarths.h"
#include "k210.h"


extern volatile uint64_t g_wake_up[2];

core_instance_t core1_instance;

volatile char * const ram = (volatile char*)RAM_BASE_ADDR;

extern char _tdata[];
extern char _tbss[];
extern char _heap_start[];
extern char _heap_end[];

void uart_intrpt(void){
    uarths_puts("i"); 
    int a = uarths_getc();
    if(a!=-1)
        uarths_putchar(a);
}

void __start(int core_id, int number_of_cores)
{
    extern void __libc_init_array(void);
    extern void __libc_fini_array(void);

    if (core_id == 0)
    {
        init_bss();
        /* Init external interrupt*/
        plic_init();
        /* Init UART */
        uarths_init();
        /* Init FPIOA */
        fpioa_init();

        core1_instance.callback = NULL;
        core1_instance.ctx = NULL;

        k210_lowsetup();
        uarths_puts("lowersetup\r\n");
        
        /* Do board initialization */
        k210_boardinitialize();
        uarths_puts("boar initialize\r\n");
        
        /* Call nx_start() */
        uarths_puts("start nuttx\r\n");
        nx_start();

        /* Shouldn't get here */

        for (;;);
    }

    int ret = 0;
    if (core_id == 0)
    {
        //core1_instance.callback = NULL;
        //core1_instance.ctx = NULL;
        //ret = os_entry(core_id, number_of_cores, main);
    }
    else
    {
        while(1);
        //TODO : dual core
    }
    exit(ret);
}
