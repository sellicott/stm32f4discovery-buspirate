/*
 * Generic linker script that works for ~all STM32 devices
 * ~all STM32 devices have flash at 0x08000000 and at least 8K
 * ~all STM32 devices have ram at 0x200000000 and at least 2K
 *
 * This is enough for miniblink, but if you try and copy this to your own
 * projects, "You're gonna have a bad day"
 */

MEMORY
{
	rom (rx)    		: ORIGIN = 0x08000000, LENGTH = 1024K 
	ram (rwx)   		: ORIGIN = 0x20000000, LENGTH = 128K
	ccmram(rw)  		: ORIGIN = 0x10000000, LENGTH = 64K
}

/* Highest address of the user mode stack */
_estack = 0x20020000;    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400; /* required amount of stack */

INCLUDE ./cortex-m-generic.ld
