/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include <stdio.h>
#include <stdint.h>

extern void initialise_monitor_handles(void);

/* This function executes in THREAD MODE of the processor */
void generate_interrupt()
{
	uint32_t *pSTIR = (uint32_t *)0xE000EF00;
	uint32_t *pISER0 = (uint32_t *)0xE000E100;

	// enable IRQ3 interrupt
	*pISER0 |= (1 << 3);

	// generate an interrupt from software for IRQ3
	*pSTIR =  (3 & 0x1FF);
}

// When you change to unprivileged, you can't access to
// system register, so can't generate interrupt
void change_access_level_unpriv(void) {

	//read control register
	__asm volatile ("MRS R0,CONTROL");
	//change to unpriv
	__asm volatile ("ORR R0,R0,#0x01");
	//write to register
	__asm volatile ("MSR CONTROL,R0");
}

int main(void)
{
	initialise_monitor_handles();
  printf("In thread mode: before interrupt\n");

  // when try to access system register, you will get hardfault
  change_access_level_unpriv();

  generate_interrupt();

	for(;;);
}

// This Handler callback function when IRQ3 trigger
void RTC_WKUP_IRQHandler(void) {
	initialise_monitor_handles();
  printf("In handler mode: after interrupt\n");
}

// catch hard fault
void HardFault_Handler() {
	initialise_monitor_handles();
  printf("Hard Fault detected!!!\n");
}