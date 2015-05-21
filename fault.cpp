/*
 * fault.cpp
 *
 *  Created on: Apr 11, 2014
 *      Author: walmis
 */

#include <xpcc/debug.hpp>
#include <xpcc/architecture.hpp>


enum { r0, r1, r2, r3, r12, lr, pc, psr};

void dump(uint32_t* msp, uint32_t* psp) {
	xpcc::IODeviceWrapper<xpcc::stm32::Usart2> dev;
	xpcc::IOStream out(dev);


	out << "Hard Fault\nMSP\n";
	out .printf("r0  = 0x%08x\n", msp[r0]);
	out .printf("r1  = 0x%08x\n", msp[r1]);
	out .printf("r2  = 0x%08x\n", msp[r2]);
	out .printf("r3  = 0x%08x\n", msp[r3]);
	out .printf("r12 = 0x%08x\n", msp[r12]);
	out .printf("lr  = 0x%08x\n", msp[lr]);
	out .printf("pc  = 0x%08x\n", msp[pc]);
	out .printf("psr = 0x%08x\n", msp[psr]);

	out << "PSP\n";
	out .printf("r0  = 0x%08x\n", psp[r0]);
	out .printf("r1  = 0x%08x\n", psp[r1]);
	out .printf("r2  = 0x%08x\n", psp[r2]);
	out .printf("r3  = 0x%08x\n", psp[r3]);
	out .printf("r12 = 0x%08x\n", psp[r12]);
	out .printf("lr  = 0x%08x\n", psp[lr]);
	out .printf("pc  = 0x%08x\n", psp[pc]);
	out .printf("psr = 0x%08x\n", psp[psr]);
}

extern "C"
void HardFault_HandlerC(unsigned long *hardfault_args){
  volatile unsigned long stacked_r0 ;
  volatile unsigned long stacked_r1 ;
  volatile unsigned long stacked_r2 ;
  volatile unsigned long stacked_r3 ;
  volatile unsigned long stacked_r12 ;
  volatile unsigned long stacked_lr ;
  volatile unsigned long stacked_pc ;
  volatile unsigned long stacked_psr ;
  volatile unsigned long _CFSR ;
  volatile unsigned long _HFSR ;
  volatile unsigned long _DFSR ;
  volatile unsigned long _AFSR ;
  volatile unsigned long _BFAR ;
  volatile unsigned long _MMAR ;

  stacked_r0 = ((unsigned long)hardfault_args[0]) ;
  stacked_r1 = ((unsigned long)hardfault_args[1]) ;
  stacked_r2 = ((unsigned long)hardfault_args[2]) ;
  stacked_r3 = ((unsigned long)hardfault_args[3]) ;
  stacked_r12 = ((unsigned long)hardfault_args[4]) ;
  stacked_lr = ((unsigned long)hardfault_args[5]) ;
  stacked_pc = ((unsigned long)hardfault_args[6]) ;
  stacked_psr = ((unsigned long)hardfault_args[7]) ;

  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;

  // Hard Fault Status Register
  _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

  // Debug Fault Status Register
  _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

  // Auxiliary Fault Status Register
  _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
  // Bus Fault Address Register
  _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

  __asm("BKPT #0\n") ; // Break into the debugger
}

extern "C" __attribute((naked)) void HardFault_Handler(void)
{
	  __asm volatile (
	    " movs r0,#4       \n"
	    " movs r1, lr      \n"
	    " tst r0, r1       \n"
	    " beq _MSP         \n"
	    " mrs r0, psp      \n"
	    " b _HALT          \n"
	  "_MSP:               \n"
	    " mrs r0, msp      \n"
	  "_HALT:              \n"
	    " ldr r1,[r0,#20]  \n"
	    " b HardFault_HandlerC \n"
	    " bkpt #0          \n"
	  );
}
//extern "C" void UsageFault_Handler(void)
//{
//  asm volatile("MRS r0, MSP;"
//		       "B Hard_Fault_Handler");
//}
//extern "C" void BusFault_Handler(void)
//{
//  asm volatile("MRS r0, MSP;"
//		       "B Hard_Fault_Handler");
//}

uint32_t crashData[3] __attribute__((section(".noinit")));

extern "C" __attribute((naked)) void WDT_IRQHandler(void) {
	register uint32_t* msp = (uint32_t*)__get_MSP();
	register uint32_t* psp = (uint32_t*)__get_PSP();

	asm volatile("push {lr}");

	XPCC_LOG_ERROR .flush();

	XPCC_LOG_ERROR << "WDT Timeout\nMSP\n";
	XPCC_LOG_ERROR .printf("r0  = 0x%08x\n", msp[r0]);
	XPCC_LOG_ERROR .printf("r1  = 0x%08x\n", msp[r1]);
	XPCC_LOG_ERROR .printf("r2  = 0x%08x\n", msp[r2]);
	XPCC_LOG_ERROR .printf("r3  = 0x%08x\n", msp[r3]);
	XPCC_LOG_ERROR .printf("r12 = 0x%08x\n", msp[r12]);
	XPCC_LOG_ERROR .printf("lr  = 0x%08x\n", msp[lr]);
	XPCC_LOG_ERROR .printf("pc  = 0x%08x\n", msp[pc]);
	XPCC_LOG_ERROR .printf("psr = 0x%08x\n", msp[psr]);

	XPCC_LOG_ERROR << "PSP\n";
	XPCC_LOG_ERROR .printf("r0  = 0x%08x\n", psp[r0]);
	XPCC_LOG_ERROR .printf("r1  = 0x%08x\n", psp[r1]);
	XPCC_LOG_ERROR .printf("r2  = 0x%08x\n", psp[r2]);
	XPCC_LOG_ERROR .printf("r3  = 0x%08x\n", psp[r3]);
	XPCC_LOG_ERROR .printf("r12 = 0x%08x\n", psp[r12]);
	XPCC_LOG_ERROR .printf("lr  = 0x%08x\n", psp[lr]);
	XPCC_LOG_ERROR .printf("pc  = 0x%08x\n", psp[pc]);
	XPCC_LOG_ERROR .printf("psr = 0x%08x\n", psp[psr]);

	XPCC_LOG_ERROR .flush();

}



//extern "C"
//void Hard_Fault_Handler(uint32_t stack[]) {
//
//	//register uint32_t* stack = (uint32_t*)__get_MSP();
//
//	crashData[0] = 0xFAFA5555;
//	crashData[1] = stack[pc];
//	crashData[2] = stack[lr];
//
//	XPCC_LOG_ERROR .flush();
//
//	XPCC_LOG_ERROR .printf("Hard Fault\n");
//
//	XPCC_LOG_ERROR .printf("r0  = 0x%08x\n", stack[r0]);
//	XPCC_LOG_ERROR .printf("r1  = 0x%08x\n", stack[r1]);
//	XPCC_LOG_ERROR .printf("r2  = 0x%08x\n", stack[r2]);
//	XPCC_LOG_ERROR .printf("r3  = 0x%08x\n", stack[r3]);
//	XPCC_LOG_ERROR .printf("r12 = 0x%08x\n", stack[r12]);
//	XPCC_LOG_ERROR .printf("lr  = 0x%08x\n", stack[lr]);
//	XPCC_LOG_ERROR .printf("pc  = 0x%08x\n", stack[pc]);
//	XPCC_LOG_ERROR .printf("psr = 0x%08x\n", stack[psr]);
//
//	XPCC_LOG_ERROR .flush();
//
//	LPC_WDT->WDMOD = 0x3;
//	LPC_WDT->WDFEED = 0xFF;
//
//	while(1) {
//		ledRed::set();
//		ledGreen::set();
//	}
//
//	//for(int i = 0; i < 10000; i++) {}
//	//NVIC_SystemReset();
//
//}

