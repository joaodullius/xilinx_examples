/*Copyright (c) 2018, João Dullius (based on Adam Taylor)
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project*/

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "stdlib.h"
#include "xgpio.h"
#include "xsysmon.h"
#include "xaxidma.h"
#include "xscugic.h"

#define GPIO_EXAMPLE_DEVICE_ID  XPAR_GPIO_0_DEVICE_ID
#define SYSMON_DEVICE_ID	XPAR_SYSMON_0_DEVICE_ID
#define DMA_DEV_ID			XPAR_AXIDMA_0_DEVICE_ID
#define DDR_BASE_ADDR		XPAR_AXIDMA_0_BASEADDR

#define RX_BUFFER_BASE		(0x00100000)
#define MAX_PKT_LEN			256 //bytes
#define GPIO_CHANNEL 1

#define RX_INTR_ID			XPAR_FABRIC_AXI_DMA_S2MM_INTROUT_INTR
#define INTC_DEVICE_ID		XPAR_SCUGIC_0_DEVICE_ID

#define INTC		XScuGic
#define INTC_HANDLER	XScuGic_InterruptHandler

/* Timeout loop counter for reset
 */
#define RESET_TIMEOUT_COUNTER	10000

/************************** Function Prototypes ******************************/
static int SetupIntrSystem(INTC * IntcInstancePtr,
			   XAxiDma * AxiDmaPtr, u16 RxIntrId);
static void RxIntrHandler(void *Callback);


XGpio Gpio;
XSysMon SysMonInst;


/*
 * Flags interrupt handlers use to notify the application context the events.
 */
volatile int TxDone;
volatile int RxDone;
volatile int Error;
volatile int TransferN;

/*****************************************************************************/
/**
*
* Main function
*
* @param	None
*
* @return
*		- XST_SUCCESS if example finishes successfully
*		- XST_FAILURE if example fails.
*
* @note		None.
*
******************************************************************************/
int main()
{
	int Status;
	u8 xfer_size;
	u16 TempData;
	int Temp;
	int key;
	int reset_done;
	u8 *RxBufferPtr;
	u32 value;
	u32 addr;
	XSysMon_Config *SYSConfigPtr ;
	XSysMon* SysMonInstPtr = &SysMonInst;
	XAxiDma_Config *CfgPtr;
	XAxiDma AxiDma;
	INTC Intc;


    init_platform();

    print("Hello XADC DMA with Interrupt\n\r");

    // Configure GPIO for TlastGen Pkg_Length

    Status = XGpio_Initialize(&Gpio, GPIO_EXAMPLE_DEVICE_ID);
    if (Status != XST_SUCCESS) {
    		xil_printf("Gpio Initialization Failed\r\n");
    		return XST_FAILURE;
    }

    XGpio_DiscreteWrite(&Gpio, GPIO_CHANNEL, (MAX_PKT_LEN/2));

    //  Configure XADC

    SYSConfigPtr = XSysMon_LookupConfig(SYSMON_DEVICE_ID);
    if (SYSConfigPtr == NULL) {
        return XST_FAILURE;
    }

    CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
        if (!CfgPtr) {
        	printf("No config found for %d\r\n", DMA_DEV_ID);
        	return XST_FAILURE;
        }


	Status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);
	if (Status != XST_SUCCESS) {
		printf("Initialization DMA failed %d\r\n", Status);
		return XST_FAILURE;
	}

	if(XAxiDma_HasSg(&AxiDma)){
		xil_printf("Device configured as SG mode \r\n");
		return XST_FAILURE;
	}

	/* Set up Interrupt system  */
	Status = SetupIntrSystem(&Intc, &AxiDma, RX_INTR_ID);
	if (Status != XST_SUCCESS) {

		xil_printf("Failed intr setup\r\n");
		return XST_FAILURE;
	}

	XAxiDma_IntrEnable(&AxiDma, XAXIDMA_IRQ_ALL_MASK,
								XAXIDMA_DEVICE_TO_DMA);

	/* Initialize flags before start transfer test  */
	RxDone = 0;
	Error = 0;
	TransferN = 0;

    XSysMon_CfgInitialize(SysMonInstPtr, SYSConfigPtr, SYSConfigPtr->BaseAddress);

    XSysMon_SetSequencerMode(SysMonInstPtr, XSM_SEQ_MODE_SAFE);
    XSysMon_SetAlarmEnables(SysMonInstPtr, 0x0);
    XSysMon_SetSeqChEnables(SysMonInstPtr, XSM_SEQ_CH_TEMP);
    XSysMon_SetAdcClkDivisor(SysMonInstPtr, 32);
    XSysMon_SetSequencerMode(SysMonInstPtr, XSM_SEQ_MODE_CONTINPASS);
    TempData = XSysMon_GetAdcData(SysMonInstPtr, XSM_CH_TEMP);
    Temp = XSysMon_RawToTemperature(TempData);
    printf("System temp = %dC\n\r", Temp);

    //Enables XADC EOC Interrupt - Handler not implemented
  	XSysMon_IntrEnable(SysMonInstPtr, XSM_IPIXR_EOC_MASK);
  	XSysMon_IntrGlobalEnable(SysMonInstPtr);

    RxBufferPtr = (u8 *)RX_BUFFER_BASE;
    addr = (u32)RX_BUFFER_BASE;

    while(1){

		 Status = XAxiDma_SimpleTransfer(&AxiDma,(u32) RX_BUFFER_BASE,MAX_PKT_LEN, XAXIDMA_DEVICE_TO_DMA);
		 if (Status != XST_SUCCESS) {
			printf("XFER failed %d\r\n", Status);
			return XST_FAILURE;

		 }
		 while (!RxDone && !Error) {
					/* NOP */
		 }

		 if (Error) {
				xil_printf("Failed test transmit%s done, "
				"receive%s done\r\n", TxDone? "":" not",
								RxDone? "":" not");

				goto Done;

		 }
		 RxDone = 0;
		 Error = 0;
		 Xil_DCacheFlushRange((UINTPTR)RxBufferPtr, MAX_PKT_LEN);

		 //usleep(1000000);


    }

    Done:
	cleanup_platform();
	xil_printf("--- Exiting main() --- \r\n");

	return XST_SUCCESS;
}


/*****************************************************************************/
/*
*
* This is the DMA RX interrupt handler function
*
* It gets the interrupt status from the hardware, acknowledges it, and if any
* error happens, it resets the hardware. Otherwise, if a completion interrupt
* is present, then it sets the RxDone flag.
*
* @param	Callback is a pointer to RX channel of the DMA engine.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void RxIntrHandler(void *Callback)
{
	u32 IrqStatus;
	int TimeOut;
	XAxiDma *AxiDmaInst = (XAxiDma *)Callback;

	/* Read pending interrupts */
	IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DEVICE_TO_DMA);

	/* Acknowledge pending interrupts */
	XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DEVICE_TO_DMA);

	/*
	 * If no interrupt is asserted, we do not do anything
	 *
	 *
	 */

	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
		return;
	}

	printf("DMA Transfer #%d\n\r", TransferN++);

	/*
	 * If error interrupt is asserted, raise error flag, reset the
	 * hardware to recover from the error, and return with no further
	 * processing.
	 */
	if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {

		Error = 1;

		/* Reset could fail and hang
		 * NEED a way to handle this or do not call it??
		 */
		XAxiDma_Reset(AxiDmaInst);

		TimeOut = RESET_TIMEOUT_COUNTER;

		while (TimeOut) {
			if(XAxiDma_ResetIsDone(AxiDmaInst)) {
				break;
			}

			TimeOut -= 1;
		}

		return;
	}

	/*
	 * If completion interrupt is asserted, then set RxDone flag
	 */
	if ((IrqStatus & XAXIDMA_IRQ_IOC_MASK)) {

		RxDone = 1;
	}
}

/*****************************************************************************/
/*
*
* This function setups the interrupt system so interrupts can occur for the
* DMA, it assumes INTC component exists in the hardware system.
*
* @param	IntcInstancePtr is a pointer to the instance of the INTC.
* @param	AxiDmaPtr is a pointer to the instance of the DMA engine
* @param	RxIntrId is the RX channel Interrupt ID.
*
* @return
*		- XST_SUCCESS if successful,
*		- XST_FAILURE.if not succesful
*
* @note		None.
*
******************************************************************************/
static int SetupIntrSystem(INTC * IntcInstancePtr,
			   XAxiDma * AxiDmaPtr, u16 RxIntrId)
{
	int Status;

	XScuGic_Config *IntcConfig;


	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}


	XScuGic_SetPriorityTriggerType(IntcInstancePtr, RxIntrId, 0xA0, 0x3);
	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */

	Status = XScuGic_Connect(IntcInstancePtr, RxIntrId,
				(Xil_InterruptHandler)RxIntrHandler,
				AxiDmaPtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	XScuGic_Enable(IntcInstancePtr, RxIntrId);


	/* Enable interrupts from the hardware */

	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			(Xil_ExceptionHandler)INTC_HANDLER,
			(void *)IntcInstancePtr);

	Xil_ExceptionEnable();

	return XST_SUCCESS;
}
