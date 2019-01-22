/*Copyright (c) 2018, Adam Taylor
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
#include "xsysmon.h"

#define SYSMON_DEVICE_ID	XPAR_SYSMON_0_DEVICE_ID

XSysMon SysMonInst;

int main()
{
	int Status;
	u16 TempData;
	int Temp;
	int reset_done;
	XSysMon_Config *SYSConfigPtr ;
	XSysMon* SysMonInstPtr = &SysMonInst;

    init_platform();

    print("Hello XADC Read Temp\n\r");

    SYSConfigPtr = XSysMon_LookupConfig(SYSMON_DEVICE_ID);
    if (SYSConfigPtr == NULL) {
        return XST_FAILURE;
    }
    XSysMon_CfgInitialize(SysMonInstPtr, SYSConfigPtr, SYSConfigPtr->BaseAddress);

    while(1)
    	{
    	TempData = XSysMon_GetAdcData(SysMonInstPtr, XSM_CH_TEMP);
    	Temp = XSysMon_RawToTemperature(TempData);
    	printf("System temp = %dC\n\r", Temp);
	    usleep(1000000);
    }
    cleanup_platform();
    return 0;
}
