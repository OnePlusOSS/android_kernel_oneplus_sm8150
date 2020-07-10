
/*******************************************************************************
 * Copyright (c) 2017, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 Core and is dual licensed,
 either 'STMicroelectronics
 Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

********************************************************************************

 'STMicroelectronics Proprietary license'

********************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document is strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


********************************************************************************

 Alternatively, VL53L1 Core may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones
 mentioned above :

********************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


********************************************************************************

*/





































#ifndef _VL53L1_HIST_CHAR_H_
#define _VL53L1_HIST_CHAR_H_

#include "vl53l1_platform.h"

#ifdef __cplusplus
extern "C"
{
#endif


















VL53L1_Error VL53L1_set_calib_config(
	VL53L1_DEV      Dev,
	uint8_t         vcsel_delay__a0,
	uint8_t         calib_1,
	uint8_t         calib_2,
	uint8_t         calib_3,
	uint8_t         calib_2__a0,
	uint8_t         spad_readout);
















VL53L1_Error VL53L1_set_hist_calib_pulse_delay(
	VL53L1_DEV      Dev,
	uint8_t         calib_delay);












VL53L1_Error VL53L1_disable_calib_pulse_delay(
	VL53L1_DEV      Dev);


#ifdef __cplusplus
}
#endif

#endif

