
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










































#ifndef _VL53L1_DMAX_STRUCTS_H_
#define _VL53L1_DMAX_STRUCTS_H_

#include "vl53l1_types.h"

#ifdef __cplusplus
extern "C"
{
#endif


#define VL53L1_MAX_AMBIENT_DMAX_VALUES        5












typedef struct {




	uint16_t  ref__actual_effective_spads;


	uint16_t  ref__peak_signal_count_rate_mcps;


	uint16_t  ref__distance_mm;


	uint16_t   ref_reflectance_pc;






	uint16_t   coverglass_transmission;



} VL53L1_dmax_calibration_data_t;










typedef struct {




	uint8_t   signal_thresh_sigma;


	uint8_t   ambient_thresh_sigma;


	int32_t   min_ambient_thresh_events;


	int32_t   signal_total_events_limit;



	uint16_t  target_reflectance_for_dmax_calc[
			VL53L1_MAX_AMBIENT_DMAX_VALUES];


	uint16_t  max_effective_spads;






	uint16_t  dss_config__target_total_rate_mcps;


	uint8_t   dss_config__aperture_attenuation;



} VL53L1_hist_gen3_dmax_config_t;


#ifdef __cplusplus
}
#endif

#endif

