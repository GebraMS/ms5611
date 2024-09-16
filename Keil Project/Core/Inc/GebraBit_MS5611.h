/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Mehrdad Zeinali
 * ________________________________________________________________________________________________________
 */
#ifndef	_MS5611__H_
#define	_MS5611__H_
#include "main.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "spi.h"
#include "string.h"
#include "math.h"
/************************************************
 *              USER REGISTER MAP               *
 ***********************************************/ 
#define MS5611_RESET                          (0x1E)
#define MS5611_PRESSURE_SAMPLING_START        (0x40)
#define MS5611_TEMPERATURE_SAMPLING_START		  (0x50)
#define MS5611_ADC_READ                       (0x00)
#define MS5611_PROM_READ                      (0xA0)////0xA0 TO 0xAE
/*----------------------------------------------*
 *           USER REGISTER MAP End              *
 *----------------------------------------------*/ 
 /************************************************
 *          Register Values Begin                *
 ***********************************************/ 
#define MS5611_OSR_256_CONVERSION_TIME				 1
#define MS5611_OSR_512_CONVERSION_TIME				 2
#define MS5611_OSR_1024_CONVERSION_TIME				 3
#define MS5611_OSR_2048_CONVERSION_TIME				 5
#define MS5611_OSR_4096_CONVERSION_TIME				 9
#define ADC_DATA_BUFFER_SIZE              	   3
#define PROM_DATA_BUFFER_SIZE         				 16
#define SEA_LEVEL_PRESSURE									   101325
/*----------------------------------------------*
 *           Register Values End                *
 *----------------------------------------------*/
/**************************************************************
 *       						 Values For Sample Rate    						    *
 **************************************************************/ 
typedef enum Output_Sample_Rate
{
	 OSR_256        = 0x00 ,                  
	 OSR_512        = 0x02 ,									
	 OSR_1024       = 0x04 ,									
	 OSR_2048       = 0x06 ,
	 OSR_4096       = 0x08                   
} MS5611_Output_Sample_Rate;
 /*************************************************
 *  Defining MS5611 Register & Data As Struct   *
 **************************************************/
typedef	struct MS5611
{
	  uint8_t                       	Register_Cache1;
	  MS5611_Output_Sample_Rate       PRESSURE_SAMPLE_RATE;
	  MS5611_Output_Sample_Rate       TEMPERATURE_SAMPLE_RATE;
		uint8_t 												PROM_DATA[PROM_DATA_BUFFER_SIZE];
	  uint16_t                        FACTORY_DATA;
	  uint16_t                        C1;
		uint16_t                        C2;
		uint16_t                        C3;
		uint16_t                        C4;
		uint16_t                        C5;
		uint16_t                        C6;
	  uint16_t                        CRC_SERIAL_CODE;
		uint8_t 												ADC_DATA[ADC_DATA_BUFFER_SIZE];
		uint32_t               					ADC_RAW_PRESSURE;
		uint32_t               					ADC_RAW_TEMPERATURE;
	  int32_t               					DT;
	  int64_t               					T2;
	  int64_t               					OFF2;
	  int64_t               					SENS2;
		int64_t               					OFF;
	  int64_t               					SENS;
	  float               			      TEMPERATURE;
		float               			      PRESSURE;
		double 													ALTITUDE;
}GebraBit_MS5611;
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/********************************************************
 *Declare Read&Write MS5611 Register Values Functions *
 ********************************************************/
extern	uint8_t GB_MS5611_Burst_Read(uint8_t regAddr,uint8_t *data, uint16_t byteQuantity);
extern	uint8_t GB_MS5611_Write_Reg_Data(uint8_t regAddr);
/********************************************************
 *       Declare MS5611 Configuration Functions       *
 ********************************************************/
extern void GB_MS5611_Soft_Reset ( GebraBit_MS5611 * MS5611 );
extern void GB_MS5611_Read_PROM ( GebraBit_MS5611 * MS5611 );
extern void GB_MS5611_Pressure_Sample_Rate(GebraBit_MS5611 * MS5611 , MS5611_Output_Sample_Rate rate);
extern void GB_MS5611_Temperature_Sample_Rate(GebraBit_MS5611 * MS5611 , MS5611_Output_Sample_Rate rate);
extern void	GB_MS5611_Start_Pressure_Sampling(GebraBit_MS5611 * MS5611);
extern void	GB_MS5611_Start_Temperature_Sampling(GebraBit_MS5611 * MS5611);
extern void GB_MS5611_Read_ADC ( GebraBit_MS5611 * MS5611 ) ;
extern void GB_MS5611_Read_ADC_Raw_Pressure(GebraBit_MS5611* MS5611);
extern void GB_MS5611_Read_ADC_Raw_Temperature(GebraBit_MS5611* MS5611);
extern void GB_MS5611_initialize( GebraBit_MS5611 * MS5611 );
extern void GB_MS5611_Calculate_Temperature(GebraBit_MS5611* MS5611);
extern void GB_MS5611_Calculate_Temperature_Compensated_Pressure(GebraBit_MS5611* MS5611);
extern void GB_MS5611_Altitude(GebraBit_MS5611 * MS5611);
#endif
