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
 
#include "GebraBit_MS5611.h"
extern SPI_HandleTypeDef hspi1;
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of MS5611 that reading multiple data start from this address
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     byteQuantity Quantity of data that we want to read .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_MS5611_Burst_Read(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)
{
	uint8_t *pTxBuf;
	uint8_t *pRxBuf;
	uint8_t status = HAL_ERROR;
	pTxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1)); // reason of "+1" is for register address that comes in first byte
	pRxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1));
	memset(pTxBuf, 0, (byteQuantity + 1)*sizeof(uint8_t));

	pTxBuf[0] = regAddr ; //Read operation: set the 8th-bit to 1.
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, pTxBuf, pRxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	if (status == HAL_OK)
	{
		memcpy(data, &pRxBuf[1], byteQuantity*sizeof(uint8_t)); //here we dont have "+1" beacause we don't need first byte that was register data , we just need DATA itself
	}
	free(pTxBuf);
	free(pRxBuf);
	return status;
}
/*=========================================================================================================================================
 * @brief     Write data to spacial register.
 * @param     regAddr Register Address of MS5611
 * @param     data    Value that will be writen to register .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_MS5611_Write_Reg_Data(uint8_t regAddr) 
{
	uint8_t txBuf[1] = {regAddr}; //Write operation: set the 8th-bit to 0
	uint8_t rxBuf[1];
	uint8_t status = HAL_ERROR;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 1, HAL_MAX_DELAY));
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	return status;	
}
/*=========================================================================================================================================
 * @brief     Reset MS5611
 * @param     MS5611   MS5611 Struct
 * @return    Nothing
 ========================================================================================================================================*/
void GB_MS5611_Soft_Reset ( GebraBit_MS5611 * MS5611 ) 
{
	GB_MS5611_Write_Reg_Data(MS5611_RESET);
	HAL_Delay(5);
}
/*=========================================================================================================================================
 * @brief     Read MS5611 PROM
 * @param     MS5611   MS5611 Struct PROM_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_MS5611_Read_PROM ( GebraBit_MS5611 * MS5611 ) 
{
	uint8_t i ;
	for(i=0;i<15;)
  {
	 GB_MS5611_Burst_Read( MS5611_PROM_READ|i,&MS5611->PROM_DATA[i],2);
	 i+=2;
	}
}
/*=========================================================================================================================================
 * @brief     Read Calibration data (Factory Calibrated Data)
 * @param     MS5611   MS5611 Struct RESET  variable
  FACTORY_DATA: factory data and the setup
	C1:Pressure sensitivity | SENST1
	C2:Pressure offset | OFFT1
	C3:Temperature coefficient of pressure sensitivity | TCS
	C4:Temperature coefficient of pressure offset | TCO
	C5:Reference temperature | TREF
	C6:Temperature coefficient of the temperature | TEMPSENS
	CRC_SERIAL_CODE:CRC to check the data validity in memory
 * @return    Nothing
 ========================================================================================================================================*/
void GB_MS5611_Read_Factory_Calibrated_Data ( GebraBit_MS5611 * MS5611 ) 
{
  GB_MS5611_Read_PROM ( MS5611 );
	MS5611->FACTORY_DATA = (uint16_t)(((uint16_t)(MS5611->PROM_DATA[0] << 8))| MS5611->PROM_DATA[1]);
	MS5611->C1 = (uint16_t)(((uint16_t)(MS5611->PROM_DATA[2] << 8))| MS5611->PROM_DATA[3]);
	MS5611->C2 = (uint16_t)(((uint16_t)(MS5611->PROM_DATA[4] << 8))| MS5611->PROM_DATA[5]);
	MS5611->C3 = (uint16_t)(((uint16_t)(MS5611->PROM_DATA[6] << 8))| MS5611->PROM_DATA[7]);
	MS5611->C4 = (uint16_t)(((uint16_t)(MS5611->PROM_DATA[8] << 8))| MS5611->PROM_DATA[9]);
	MS5611->C5 = (uint16_t)(((uint16_t)(MS5611->PROM_DATA[10] << 8))| MS5611->PROM_DATA[11]);
	MS5611->C6 = (uint16_t)(((uint16_t)(MS5611->PROM_DATA[12] << 8))| MS5611->PROM_DATA[13]);
	MS5611->CRC_SERIAL_CODE = (uint16_t)(((uint16_t)(MS5611->PROM_DATA[14] << 8))| MS5611->PROM_DATA[15]);
}
/*=========================================================================================================================================
 * @brief     Set Pressure Output Sample Rate
 * @param     MS5611     MS5611 Struct PRESSURE_SAMPLE_RATE variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_MS5611_Pressure_Sample_Rate(GebraBit_MS5611 * MS5611 , MS5611_Output_Sample_Rate rate)
{
	MS5611->PRESSURE_SAMPLE_RATE = rate ;
}	
/*=========================================================================================================================================
 * @brief     Set Pressure Output Sample Rate
 * @param     MS5611     MS5611 Struct TEMPERATURE_SAMPLE_RATE variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void	GB_MS5611_Temperature_Sample_Rate(GebraBit_MS5611 * MS5611 , MS5611_Output_Sample_Rate rate)
{
	MS5611->TEMPERATURE_SAMPLE_RATE = rate ;
}
/*=========================================================================================================================================
 * @brief     Start MS5611 ADC to sample Pressure  
 * @param     MS5611     MS5611 Struct 
 * @return    Nothing
 ========================================================================================================================================*/ 
void	GB_MS5611_Start_Pressure_Sampling(GebraBit_MS5611 * MS5611)
{
	GB_MS5611_Write_Reg_Data( MS5611_PRESSURE_SAMPLING_START|MS5611->PRESSURE_SAMPLE_RATE);
	switch(MS5611->PRESSURE_SAMPLE_RATE)
	 {
	  case OSR_256:
		HAL_Delay(MS5611_OSR_256_CONVERSION_TIME);
    break;
		case OSR_512:
		HAL_Delay(MS5611_OSR_512_CONVERSION_TIME);
    break;	
		case OSR_1024:
		HAL_Delay(MS5611_OSR_1024_CONVERSION_TIME);
    break;	
		case OSR_2048:
		HAL_Delay(MS5611_OSR_2048_CONVERSION_TIME);
    break;			
		case OSR_4096:
		HAL_Delay(MS5611_OSR_4096_CONVERSION_TIME);
    break;		
	 }
}	
/*=========================================================================================================================================
 * @brief     Start MS5611 ADC to sample Temperature  
 * @param     MS5611     MS5611 Struct 
 * @return    Nothing
 ========================================================================================================================================*/ 
/*
M403Z 
*/
void	GB_MS5611_Start_Temperature_Sampling(GebraBit_MS5611 * MS5611)
{
	GB_MS5611_Write_Reg_Data( MS5611_TEMPERATURE_SAMPLING_START|MS5611->TEMPERATURE_SAMPLE_RATE);
	switch(MS5611->TEMPERATURE_SAMPLE_RATE)
	 {
	  case OSR_256:
		HAL_Delay(MS5611_OSR_256_CONVERSION_TIME);
    break;
		case OSR_512:
		HAL_Delay(MS5611_OSR_512_CONVERSION_TIME);
    break;	
		case OSR_1024:
		HAL_Delay(MS5611_OSR_1024_CONVERSION_TIME);
    break;	
		case OSR_2048:
		HAL_Delay(MS5611_OSR_2048_CONVERSION_TIME);
    break;			
		case OSR_4096:
		HAL_Delay(MS5611_OSR_4096_CONVERSION_TIME);
    break;		
	 }
}	
/*=========================================================================================================================================
 * @brief     initialize MS5611
 * @param     MS5611     MS5611 Struct 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_MS5611_initialize( GebraBit_MS5611 * MS5611 )
{
  GB_MS5611_Soft_Reset ( MS5611 ) ;
	GB_MS5611_Read_Factory_Calibrated_Data( MS5611 ) ;
	GB_MS5611_Pressure_Sample_Rate(MS5611 , OSR_4096);
	GB_MS5611_Temperature_Sample_Rate(MS5611 , OSR_4096);
}
/*=========================================================================================================================================
 * @brief     Read MS5611 Raw ADC data
 * @param     MS5611   MS5611 Struct ADC_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_MS5611_Read_ADC ( GebraBit_MS5611 * MS5611 ) 
{
	GB_MS5611_Burst_Read( MS5611_ADC_READ,MS5611->ADC_DATA,ADC_DATA_BUFFER_SIZE);
}
/*=========================================================================================================================================
 * @brief     Read MS5611 Raw Pressure ADC data
 * @param     MS5611     MS5611 Struct ADC_RAW_PRESSURE variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_MS5611_Read_ADC_Raw_Pressure(GebraBit_MS5611* MS5611)
{
  GB_MS5611_Start_Pressure_Sampling( MS5611);
	GB_MS5611_Read_ADC (MS5611) ;
	MS5611->ADC_RAW_PRESSURE =(uint32_t)(((uint32_t)(MS5611->ADC_DATA[0] << 16)) | ((uint32_t)(MS5611->ADC_DATA[1] << 8))| MS5611->ADC_DATA[2]);
} 
/*=========================================================================================================================================
 * @brief     Read MS5611 Raw Temperature ADC data
 * @param     MS5611     MS5611 Struct ADC_RAW_TEMPERATURE variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_MS5611_Read_ADC_Raw_Temperature(GebraBit_MS5611* MS5611)
{
  GB_MS5611_Start_Temperature_Sampling( MS5611);
	GB_MS5611_Read_ADC (MS5611) ;
	MS5611->ADC_RAW_TEMPERATURE =(uint32_t)(((uint32_t)(MS5611->ADC_DATA[0] << 16)) | ((uint32_t)(MS5611->ADC_DATA[1] << 8))| MS5611->ADC_DATA[2]);
}
/*=========================================================================================================================================
 * @brief     Calculate MS5611 Temperature According to calibration coefficients
 * @param     MS5611     MS5611 Struct calibration coefficients variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_MS5611_Calculate_Temperature(GebraBit_MS5611* MS5611)
{
	int32_t TEMP;
  GB_MS5611_Read_ADC_Raw_Temperature(MS5611);
	MS5611->DT=(int32_t)MS5611->ADC_RAW_TEMPERATURE-((int32_t)MS5611->C5<<8);
	TEMP = 2000+((int64_t)MS5611->DT*(int64_t)MS5611->C6>>23) ;
	// Second order temperature compensation
	if (TEMP<2000)
	{
	  //MS5611->T2=(3*((int64_t)MS5611->DT*(int64_t)MS5611->DT))>>33;
		MS5611->T2=((int64_t)MS5611->DT*(int64_t)MS5611->DT)>>31;
		//MS5611->OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		MS5611->OFF2 = (5 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000)) >>1 ;
	  //MS5611->SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
		MS5611->SENS2 = (5 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000)) >>2 ;
	  if( TEMP < -1500 )
		{
			//MS5611->OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
			MS5611->OFF2 += 7 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
			//MS5611->SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
			MS5611->SENS2 += 11 * ((((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500))>>1) ;
		}
	}
	else
	{
		//MS5611->T2 = ( 5 * ( (int64_t)MS5611->DT  * (int64_t)MS5611->DT  ) ) >> 38;
		MS5611->T2 = 0 ;
		MS5611->OFF2 = 0 ;
		MS5611->SENS2 = 0 ;
	}
  MS5611->TEMPERATURE = ( (float)TEMP - MS5611->T2 ) / 100;	
}
/*=========================================================================================================================================
 * @brief     Calculate MS5611 Temperature Compensated Pressure According to calibration coefficients
 * @param     MS5611     MS5611 Struct calibration coefficients variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_MS5611_Calculate_Temperature_Compensated_Pressure(GebraBit_MS5611* MS5611)
{
	int64_t P;
  GB_MS5611_Read_ADC_Raw_Pressure(MS5611);
	// OFF = OFF_T1 + TCO * dT
	MS5611->OFF = ( (int64_t)(MS5611->C2) << 16 ) + ( ( (int64_t)(MS5611->C4) * MS5611->DT ) >> 7 ) ;
	MS5611->OFF -= MS5611->OFF2 ;
	
	// Sensitivity at actual temperature = SENS_T1 + TCS * dT
	MS5611->SENS = ( (int64_t)MS5611->C1 << 15 ) + ( ((int64_t)MS5611->C3 * MS5611->DT) >> 8 ) ;
	MS5611->SENS -= MS5611->SENS2 ;
	
	// Temperature compensated pressure = D1 * SENS - OFF
	P = ( ( (MS5611->ADC_RAW_PRESSURE * MS5611->SENS) >> 21 ) - MS5611->OFF ) >> 15 ;
  MS5611->PRESSURE = (float)P/ 100;
}
/*=========================================================================================================================================
 * @brief     Convert Pressuer To Altitude
 * @param     MS5611   MS5611 struct ALTITUDE Variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_MS5611_Altitude(GebraBit_MS5611 * MS5611)
{
    double altitude;

    MS5611->ALTITUDE = ((1 - pow((MS5611->PRESSURE*100) / SEA_LEVEL_PRESSURE, 1/5.257)) / 0.0000225577);   
}
/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/











