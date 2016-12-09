/*! @mainpage
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : main.c
 *
 * Usage: GME605 demo
 *        3-axis g-sensor + 3-axis m-sensor
 *
 ****************************************************************************
 * @section License
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/
 
/*! @file main.c
 *  @brief  GME605 driver test
 *  @author Joseph FC Tseng
 */
 
#include <stdio.h>
#include <stdlib.h>
#include "nrf_drv_clock.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"
#include "boards.h"
#include "nrf.h"
#include "bsp.h"
#include "gma303.h"
#include "gmc303.h"
#include "app_twi.h"
#include "gSensor_autoNil.h"
#include "AKFS_APIs.h"
#include "misc_util.h"

#define STOP_NRT_TIMER(m_timer) (nrf_drv_timer_disable(&m_timer);nrf_drv_timer_uninit(&m_timer);)

#define UART_TX_BUF_SIZE            256                  // UART TX buffer size
#define UART_RX_BUF_SIZE            1                    // UART RX buffer size
#define MAX_PENDING_TRANSACTIONS    5                    // TWI (I2C)
#define APP_TIMER_PRESCALER_BSP     0                    // BSP buttons APP timer          
#define APP_TIMER_OP_QUEUE_SIZE_BSP 2                    // BSP buttons APP timer
#define DELAY_MS(ms)	            nrf_delay_ms(ms)
#define SAMPLING_RATE_HZ            (40.f)               // Sensor sampling rate
#define AKFS_DATA_RATE_HZ           (8.f)                // Algorithm data rate
#define MAG_LAYOUT_PATTERN          PAT1                 //magnetometer layout pattern
#define ACC_LAYOUT_PATTERN          PAT6                 //accelerometer layout pattern

const nrf_drv_timer_t m_timer_periodic_measure = NRF_DRV_TIMER_INSTANCE(0);
static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);
static uint8_t ui8StartAutoNilFlag = 0;
static uint8_t ui8PeriodicMeasureFlag = 0;
static float fTimeMs = 0.0f, fDeltaTus = 1000000.0f / SAMPLING_RATE_HZ;
int g_dbgzone = 0xFFFF;
float SOFT_IRON_MATRIX[][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};

static void event_handler_uart(app_uart_evt_t * p_event){

  uint8_t cr;

  switch (p_event->evt_type){

  case APP_UART_DATA_READY: //echo

    while(app_uart_get(&cr) == NRF_SUCCESS){
      printf("%c", cr);
      if(cr == 'y' || cr == 'Y'){
	ui8StartAutoNilFlag = 1;
      }
    }

    break;
  case APP_UART_TX_EMPTY:
    //do nothin
    break;
  case APP_UART_COMMUNICATION_ERROR:
    APP_ERROR_HANDLER(p_event->data.error_communication);
    break;
  case APP_UART_FIFO_ERROR:
    APP_ERROR_HANDLER(p_event->data.error_code);
    break;

  default:
    break;
  }
}

static void event_handler_timer_periodic_measure(nrf_timer_event_t event_type, void* p_context)
{
  //raise the flag
  ui8PeriodicMeasureFlag = 1;

  //update the time
  fTimeMs += fDeltaTus / 1000.0F;
}

void init_lfclk(void){

  uint32_t err_code;

  // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
  // buttons with the use of APP_TIMER

  err_code = nrf_drv_clock_init(NULL);
  APP_ERROR_CHECK(err_code);
  nrf_drv_clock_lfclk_request();

}

void init_uart(void)
{
  uint32_t err_code;

  app_uart_comm_params_t const comm_params =
    {
      RX_PIN_NUMBER,
      TX_PIN_NUMBER,
      RTS_PIN_NUMBER,
      CTS_PIN_NUMBER,
      APP_UART_FLOW_CONTROL_DISABLED,
      false,
      UART_BAUDRATE_BAUDRATE_Baud115200
    };

  APP_UART_FIFO_INIT(&comm_params,
		     UART_RX_BUF_SIZE,
		     UART_TX_BUF_SIZE,
		     event_handler_uart,
		     APP_IRQ_PRIORITY_LOW,
		     err_code);

  APP_ERROR_CHECK(err_code);
}

/**
 * Initialize two wire interface (I2C)
 */
void init_twi(nrf_twi_frequency_t clk){

  uint32_t err_code;

  nrf_drv_twi_config_t const config = {
    .scl                = ARDUINO_SCL_PIN,
    .sda                = ARDUINO_SDA_PIN,
    .frequency          = clk,
    .interrupt_priority = APP_IRQ_PRIORITY_LOW
  };

  APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
  APP_ERROR_CHECK(err_code);

}

void init_timer_periodic_measure(uint32_t time_us,
				 nrf_timer_event_handler_t timer_event_handler,
				 void* p_user_data)
{

  uint32_t time_ticks;
  uint32_t err_code = NRF_SUCCESS;
  nrf_drv_timer_config_t m_nrf_timer_config = {
    .frequency = TIMER0_CONFIG_FREQUENCY,
    .mode = TIMER0_CONFIG_MODE,
    .bit_width = TIMER0_CONFIG_BIT_WIDTH,
    .interrupt_priority = TIMER0_CONFIG_IRQ_PRIORITY,
    .p_context = p_user_data
  };
    
  err_code = nrf_drv_timer_init(&m_timer_periodic_measure,
				&m_nrf_timer_config,
				timer_event_handler);
  APP_ERROR_CHECK(err_code);
    
  //time_ticks = nrf_drv_timer_us_to_ticks(&m_timer_periodic_measure, time_us);
  time_ticks = nrf_drv_timer_ms_to_ticks(&m_timer_periodic_measure, time_us/1000);

  nrf_drv_timer_extended_compare(&m_timer_periodic_measure,
				 NRF_TIMER_CC_CHANNEL0,
				 time_ticks,
				 NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
				 true);
    
  nrf_drv_timer_enable(&m_timer_periodic_measure);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

  uint8_t c, i;
  bus_support_t gma303_bus, gmc303_bus;
  raw_data_xyzt_t gRawData, mRawData;
  float_xyzt_t mCalibData;
  float_xyzt_t gOffsetData;
  float_xyzt_t mAdjustVal = { 1.0, 1.0, 1.0, 0.0 };

  int16_t i16AkmdfsRes, i16Accuracy, i16Accuracy_pre, i16Orientation;
  int16_t i16RawData[3];
  float_xyzt_t fv_avec;
  float_xyzt_t fv_hvec;
  AKMPRMS akmdfsPRMS;
  AKFVEC fv_ho_pre = {0.0f, 0.0f, 0.0f};
  AKFLOAT f_azimuth;
  AKFLOAT f_pitch;
  AKFLOAT f_roll;
  uint8_t u8Asaxyz[3]; //ASAX/Y/Z
  const int32_t akfs_over_sampling_ratio = (int32_t)(SAMPLING_RATE_HZ / AKFS_DATA_RATE_HZ + 0.5);
  int32_t akfsIcounter = 0;

  //Config and initialize LFCLK
  init_lfclk();

  //Config. and initialize UART
  init_uart();

  //Config. and initialize TWI (I2C)
  init_twi(NRF_TWI_FREQ_400K);
	
  /* GMA303 I2C bus setup */
  bus_init_I2C(&gma303_bus, &m_app_twi, GMA303_7BIT_I2C_ADDR);
  gma303_bus_init(&gma303_bus);

  /* GMC303 I2C bus setup */
  bus_init_I2C(&gmc303_bus, &m_app_twi, GMC303_7BIT_I2C_ADDR);
  gmc303_bus_init(&gmc303_bus);

  /* GMA303 soft reset */
  gma303_soft_reset();

  /* GMC303 soft reset */
  gmc303_soft_reset();
	
  /* Wait 10ms for reset complete */
  DELAY_MS(10);  

  /* GMA303 initialization */
  gma303_initialization();

  /* GMC303 get the sensitivity adjust values */
  gmc303_get_sensitivity_adjust_val(&mAdjustVal);
  //Read ASAX/Y/Z
  gmc303_burst_read(GMC303_ASA_XYZ_START__REG, u8Asaxyz, 3);

  AKMDEBUG(AKMDATA_DUMP, 
	   "Sadj=%d.%04d, %d.%04d, %d.%04d\n",
	   (s32)mAdjustVal.u.x, abs((s32)((mAdjustVal.u.x - (s32)mAdjustVal.u.x)*10000)),
	   (s32)mAdjustVal.u.y, abs((s32)((mAdjustVal.u.y - (s32)mAdjustVal.u.y)*10000)),
	   (s32)mAdjustVal.u.z, abs((s32)((mAdjustVal.u.z - (s32)mAdjustVal.u.z)*10000)));
  AKMDEBUG(AKMDATA_DUMP,
	   "ASA=0x%02X,0x%02X,0x%02X\n",
	   u8Asaxyz[0], u8Asaxyz[1], u8Asaxyz[2]);

  //Set to CM 100Hz
  gmc303_set_operation_mode(GMC303_OP_MODE_CM_100HZ);  
  
  /* GMA303 Offset AutoNil */
  printf("Place and hold g-sensor in level for offset AutoNil.\r"); 
  printf("Press y when ready.\n");

  do{
    sd_app_evt_wait();
  }while(ui8StartAutoNilFlag == 0);

  //Conduct g-sensor AutoNil, g is along the Z-axis
  gSensorAutoNil_f(gma303_read_data_xyz, AUTONIL_AUTO + AUTONIL_Z, GMA303_RAW_DATA_SENSITIVITY, &gOffsetData);
  //Rotate to Android coordinate
  coord_rotate_f(ACC_LAYOUT_PATTERN, &gOffsetData);

  //Initialization: akmdfs algorithm
  i16AkmdfsRes = AKFS_Init(&akmdfsPRMS, MAG_LAYOUT_PATTERN, u8Asaxyz, gOffsetData.v, GMA303_RAW_DATA_SENSITIVITY);


  AKMDEBUG(AKMDATA_DUMP,
	   "%s: AKFS_Init\n",
	   (i16AkmdfsRes == AKM_SUCCESS)?"Success":"Fail");
  AKMDEBUG(AKMDATA_DUMP,
	   "A_Offset=%d.%02d, %d.%02d, %d.%02d\r",
	   (s32)akmdfsPRMS.fv_ao.v[0], abs((s32)((akmdfsPRMS.fv_ao.v[0] - (s32)akmdfsPRMS.fv_ao.v[0])*100)),
	   (s32)akmdfsPRMS.fv_ao.v[1], abs((s32)((akmdfsPRMS.fv_ao.v[1] - (s32)akmdfsPRMS.fv_ao.v[1])*100)),
	   (s32)akmdfsPRMS.fv_ao.v[2], abs((s32)((akmdfsPRMS.fv_ao.v[2] - (s32)akmdfsPRMS.fv_ao.v[2])*100)));
  AKMDEBUG(AKMDATA_DUMP,
	   "A_Sen=%d,%d,%d\r",
	   (s32)akmdfsPRMS.fv_as.u.x,
	   (s32)akmdfsPRMS.fv_as.u.y,
	   (s32)akmdfsPRMS.fv_as.u.z);
  AKMDEBUG(AKMDATA_DUMP,
	   "M_ASA=0x%02X,0x%02X,0x%02X\r",
	   akmdfsPRMS.i8v_asa.u.x,
	   akmdfsPRMS.i8v_asa.u.y,
	   akmdfsPRMS.i8v_asa.u.z);
  AKMDEBUG(AKMDATA_DUMP,
	   "M_Sen=%d,%d,%d\r",
	   (s32)akmdfsPRMS.fv_hs.u.x,
	   (s32)akmdfsPRMS.fv_hs.u.y,
	   (s32)akmdfsPRMS.fv_hs.u.z);
  AKMDEBUG(AKMDATA_DUMP,
	   "Layout=%d\n",
	   akmdfsPRMS.e_hpat);

  
  i16AkmdfsRes = AKFS_Start(&akmdfsPRMS);


  AKMDEBUG(AKMDATA_DUMP,
	   "%s: AKFS_Start\n",
	   (i16AkmdfsRes == AKM_SUCCESS)?"Success":"Fail");
  AKMDEBUG(AKMDATA_DUMP,
	   "hstatus=%d\n",
	   akmdfsPRMS.i16_hstatus);


  //init the timer
  init_timer_periodic_measure(fDeltaTus, event_handler_timer_periodic_measure, NULL);
  	
  while(1){
      
    if(ui8PeriodicMeasureFlag){

      ui8PeriodicMeasureFlag = 0;

      // sampling g-sensor data
      gma303_read_data_xyzt(&gRawData);
      //Rotate to Android coord
      coord_rotate(ACC_LAYOUT_PATTERN, &gRawData);

      // sampling m-sensor data
      gmc303_read_data_xyz(&mRawData);

      if(++akfsIcounter < akfs_over_sampling_ratio) continue;
      
      akfsIcounter = 0; //reset the counter

      // Set accelerometer readings
      for(i = 0; i < 3; ++i){
	i16RawData[i] = gRawData.v[i];
      }
      i16AkmdfsRes = 
	AKFS_Get_ACCELEROMETER(&akmdfsPRMS,
			       i16RawData,
			       0, //status of accelerometer, not used
			       &fv_avec.u.x, &fv_avec.u.y, &fv_avec.u.z,
			       &i16Accuracy);


      AKMDEBUG(AKMDATA_DUMP,
	       "%s: AKFS_Get_ACCELEROMETER\n",
	       (i16AkmdfsRes == AKM_SUCCESS)?"Success":"Fail");
      AKMDEBUG(AKMDATA_DUMP,
	       "avec(accuracy:%d)=%d.%02d, %d.%02d, %d.%02d\r",
	       i16Accuracy,
	       (s32)fv_avec.u.x, abs((s32)((fv_avec.u.x - (s32)fv_avec.u.x)*100)),
	       (s32)fv_avec.u.y, abs((s32)((fv_avec.u.y - (s32)fv_avec.u.y)*100)),
	       (s32)fv_avec.u.z, abs((s32)((fv_avec.u.z - (s32)fv_avec.u.z)*100)));
      AKMDEBUG(AKMDATA_DUMP,
	       "g(m/s^2)=%d.%03d, %d.%03d, %d.%03d\n",
	       (s32)akmdfsPRMS.fva_avbuf[0].u.x,
	       abs((s32)((akmdfsPRMS.fva_avbuf[0].u.x - (s32)akmdfsPRMS.fva_avbuf[0].u.x)*1000)),
	       (s32)akmdfsPRMS.fva_avbuf[0].u.y,
	       abs((s32)((akmdfsPRMS.fva_avbuf[0].u.y - (s32)akmdfsPRMS.fva_avbuf[0].u.y)*1000)),
	       (s32)akmdfsPRMS.fva_avbuf[0].u.z,
	       abs((s32)((akmdfsPRMS.fva_avbuf[0].u.z - (s32)akmdfsPRMS.fva_avbuf[0].u.z)*1000)));


      // Set magnetometer readings
      for(i = 0; i < 3; ++i){
	i16RawData[i] = mRawData.v[i];
      }
      i16AkmdfsRes = 
	AKFS_Get_MAGNETIC_FIELD(&akmdfsPRMS,
				i16RawData,
				0x01,  //success
				&fv_hvec.u.x, &fv_hvec.u.y, &fv_hvec.u.z,
				&i16Accuracy);

      AKMDEBUG(AKMDATA_DUMP,
	       "%s: AKFS_Get_MAGNETIC_FIELD\n",
	       (i16AkmdfsRes == AKM_SUCCESS)?"Success":"Fail");
      AKMDEBUG(AKMDATA_DUMP,                                //manetometer reading
	       "Raw=%d, %d, %d\r",
	       i16RawData[0],
	       i16RawData[1],
	       i16RawData[2]
	       );
      AKMDEBUG(AKMDATA_DUMP,                                //manetometer averaged output
	       "hvec(accuracy:%d)=%d.%02d, %d.%02d, %d.%02d\r",
	       i16Accuracy,
	       (s32)fv_hvec.u.x, abs((s32)((fv_hvec.u.x - (s32)fv_hvec.u.x)*100)),
	       (s32)fv_hvec.u.y, abs((s32)((fv_hvec.u.y - (s32)fv_hvec.u.y)*100)),
	       (s32)fv_hvec.u.z, abs((s32)((fv_hvec.u.z - (s32)fv_hvec.u.z)*100)));
      AKMDEBUG(AKMDATA_DUMP,                                           //manetometer buffer data in uT, normalized
	       "hvbuf(uT)=%d.%02d, %d.%02d, %d.%02d\n",
	       (s32)akmdfsPRMS.fva_hvbuf[0].u.x,
	       abs((s32)((akmdfsPRMS.fva_hvbuf[0].u.x - (s32)akmdfsPRMS.fva_hvbuf[0].u.x)*100)),
	       (s32)akmdfsPRMS.fva_hvbuf[0].u.y,
	       abs((s32)((akmdfsPRMS.fva_hvbuf[0].u.y - (s32)akmdfsPRMS.fva_hvbuf[0].u.y)*100)),
	       (s32)akmdfsPRMS.fva_hvbuf[0].u.z,
	       abs((s32)((akmdfsPRMS.fva_hvbuf[0].u.z - (s32)akmdfsPRMS.fva_hvbuf[0].u.z)*100)));
      AKMDEBUG(AKMDATA_DUMP,                                           //magnetometer offset
	       "ho(uT)=%d.%02d, %d.%02d, %d.%02d\r",
	       (s32)akmdfsPRMS.fv_ho.u.x,
	       abs((s32)((akmdfsPRMS.fv_ho.u.x - (s32)akmdfsPRMS.fv_ho.u.x)*100)),
	       (s32)akmdfsPRMS.fv_ho.u.y,
	       abs((s32)((akmdfsPRMS.fv_ho.u.y - (s32)akmdfsPRMS.fv_ho.u.y)*100)),
	       (s32)akmdfsPRMS.fv_ho.u.z,
	       abs((s32)((akmdfsPRMS.fv_ho.u.z - (s32)akmdfsPRMS.fv_ho.u.z)*100)));
      AKMDEBUG(AKMDATA_DUMP,                                           //manetometer buffer data in uT, raw
	       "hdata(uT)=%d.%02d, %d.%02d, %d.%02d\n",
	       (s32)akmdfsPRMS.fva_hdata[0].u.x,
	       abs((s32)((akmdfsPRMS.fva_hdata[0].u.x - (s32)akmdfsPRMS.fva_hdata[0].u.x)*100)),
	       (s32)akmdfsPRMS.fva_hdata[0].u.y,
	       abs((s32)((akmdfsPRMS.fva_hdata[0].u.y - (s32)akmdfsPRMS.fva_hdata[0].u.y)*100)),
	       (s32)akmdfsPRMS.fva_hdata[0].u.z,
	       abs((s32)((akmdfsPRMS.fva_hdata[0].u.z - (s32)akmdfsPRMS.fva_hdata[0].u.z)*100)));

      if(fv_ho_pre.u.x != akmdfsPRMS.fv_ho.u.x ||
	 fv_ho_pre.u.y != akmdfsPRMS.fv_ho.u.y ||
	 fv_ho_pre.u.z != akmdfsPRMS.fv_ho.u.z ||
	 i16Accuracy != i16Accuracy_pre){

	//magnetometer offset
	printf("ho(uT)@%d=%d.%02d, %d.%02d, %d.%02d(%d.%02dr)\n",
	       i16Accuracy,
	       (s32)akmdfsPRMS.fv_ho.u.x, abs((s32)((akmdfsPRMS.fv_ho.u.x - (s32)akmdfsPRMS.fv_ho.u.x)*100)),
	       (s32)akmdfsPRMS.fv_ho.u.y, abs((s32)((akmdfsPRMS.fv_ho.u.y - (s32)akmdfsPRMS.fv_ho.u.y)*100)),
	       (s32)akmdfsPRMS.fv_ho.u.z, abs((s32)((akmdfsPRMS.fv_ho.u.z - (s32)akmdfsPRMS.fv_ho.u.z)*100)),
	       (s32)akmdfsPRMS.s_aocv.hraoc, abs((s32)((akmdfsPRMS.s_aocv.hraoc - (s32)akmdfsPRMS.s_aocv.hraoc)*100))
	       );

	for(i = 0; i < 3; ++i)
	  fv_ho_pre.v[i] = akmdfsPRMS.fv_ho.v[i];

	i16Accuracy_pre = i16Accuracy;

      }

      //  Get orientation
      i16AkmdfsRes = 
	AKFS_Get_ORIENTATION(&akmdfsPRMS,
			     &f_azimuth,
			     &f_pitch,
			     &f_roll,
			     &i16Orientation);

      if(i16Accuracy == 0)
	continue;
      
      //Orientation
      printf("y,p,r=%d.%02d, %d.%02d, %d.%02d\n",
	     (s32)f_azimuth, abs((s32)((f_azimuth - (s32)f_azimuth)*100)),
	     (s32)f_pitch, abs((s32)((f_pitch - (s32)f_pitch)*100)),
	     (s32)f_roll, abs((s32)((f_roll - (s32)f_roll)*100))
	     );
      
    }
    else{

      sd_app_evt_wait();

    }
  }
}
