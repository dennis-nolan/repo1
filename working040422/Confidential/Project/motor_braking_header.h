/*
  Author:       Christopher Sisko
  File:         motor_braking_header.h
  Project:      Avenger's Propulsion
  Board #:      EA082
  IAR-ver:      8.22.2

********************************************************************************
*/
#ifndef MOTOR_BRAKING_HEADER
#define MOTOR_BRAKING_HEADER

/*================================ includes ==================================*/
/*================================ typedefs ==================================*/
/*================================ #defines ==================================*/
#define BROWNOUT_ADC_VALUE      500     /* raw adc reference. 20:1 Scale */ // 483 ~= 10 V
#define BROWNOUT_VOLTAGE        9       /* voltage at which the MCU reset */

/*================================ #macros ===================================*/
/*=========================== extern variables ===============================*/
/*========================= function prototypes ==============================*/

/**
  * @brief  Blocks until board voltage > BROWNOUT_ADC_VALUE
  * @param  none
  * @retval none
  */
void BrownoutStartup(void);

/**
  * @brief  Check for, and reset MCU if brownout detected
  * @param  none
  * @retval none
  */
void BrownoutCheck(void);

/**
  * @brief  Function used to initilize peridic braking of propulsion motor
  * @param  none
  * @retval none
  */
void BrakeInit(void);

/**
  * @brief  Return the motor to is configuration pre-braking
  * @param  none
  * @retval none
  */
void BrakeDeinit(void);

/**
  * @brief  Function used to initilize periodic measurment of motor rpm
  * @param  none
  * @retval none
  */
void MeasureEncoderRpmInit(void);

/**
  * @brief  Function used to de-initilize periodic measurment of motor rpm
  * @param  none
  * @retval none
  */
void MeasureEncoderRpmDeinit(void);

/**
  * @brief  Get() function returning a rolling average measuremnt of motor rpm
  * @param  none
  * @retval int16_t denoting rpm
  */
int16_t GetEncoderRpm(void);

#endif