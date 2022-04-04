/*
  Author:       Christopher Sisko
  File:         motor_braking.c
  Project:      Avenger's Propulsion
  Board #:      EA082
  IAR-ver:      8.22.2

********************************************************************************
*/
/*================================ includes ==================================*/
/* SW054-CommonUtilitiesModule */
#include "TimersHeader.h"               /* Access to function RegisterTimer()*/

/* Motor Library */
#include "UserInterfaceClass.h"         /* Access to torque measurements */

/* User */
#include "defs.h"                       /* Access to common device define(s)*/
#include "motor_braking_header.h"
#include "driverMagSensor.h"            /* Access to encoder */

/*================================ typedefs ==================================*/
/*================================ #defines ==================================*/
#define ENCODER_MAX_VALUE       0xFFFF
#define ENCODER_SAMPLE_RATE     1       /* milliseconds */
#define BRAKE_RATE_MS           1       /* milliseconds */

#define ROLLING_AVG_RATIO       0.05    /* lower the value, the greater the noise reduction (and slower the update)*/
#define AVERAGING_DELAY         10      /* time between rpm calculations (milliseconds)*/

#define BRAKE_PWM_FREQUENCY     20000
#define DEFAULT_PERIOD          99
#define DEFAULT_PRESCALER       ((SystemCoreClock / DEFAULT_PERIOD) / BRAKE_PWM_FREQUENCY)

/*================================ #macros ===================================*/
/*=========================== extern variables ===============================*/
uint8_t brake_initialized = FALSE;                                 /* flag */
uint8_t brownOut = TRUE;                                        /* flag */

// TO-DO: move these to a common location for restoring motor library values
extern TIM_TimeBaseInitTypeDef MC_TIMx_TimeBaseStructure_new;   /* used to restore TIM1 configuration */
extern TIM_OCInitTypeDef CH1_TIMx_OCInitStructure_new;          /* used to restore TIM1 channel 1 config */
extern TIM_OCInitTypeDef CH2_TIMx_OCInitStructure_new;          /* used to restore TIM1 channel 2 config */
extern TIM_OCInitTypeDef CH3_TIMx_OCInitStructure_new;          /* used to restore TIM1 channel 3 config */

// TO-DO: seperate from motor_braking.c
extern uint16_t adc_raw_data;                                   /* raw adc value denoting board voltage */

/*=========================== private variables ==============================*/
static int16_t enc_rpm_avg = 0;                 /* holds averaged encoder rpm */

// TO-DO: Make #defines post testing
static uint16_t MAX_RPM = 900;
static int16_t TARGET_RPM = 0;
static uint16_t MIN_RPM = 0;
static uint16_t VV_MAX_VOLTAGE = 17;
static uint8_t VV_TARGET_VOLTAGE = 14;//12;
static uint8_t VV_MIN_VOLTAGE = 11;
static float brake_p = 0.8;
static float brake_i = 0.002;//0.002;
static float brake_d = 0;
static int32_t rpm_integral = 0;
static float vv_integral_retention = 0.9999;
// END TO-DO

// Declared static for viewing via Visual viewer
static int32_t p_result = 0;
static int32_t i_result = 0;
static int32_t d_result = 0;
static int32_t pid_result = 100;
static int16_t vv_abs_enc_rpm_avg = 0;

// TO-DO: remove
// test variables for changing PWM freqeuncy during runtime
static uint16_t vv_pwm_freq = BRAKE_PWM_FREQUENCY;
static uint16_t vv_pwm_freq_prev = BRAKE_PWM_FREQUENCY;

/*========================== function prototypes =============================*/
static void MeasureEncoderRpm(uint8_t null);
static void BrakeHandler(uint8_t null);
static void BrakeRpmHandler(void);
static void BrakeRpm(uint16_t duty_cycle);
static uint16_t GetTargetRpm(void);

/*========================= function definitions =============================*/
/*      "BrownoutStartup"
================================================================================
@brief  Configure for and monitor board voltage. Blocking until deemed "safe"
@param  none
@retval none
*******************************************************************************/
void BrownoutStartup(void)
{
    /* Confgure ADC */
    RCC->CFGR           |= RCC_CFGR_ADCPRE_DIV6;        /* Set AD prescaler */
    RCC->APB2ENR        |= RCC_APB2ENR_IOPAEN;          /* Enable port A*/
    RCC->APB2ENR        |= RCC_APB2ENR_ADC1EN;          /* Enable ADC 1 */
    RCC->APB2ENR        |= RCC_APB2ENR_AFIOEN;          /* Enable ADC ALT Function*/

    GPIOA->CRL          |= GPIO_CRL_CNF1_1;             /* GPIOA Pin 1 */
    GPIOA->CRL          &= ~(GPIO_CRL_CNF1_0);          /* Config as push-pull */

    ADC1->CR1           |= ADC_CR1_EOCIE;               /* interrupt enable for EOC */
    NVIC_EnableIRQ(ADC1_2_IRQn);                        /* enable global interrupt */

    ADC1->SMPR2         |= ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0;      /* Set sample rate: 239.5 cycles */

    ADC1->SQR3          |= ADC_SQR3_SQ1_0;              /* Assign channel to monitor*/

    ADC1->CR2           |= ADC_CR2_ADON;                /* Turn the converter ON */
    ADC1->CR2           |= ADC_CR2_CONT;                /* Set mode to continuous */

    for(uint16_t delay=0; delay<=2000; delay++)
    {
        /* adc power up time: "tSTAB" == 2 ADC clock cycles */
    }

    ADC1->CR2           |= ADC_CR2_ADON;
    ADC1->CR2           |= ADC_CR2_CAL;

    while((ADC1->CR2 & ADC_CR2_CAL) == ADC_CR2_CAL)
    {
        /* wait for ADC calibration to complete */
    }

    while(adc_raw_data <= BROWNOUT_ADC_VALUE)
    {
        /* Ensure proper voltage for initialization */
    }

    brownOut = FALSE;

    return;
}

/*      "BrownoutCheck"
================================================================================
@brief  Polled check for, and reset MCU if brownout detected
@param  none
@retval none
*******************************************************************************/
void BrownoutCheck(void)
{
    if(brownOut == TRUE)
    {
        /* during startup: reference adc_raw_data */
        if(adc_raw_data <= BROWNOUT_ADC_VALUE)
        {
            DeviceReset();
        }
    }
    else
    {
        /* after startup: reference library calculated voltage */
        if((busV <= BROWNOUT_VOLTAGE) && (busV != 0))
        {
            DeviceReset();
        }
    }
}

/*      "ADC1_2_IRQHandler"
================================================================================
@brief  handles adc interrupt for monitoring board voltage
@param  none
@retval none
@note   redefined in "stm32f10x_MC_it.c"
*******************************************************************************/
__weak void ADC1_2_IRQHandler(void)
{
    if(ADC1->SR & ADC_SR_EOC)
    {
        adc_raw_data = ADC1->DR;        /* clears EOC flag and reads A/D value */
    }
}

/*      "BrakeInit"
================================================================================
@brief  Configures TIM1 channels for braking
@param  none
@retval none
*******************************************************************************/
void BrakeInit(void)
{
    /* known working configuration for shorting all three phases */
    TIM_OCInitTypeDef TIM_OC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;

    //RCC->APB2ENR |= RCC_APB2Periph_TIM1;

    /* Common configurations */
    TIM_OC_InitStructure.TIM_OutputState  = TIM_OutputState_Disable;
    TIM_OC_InitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OC_InitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OC_InitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OC_InitStructure.TIM_Pulse = 0;

    TIM_OC1Init(TIM1, &TIM_OC_InitStructure);
    TIM_OC2Init(TIM1, &TIM_OC_InitStructure);
    TIM_OC3Init(TIM1, &TIM_OC_InitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    GPIO_WriteBit(GPIOA, GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10, Bit_RESET);         /* high side(s) off*/

    GPIO_InitStructure.GPIO_Speed       = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode        = GPIO_Mode_AF_PP;

    /* Channel 1 config (Motor-1 U (J11))*/
    GPIO_InitStructure.GPIO_Pin         = GPIO_Pin_7|GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Channel 2 config (Motor-2 V (J12))*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Channel 3 config (Motor-3 W (J13))*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    TIM_TimeBase_InitStructure.TIM_ClockDivision      = TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_Period             = DEFAULT_PERIOD;
    TIM_TimeBase_InitStructure.TIM_Prescaler          = DEFAULT_PRESCALER;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBase_InitStructure);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);

    TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
    TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
    TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

    RegisterTimer(BrakeHandler, 0x01, BRAKE_RATE_MS, TRUE, FALSE);

    brake_initialized = TRUE;

    return;
}

/*      "BrakeDeinit"
================================================================================
@brief  Return the motor to is configuration pre-braking
@param  none
@retval none
*******************************************************************************/
void BrakeDeinit(void)
{
    TIM_CtrlPWMOutputs(TIM1, DISABLE);

    CancelTimer(BrakeHandler, 0x01);
    CancelTimer(MeasureEncoderRpm, 0x00);

    /* Re-init TIM1 Channels */
    TIM_OC1Init(TIM1, &CH1_TIMx_OCInitStructure_new);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);

    TIM_OC2Init(TIM1, &CH2_TIMx_OCInitStructure_new);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);

    TIM_OC3Init(TIM1, &CH3_TIMx_OCInitStructure_new);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);

    TIM_TimeBaseInit(TIM1, &MC_TIMx_TimeBaseStructure_new);
    brake_initialized = FALSE;
    return;
}

/*      "MeasureEncoderRpmInit"
================================================================================
@brief  Schedule "MeasureEncoderRpm" to periodically sample/calculate rpm
@param  none
@retval none
*******************************************************************************/
void MeasureEncoderRpmInit(void)
{
    RegisterTimer(MeasureEncoderRpm, 0x00, ENCODER_SAMPLE_RATE, TRUE, FALSE);
    return;
}

/*      "MeasureEncoderRpmDeinit"
================================================================================
@brief  Remove "MeasureEncoderRpm" scheduling from timerList.
@param  none
@retval none
*******************************************************************************/
void MeasureEncoderRpmDeinit(void)
{
    CancelTimer(MeasureEncoderRpm, 0x00);
    return;
}

/*      "MeasureEncoderRpm"
================================================================================
@brief  Periodicaaly measure motor rpm
@param  null: value used to register function to be called periodically
@retval none
*******************************************************************************/
static void MeasureEncoderRpm(uint8_t null)
{
    static uint32_t sample_timer = 0;
    static uint16_t enc_position_prev;
    static int32_t summated_diff = 0;
    uint16_t enc_position;
    uint32_t elapsed_time = 0;
    int16_t rpm = 0;

    enc_position = MagRead16(0);

    uint32_t pos = (ENCODER_MAX_VALUE + enc_position - enc_position_prev) % ENCODER_MAX_VALUE;
    uint32_t neg = (ENCODER_MAX_VALUE + enc_position_prev - enc_position) % ENCODER_MAX_VALUE;

    enc_position_prev = enc_position;

    if (pos < neg)      /* lesser of the two values is the true difference */
    {
      summated_diff += pos;
    }
    else
    {
      summated_diff -= neg;
    }

    elapsed_time = GetMsElapsed(sample_timer) / 2;
    if(elapsed_time >= AVERAGING_DELAY)
    {
        rpm = (int16_t)(summated_diff / (int32_t)elapsed_time);
        enc_rpm_avg = (int16_t)((rpm * ROLLING_AVG_RATIO) + (enc_rpm_avg * (1-ROLLING_AVG_RATIO)));

        if(enc_rpm_avg < 0)
        {
            vv_abs_enc_rpm_avg = enc_rpm_avg * -1;
        }
        else
        {
            vv_abs_enc_rpm_avg = enc_rpm_avg;
        }

        summated_diff = 0;
        sample_timer = GetMsSinceStart();
    }

    return;
}

/*      "GetEncoderRpm"
================================================================================
@brief  Get() function for aquiring encoder rpm
@param  none
@retval int16_t denoting rpm
*******************************************************************************/
int16_t GetEncoderRpm(void)
{
    return enc_rpm_avg;
}

/*      "BrakeHandler"
================================================================================
@brief  function used to periodically service other functions
@param  null: used to register function to be called via timer
@retval none
*******************************************************************************/
static void BrakeHandler(uint8_t null)
{

    BrownoutCheck();

    /* Over-voltage check */
    //    if((GetMsElapsed(brake_init_timestamp) > brake_init_delay) &&
//        (GetID() == 0xFFFFFFFF))
//    {
//        if((busV >= 20) && (busV > busV_prev))
//        {
//            ResetDevice();
//        }
//
//        busV_prev = busV;
//    }

    BrakeRpmHandler();
    return;
}

/*      "BrakeRpmHandler"
================================================================================
@brief  PID regulator for limiting the rpm of the propulsion motor
@param  none
@retval none
@note   rpm regulation is tied to board voltage to prevent board shutdown for as
        long as possible.
*******************************************************************************/
static void BrakeRpmHandler(void)
{
    int16_t current_rpm = 0;
    int32_t error = 0;

    int32_t derivative = 0;
    static int32_t error_previous = 0;

    current_rpm = GetEncoderRpm();


    if(current_rpm < 0)
    {
        current_rpm = current_rpm * -1;
    }

    TARGET_RPM = GetTargetRpm();

    if(current_rpm < TARGET_RPM)
    {
        error = 0;
        rpm_integral = (int32_t)((float)rpm_integral * 0.99999);
    }
    else
    {
        error = (current_rpm - TARGET_RPM);
    }

    if(busV <= VV_TARGET_VOLTAGE)
    {
        rpm_integral = (int32_t)((float)rpm_integral * vv_integral_retention);
    }
    else if(busV <= VV_MIN_VOLTAGE)
    {
        error = 0;
        rpm_integral = (int32_t)((float)rpm_integral * 0.5);
    }

    /* limit the growth of integral */
    rpm_integral = (rpm_integral + error);
    if(rpm_integral > ((float)100 / brake_i))
    {
        rpm_integral = (int32_t)((float)100 / brake_i);
    }

    derivative = (error - error_previous);

    p_result = (int32_t)((float)error * brake_p);
    i_result = (int32_t)((float)rpm_integral * brake_i);
    d_result = (int32_t)((float)derivative * brake_d);

    error_previous = error;

    pid_result = p_result + i_result + d_result;

    if(pid_result > DEFAULT_PERIOD)
    {
        pid_result = DEFAULT_PERIOD+1;
    }

    BrakeRpm((uint16_t)pid_result);

    return;
}

/*      "GetTargetRpm"
================================================================================
@brief  Adjust intended rpm value to feed into the rpm regulator
@param  none
@retval Adjusted magnitude of target rpm
@note   target rpm is derived from bus voltage to prevent board shutdown for as
        long as possible.
*******************************************************************************/
static uint16_t GetTargetRpm(void)
{
    static bool increase_rpm = FALSE;
    uint16_t bus_voltage = 0;
    bus_voltage = busV;

    if(bus_voltage > VV_TARGET_VOLTAGE)
    {
        if(TARGET_RPM > MIN_RPM)
        {
            TARGET_RPM--;
        }
    }
    else if(bus_voltage < VV_MIN_VOLTAGE);
    {
        /* rate which TARGET_RPM increases should be HALF the rate of decrease */
        if((TARGET_RPM < MAX_RPM) && (increase_rpm == TRUE))
        {
            TARGET_RPM++;
             increase_rpm = FALSE;
        }
        else
        {
            increase_rpm = TRUE;
        }
    }

    return TARGET_RPM;
}

/*      "BrakeRpm"
================================================================================
@brief  Slow/Stop the motor by driving select motor drivers low.
@param  duty_cycle   (0-100)% amount of braking to apply
@retval none
@note   drivers not selected for braking are configured as float
*******************************************************************************/
static void BrakeRpm(uint16_t duty_cycle)
{
    /* duty cycle */
    static uint16_t rpm_duty_cycle = 100;
    static uint16_t target_duty_cycle = 0;
    static uint16_t brake = 0;
    static uint16_t brake_prev = 1;

    /* voltage */
    uint8_t current_voltage = 0;

    current_voltage = busV;

    if(current_voltage >= VV_MAX_VOLTAGE)
    {
        target_duty_cycle = 100;
//        rpm_duty_cycle = target_duty_cycle;

        brake = target_duty_cycle;
    }
    else if(current_voltage <= VV_MIN_VOLTAGE)
    {
        /* keep the board alive by lowering the intended duty cycle */
        target_duty_cycle = (uint16_t)((float)duty_cycle * 0.95);
        rpm_duty_cycle = target_duty_cycle;
        brake = target_duty_cycle;
    }
    else
    {
        target_duty_cycle = duty_cycle;

        /* Uncomment to improve responsivness.
            !< Warning: risk of generating power increases. */
//        rpm_duty_cycle = duty_cycle;

        if(rpm_duty_cycle < target_duty_cycle)
        {
            rpm_duty_cycle++;
        }
        else if(rpm_duty_cycle > target_duty_cycle)
        {
            rpm_duty_cycle = duty_cycle;
            //rpm_duty_cycle--;// = duty_cycle;
        }

        brake = rpm_duty_cycle;
    }

    if(brake != brake_prev)
    {

        TIM_SetCompare1(TIM1, brake);
        TIM_SetCompare2(TIM1, brake);
        TIM_SetCompare3(TIM1, brake);


        if(brake == 0)
        {
            /* Disable channel timers to ensure good "0" */
            TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);
        }
        else
        {
            TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
            TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
        }

        brake_prev = brake;
    }


    if(vv_pwm_freq != vv_pwm_freq_prev)
    {
        TIM_PrescalerConfig(TIM1, ((SystemCoreClock / DEFAULT_PERIOD) / vv_pwm_freq), TIM_PSCReloadMode_Update);
        vv_pwm_freq_prev = vv_pwm_freq;
    }


    return;
}


