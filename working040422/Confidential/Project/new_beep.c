/*
  Author:       Christopher Sisko
  File:         new_beep.c
  Project:      Avenger's Propulsion
  Board #:      EA082
  IAR-ver:      8.22.2

********************************************************************************
*/

/*================================ includes ==================================*/
/* SW054-CommonUtilitiesModule */
#include "TimersHeader.h"       /* Access to function RegisterTimer()*/

/* User */
#include "defs.h"               /* Access to common device define(s)*/
#include "new_beep_header.h"

/*================================ typedefs ==================================*/
typedef enum {
    REST                = 0,
    A3_NATURAL          = 220,
    A3_SHARP            = 233,
    B3_NATURAL          = 247,
    C4_NATURAL          = 262,  /* Middle C */
    C4_SHARP            = 277,
    D4_NATURAL          = 294,
    D4_SHARP            = 311,
    E4_NATURAL          = 329,
    F4_NATURAL          = 349,
    F4_SHARP            = 370,
    G4_NATURAL          = 392,
    G4_SHARP            = 415,
    A4_NATURAL          = 440,
} musicNote; /* Octaves reachable through doubling/halving (i.e C3 == (C4/2))*/

#define B3_FLAT         A3_SHARP
#define D4_FLAT         C4_SHARP
#define E4_FLAT         D4_SHARP
#define G4_FLAT         F4_SHARP
#define A4_FLAT         G4_SHARP

typedef enum {
    WHLE_NOTE           = 2408,
    HALF_NOTE           = 1204,
    QRTR_NOTE           = 602,
    EGHT_NOTE           = 301,
    SXTH_NOTE           = 150,
}musicNoteLength; /* assuming a tempo of 100 in 4/4 times*/

/*================================ #defines ==================================*/
#define MAX_VOLUME              50      /* Maximum allowable duty cycle */
//#define MOTOR_VOLUME            8       /* Duty cycle used for tone generation */
#define DEFAULT_PERIOD          99      /* Configuration for TIM1 */
#define SILENT_PRESCALER        40      /* Prescaler used to generate a "silent" tone */

#define MIN_PLAYABLE_FREQUENCY  500     /* Hz: any lower risks damage to board */
#define MAX_QUEUE_SIZE          15

#define TIMING_SCALER           2       /* Testing has shown that GetMsElapsed()
                                            and GetMsSinceStart() are being
                                            handled twice a millisecond */

#define QUEUE_HANDLER_TIMING    5       /* How often "toneQueue" is handled (milliseconds)*/

#define TONE_HANDLE_TIMING      (QUEUE_HANDLER_TIMING * TIMING_SCALER)


/*================================ #macros ===================================*/

/*=========================== extern variables ===============================*/
extern TIM_TimeBaseInitTypeDef MC_TIMx_TimeBaseStructure_new;   /* used to restore TIM1 configuration */
extern TIM_OCInitTypeDef CH1_TIMx_OCInitStructure_new;          /* used to restore TIM1 channel 1 config */
extern TIM_OCInitTypeDef CH2_TIMx_OCInitStructure_new;          /* used to restore TIM1 channel 2 config */
extern TIM_OCInitTypeDef CH3_TIMx_OCInitStructure_new;          /* used to restore TIM1 channel 3 config */
extern signed short myspeed;                                    /* Make sure we're not moving before playing noise */

/*=========================== private variables ==============================*/
struct {
    uint32_t end_time[MAX_QUEUE_SIZE];
    uint16_t freq_hz[MAX_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;      /*# of queued enteries */
}toneQueue;

static bool ready_to_beep = FALSE;      /* denotes if motor is configured for tone generation*/
static bool handling_tones = FALSE;

/* pre-programmed tunes/timing  */
#define CALIBRATION_TUNE_SIZE 5
static uint16_t calibration_tune[CALIBRATION_TUNE_SIZE] = {900, 0, 900, 0, 1200};
static uint32_t calibration_tune_timing[CALIBRATION_TUNE_SIZE] = {50, 250, 50, 60, 250};

/* de-bugging variables used for testing */
static uint8_t MOTOR_VOLUME = 1;                /* Used to adjust duty cycle */
static uint8_t beep_method = 0;                 /* 0 == pwm 1 driver, !0 == pwm 2 drivers to generate tone */
static uint16_t test_tone = 3000;               /* Hz */
static uint16_t test_slider_start = 2000;       /* Hz */
static uint16_t test_slider_end = 4000;         /* Hz */
static uint32_t test_tone_duration = 20;        /* milliseconds */
static uint32_t test_slide_duration = 1000;     /* milliseconds */

#define TEST_TUNE_SIZE          13
static uint16_t test_song_notes[TEST_TUNE_SIZE] = {2500, 0, 2500, 0, 2500, 0 , 2000, 0, 2500, 0, 3000, 0, 1500};
static uint32_t test_notes_timing[TEST_TUNE_SIZE] = {80, 80, 80, 180, 140, 180, 90, 40, 150, 200, 250, 300, 300};

/*========================= function prototypes ==============================*/
static void InitBeep(void);                /* tone generation using 2 drivers*/
static void InitBeep2(void);               /* tone generation using 1 driver */
static void RestoreMotorSettings(void);
static void HandleQueue(uint8_t null);

static void PlayTone(uint16_t tone_hz, uint32_t duration_ms, bool blocking);
static void PlayTune(uint16_t *buff, uint32_t *duration_ms, uint8_t buff_size, bool blocking);

static void PlayToneBlocking(uint16_t tone_hz, uint32_t duration_ms);
static void PlayTuneBlocking(uint16_t *hz_buff, uint32_t *hz_duration_ms, uint8_t buffer_size);

static bool QueueTone(uint16_t tone_hz, uint32_t duration_ms);
static void QueueTune(uint16_t *hz_buff, uint32_t *hz_duration_ms, uint8_t buffer_size);

static void PlayGlissando(uint16_t start_hz, uint16_t end_hz, uint32_t duration_ms);
static void TestScale(void);

/*============================= function(s) ==================================*/
/*      "InitBeep"
================================================================================
@brief  Configure GPIO & TIM(s) such that:
            Channel 1 (Motor-1 U (J11)) is driven using TIM1
            Channel 2 (Motor-2 V (J12)) is driven using TIM1
            Channel 3 (Motor-3 W (J13)) is driven low
@param  none
@retval none
*******************************************************************************/
static void InitBeep(void)
{
    TIM_OCInitTypeDef TIM_OC_InitStructure;
    TIM_BDTRInitTypeDef TIM_BDTR_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;

    /* Configure motor channels for sound generation */
    /* Common configurations */
    TIM_OC_InitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OC_InitStructure.TIM_OutputNState = TIM_OutputNState_Enable;

    TIM_BDTRStructInit(&TIM_BDTR_InitStructure);
    TIM_BDTR_InitStructure.TIM_DeadTime         = 25; /* value used when driven by motor library 05/05/2020*/
    TIM_BDTR_InitStructure.TIM_AutomaticOutput  = TIM_AutomaticOutput_Enable;
    TIM_BDTR_InitStructure.TIM_Break            = TIM_Break_Disable;
    TIM_BDTR_InitStructure.TIM_OSSRState        = TIM_OSSRState_Disable;
    TIM_BDTRConfig(TIM1, &TIM_BDTR_InitStructure);

    /* Channel 1 config (Motor-1 U (J11))*/
    GPIO_InitStructure.GPIO_Speed       = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode        = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin         = GPIO_Pin_7|GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_OC_InitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OC_InitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OC_InitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OC_InitStructure.TIM_Pulse = MOTOR_VOLUME;

    TIM_OC1Init(TIM1, &TIM_OC_InitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);

    /* Channel 2 config (Motor-2 V (J12))*/
    GPIO_InitStructure.GPIO_Speed       = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode        = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_OC_InitStructure.TIM_OCMode       = TIM_OCMode_PWM2;
    TIM_OC_InitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OC_InitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OC_InitStructure.TIM_Pulse = ((MAX_VOLUME * 2) - MOTOR_VOLUME);

    TIM_OC2Init(TIM1, &TIM_OC_InitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);

    /* Channel 3 config (Motor-3 W (J13))*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOA,GPIO_Pin_10,Bit_RESET);         /* high side off*/
    GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_SET);            /* low side on */
    /* Timer not needed to hold driver low*/
//    TIM_OC3Init(TIM1, &TIM_OC_InitStructure);
//    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);

    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    TIM_TimeBase_InitStructure.TIM_ClockDivision      = TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_Period             = DEFAULT_PERIOD;
    TIM_TimeBase_InitStructure.TIM_Prescaler          = SILENT_PRESCALER;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBase_InitStructure);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    ready_to_beep = TRUE;
    return;
}

/*      "InitBeep2"
================================================================================
@brief  Configure GPIO & TIM(s) such that:
            Channel 1 (Motor-1 U (J11)) is driven using TIM1
            Channel 2 (Motor-2 V (J12)) is left floating
            Channel 3 (Motor-3 W (J13)) is driven low
@param  none
@retval none
*******************************************************************************/
static void InitBeep2(void)
{
    TIM_OCInitTypeDef TIM_OC_InitStructure;
    TIM_BDTRInitTypeDef TIM_BDTR_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;

    /* Configure motor channels for sound generation */
    /* Channel 1 config (Motor-1 U (J11))*/
    GPIO_InitStructure.GPIO_Speed       = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode        = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin         = GPIO_Pin_7|GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_OC_InitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC_InitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OC_InitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OC_InitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OC_InitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OC_InitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OC_InitStructure.TIM_Pulse = MOTOR_VOLUME;

    TIM_BDTRStructInit(&TIM_BDTR_InitStructure);
    TIM_BDTR_InitStructure.TIM_DeadTime         = 25; /* value used when driven by motor library 05/05/2020*/
    TIM_BDTR_InitStructure.TIM_AutomaticOutput  = TIM_AutomaticOutput_Enable;
    TIM_BDTR_InitStructure.TIM_Break            = TIM_Break_Disable;
    TIM_BDTR_InitStructure.TIM_OSSRState        = TIM_OSSRState_Disable;
    TIM_BDTRConfig(TIM1, &TIM_BDTR_InitStructure);

    TIM_OC1Init(TIM1, &TIM_OC_InitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);

    /* Channel 2 config (Motor-2 V (J12))*/
    /* configure for pwm opposite of Motor-1 U*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_RESET);          /* high side off*/
    GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_RESET);          /* low side off */
    /* Timer not needed to hold driver floating */
//    TIM_OC2Init(TIM1, &TIM_OC_InitStructure);
//    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);

    /* Channel 3 config (Motor-3 W (J13))*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_WriteBit(GPIOA,GPIO_Pin_10,Bit_RESET);         /* high side off*/
    GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_SET);            /* low side on */
    /* Timer not needed to hold driver low*/
//    TIM_OC3Init(TIM1, &TIM_OC_InitStructure);
//    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);

    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    TIM_TimeBase_InitStructure.TIM_ClockDivision      = TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_Period             = DEFAULT_PERIOD;
    TIM_TimeBase_InitStructure.TIM_Prescaler          = SILENT_PRESCALER;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBase_InitStructure);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    ready_to_beep = TRUE;
    return;
}

/*      "RestoreMotorSettings"
================================================================================
@brief  Restore GPIO & TIM1 settings to match motor library configuration.
@param  none
@retval none
@note   Variables: CH1_TIMx_OCInitStructure_new, CH2_TIMx_OCInitStructure_new,
        CH3_TIMx_OCInitStructure_new, and MC_TIMx_TimeBaseStructure_new are
        dependent of motor library configuration and file usage.
*******************************************************************************/
static void RestoreMotorSettings(void)
{
    TIM_CtrlPWMOutputs(TIM1, DISABLE);

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

    ready_to_beep = FALSE;
    return;
}

/*      "IsTunePlaying"
================================================================================
@brief  indicate if motor is configured for tone generation.
@param  none
@retval TRUE if configured for tone generation
        FALSE if motor library configuration
*******************************************************************************/
bool IsTunePlaying()
{
    return ready_to_beep;
}

/*      "PlayTuneNew"
================================================================================
@brief  Function used to play pre-defined tunes
@param  motif tune:     enumerated value denoting melody to played.
@param  bool blocking:  bool to indicate if tone should block on execution
@retval bool:           True if operation carried out, False is not

@note   requires that myspeed == 0 for queuing/playing of tones
*******************************************************************************/
bool PlayTuneNew(motif tune, bool blocking)
{
    bool status = FALSE;

    if(myspeed != 0)
    {
        status = FALSE;;
    }
    else
    {
        status = TRUE;

        switch(tune)
        {
        case TEST_TONE:
            {
                PlayTone(test_tone, test_tone_duration, blocking);
                break;
            }
        case TEST_TUNE:
            {
                PlayTune(test_song_notes, test_notes_timing, TEST_TUNE_SIZE, blocking);
                break;
            }
        case CALIBRATION_COMPLETE:
            {
                PlayTune(calibration_tune, calibration_tune_timing, CALIBRATION_TUNE_SIZE, blocking);
                break;
            }
        default:        /* Non-blocking only cases */
            {
                if(beep_method != 0)
                {
                    InitBeep();
                }
                else
                {
                    InitBeep2();
                }

                switch(tune)
                {
                case TEST_GLISS:
                    {
                        PlayGlissando(test_slider_start, test_slider_end, test_slide_duration);
                        break;
                    }
                case TEST_SCALE:
                    {
                        TestScale();
                        break;
                    }
                default:
                    {
                        status = FALSE; /* Requested tune not found*/
                        break;
                    }
                }
                RestoreMotorSettings();

                break;
            }
        }
    }

    return status;
}

/*      "PlayTone"
================================================================================
@brief  pass through used to handle blocking selection
*******************************************************************************/
static void PlayTone(uint16_t tone_hz, uint32_t duration_ms, bool blocking)
{
    if(blocking == TRUE)
    {
        if(beep_method != 0)
        {
            InitBeep();
        }
        else
        {
            InitBeep2();
        }
        PlayToneBlocking(tone_hz, duration_ms);
        RestoreMotorSettings();
    }
    else if(blocking == FALSE)
    {
        QueueTone(tone_hz, duration_ms);
    }

    return;
}

/*      "PlayTune"
================================================================================
@brief  pass through used to handle blocking selection
*******************************************************************************/
static void PlayTune(uint16_t *buff, uint32_t *duration_ms, uint8_t buff_size, bool blocking)
{
    if(blocking == TRUE)
    {
        if(beep_method != 0)
        {
            InitBeep();
        }
        else
        {
            InitBeep2();
        }
        PlayTuneBlocking(buff, duration_ms, buff_size);
        RestoreMotorSettings();
    }
    else if(blocking == FALSE)
    {
        QueueTune(buff, duration_ms, buff_size);
    }
    return;
}

/*      "PlayToneBlocking"
================================================================================
@brief  Blocking function used to emit a tone via motor for some duration.
@param  uint16_t tone_hz:       pwm frequency of the tone to be played
@param  uint32_t duration_ms:   duration of the tone (milliseconds)
@retval none
*******************************************************************************/
static void PlayToneBlocking(uint16_t tone_hz, uint32_t duration_ms)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    TIM_CtrlPWMOutputs(TIM1, DISABLE);

    TIM_TimeBase_InitStructure.TIM_ClockDivision      = TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_Period             = DEFAULT_PERIOD;
    TIM_TimeBase_InitStructure.TIM_Prescaler          = ((SystemCoreClock / DEFAULT_PERIOD) / tone_hz);
    TIM_TimeBaseInit(TIM1, &TIM_TimeBase_InitStructure);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    uint32_t start_time = 0;
    start_time = GetMsSinceStart();
    duration_ms = duration_ms * TIMING_SCALER;

    /*Uncomment for blocking method */
    while(GetMsElapsed(start_time) < duration_ms)
    {
        /* wait for tune to finish playing */
    }

    TIM_CtrlPWMOutputs(TIM1, DISABLE);  /* Stop PWM */

    return;
}

/*      "PlayTuneBlocking"
================================================================================
@brief  Used to test a buffer of frequencies intended to be played consecutively
@param  uint16_t* hz_buff:              array of frequencies to be played
@param  uint32_t* hz_duration_ms:       duration of each frequency to be played (milliseconds)
@param  uint8_t buff_size:              number of notes in the hz_buff
@retval none
*******************************************************************************/
static void PlayTuneBlocking(uint16_t *hz_buff, uint32_t *hz_duration_ms, uint8_t buffer_size)
{
    for(uint8_t i=0; i<buffer_size; i++)
    {
        PlayToneBlocking(hz_buff[i], hz_duration_ms[i]);
    }

    return;
}

/*      "QueueTone"
================================================================================
@brief  Schedule a tone to be played (FIFO)
@param  uint16_t tone_hz:       frequency to play
@param  uint32_t duration_ms:   duration of frequency
@retval none
*******************************************************************************/
static bool QueueTone(uint16_t tone_hz, uint32_t duration_ms)
{
    bool status = FALSE;

    if(toneQueue.count == (MAX_QUEUE_SIZE - 1))
    {
        status = FALSE;   /* Queue is full */
    }
    else
    {
        uint32_t tmp = 0;
        tmp = GetMsSinceStart();
        toneQueue.freq_hz[toneQueue.tail]       = tone_hz;
        toneQueue.end_time[toneQueue.tail]      = (tmp + (duration_ms * TIMING_SCALER));
        toneQueue.count++;

        if(toneQueue.tail == (MAX_QUEUE_SIZE - 1))
        {
            toneQueue.tail = 0;     /* wrap around */
        }
        else
        {
            toneQueue.tail++;
        }
        status = TRUE;

        if(handling_tones == FALSE)
        {
            handling_tones = TRUE;
            RegisterTimer(HandleQueue, 0, TONE_HANDLE_TIMING, TRUE, FALSE);
        }
    }


    return status;
}

/*      "QueueTune"
================================================================================
@brief  Schedule a tune to be played (FIFO)
@param  uint16_t *hz_buff:              buffer of frequencies to play
@param  uint32_t *hz_duration_ms:       buffer containing the timing per frequency
@param  uint8_t buffer_size:            size of buffer
@retval none
*******************************************************************************/
void QueueTune(uint16_t *hz_buff, uint32_t *hz_duration_ms, uint8_t buffer_size)
{
    uint32_t reference = 0;

    QueueTone(hz_buff[0], hz_duration_ms[0]);

    reference = hz_duration_ms[0];

    for(uint8_t note=1; note<buffer_size; note++)
    {
        reference = reference + hz_duration_ms[note];
        QueueTone(hz_buff[note], reference);
    }

    return;
}

/*      "HandleQueue"
================================================================================
@brief  Manage the generation of queued tones .
@param  uint16_t null:  used for scheduling through use of "RegisterTimer()"
@retval none
*******************************************************************************/
static void HandleQueue(uint8_t null)
{
    if(myspeed != 0)
    {
        /* Not safe to beep */
    }
    else
    {
        if(toneQueue.count > 0)
        {
            if(ready_to_beep != TRUE)
            {
                if(beep_method != 0)
                {
                    InitBeep();
                }
                else
                {
                    InitBeep2();
                }
            }
            else
            {
                if((toneQueue.end_time[toneQueue.head]) > GetMsSinceStart())
                {
                    if(toneQueue.freq_hz[toneQueue.head] == 0)
                    {
                        /* For 0, play a tone that cannot be heard */
                        TIM_PrescalerConfig(TIM1, SILENT_PRESCALER, TIM_PSCReloadMode_Update);
                    }
                    else
                    {
                        if(toneQueue.freq_hz[toneQueue.head] < MIN_PLAYABLE_FREQUENCY)
                        {
                            TIM_PrescalerConfig(TIM1, ((SystemCoreClock / DEFAULT_PERIOD) / MIN_PLAYABLE_FREQUENCY), TIM_PSCReloadMode_Update);
                        }
                        else
                        {
                            TIM_PrescalerConfig(TIM1, ((SystemCoreClock / DEFAULT_PERIOD) / (toneQueue.freq_hz[toneQueue.head])), TIM_PSCReloadMode_Update);
                        }

                        //TIM_CtrlPWMOutputs(TIM1, ENABLE);
                    }
                }
                else
                {
                    /* time to play queued tone has passed */
                    if(toneQueue.head == (MAX_QUEUE_SIZE - 1))
                    {
                        toneQueue.head = 0;     /* wrap around */
                    }
                    else
                    {
                        toneQueue.head++;
                    }
                    //TIM_CtrlPWMOutputs(TIM1, DISABLE);
                    toneQueue.count--;
                }
            }
        }
        else
        {
            CancelTimer(HandleQueue, 0);
            RestoreMotorSettings();
            handling_tones = FALSE;
        }
    }


    return;
}

/*      "PlayGlissando"
================================================================================
@brief  Blocking function used to "slide" between two frequencies over a given duration
@param  uint16_t start_hz:      frequency to start at (hertz)
@param  uint16_t end_hz:        freqeuncy to end on (hertz)
@param  uint32_t duration_ms:   duration of the transition (milliseconds)
@retval none
*******************************************************************************/
static void PlayGlissando(uint16_t start_hz, uint16_t end_hz, uint32_t duration_ms)
{
    uint16_t min_freq = 0;
    uint16_t max_freq = 0;
    uint16_t transition_tone = 0;

    /* Bounds & limits */
    if(start_hz > end_hz)
    {
        max_freq = start_hz;
        min_freq = end_hz;
    }
    else
    {
        max_freq = end_hz;
        min_freq = start_hz;
    }

    transition_tone = start_hz;

    /*Config TIM */
    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
    TIM_TimeBase_InitStructure.TIM_ClockDivision      = TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_Period             = DEFAULT_PERIOD;
    TIM_TimeBase_InitStructure.TIM_Prescaler          = ((SystemCoreClock / DEFAULT_PERIOD) / start_hz);
    TIM_TimeBaseInit(TIM1, &TIM_TimeBase_InitStructure);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /* Delay */
    uint32_t start_time = 0;
    uint32_t temp_timer = 0;
    int16_t step = 0;

    start_time = GetMsSinceStart();
    temp_timer = start_time;
    step = ((int16_t)end_hz - (int16_t)start_hz) / ((int16_t)duration_ms / TONE_HANDLE_TIMING);

    duration_ms = duration_ms * TIMING_SCALER;
    while(GetMsElapsed(start_time) < duration_ms)
    {
        if(GetMsElapsed(temp_timer) >= TONE_HANDLE_TIMING)   /* testing shows that requested duration is halved */
        {
            temp_timer = GetMsSinceStart();
            transition_tone = transition_tone + step;

            if(transition_tone < min_freq)
            {
                transition_tone = min_freq;
            }
            else if(transition_tone > max_freq)
            {
                transition_tone = max_freq;
            }
            TIM_PrescalerConfig(TIM1, ((SystemCoreClock / DEFAULT_PERIOD) / transition_tone), TIM_PSCReloadMode_Update);
        }
    }

    /* Stop PWM */
    TIM_CtrlPWMOutputs(TIM1, DISABLE);

    return;
}

/* Used to test freqeuncies between min and 20000 Hz */
static uint16_t freqeuncy_viewer = MIN_PLAYABLE_FREQUENCY;
static uint16_t frequency_viewer_duration = 1000;
static void TestScale(void)
{
    /* Frequency resonance @ 450, 880, 1758, and 16312 Hz */
    for(; freqeuncy_viewer<20000; freqeuncy_viewer=freqeuncy_viewer+100)
    {
        PlayToneBlocking(freqeuncy_viewer, frequency_viewer_duration);
    }

    return;
}
