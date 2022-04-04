/*
  Author:       Christopher Sisko
  File:         new_beep_header.h
  Project:      Avenger's Propulsion
  Board #:      EA082
  IAR-ver:      8.22.2

********************************************************************************
*/
#ifndef NEW_BEEP_HEADER
#define NEW_BEEP_HEADER

/*================================ includes ==================================*/

/*================================ typedefs ==================================*/
typedef enum {
    TEST_TONE,
    TEST_GLISS,                 /* non-blocking only */
    TEST_TUNE,
    TEST_SCALE,
    CALIBRATION_COMPLETE,
} motif;

/*================================ #defines ==================================*/

/*================================ #macros ===================================*/

/*=========================== extern variables ===============================*/

/*========================= function prototypes ==============================*/

/**
  * @brief  Function used to play pre-defined tunes
  * @param  motif tune:     enumerated value denoting melody to played.
  * @param  bool blocking:  bool to indicate if tone should block on execution
  * @retval bool:           True if operation carried out, False is not
  *
  * @note   safety requires that myspeed == 0 for queuing/playing of tones
  */
bool PlayTuneNew(motif tune, bool blocking);

/**
  * @brief  indicate if motor is configured for tone generation.
  * @param  none
  * @retval TRUE if configured for tone generation.
            FALSE if motor library configuration
  */
bool IsTunePlaying(void);
#endif