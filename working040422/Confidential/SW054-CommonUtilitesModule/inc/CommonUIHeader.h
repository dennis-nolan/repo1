
#include "CommonTypes.h"

//button simple state enum
enum {UNDETERMINED,PRESSED,RELEASED};

//button state enum
//enum {NO_VALID_PRESS=0,PRESS_IMMEDIATE,SINGLE_PRESS_IMMEDIATE,SINGLE_PRESS_FINAL,DOUBLE_PRESS,BUTTON_HELD,BUTTON_RELEASED};

//LED command enum
enum {LED_OFF=0, LED_ON, LED_TOGGLE};

void InitUI(uint8_t totalNumButtons, uint8_t sampleRateMs, uint8_t numSamples, uint8_t pressRequirement);//, uint16_t pressTimeout, uint16_t deadtime);
bool LedPatternSet(uint8_t ledIndex, uint8_t cmd, uint16_t rate);
bool LedPatternSetAdvanced(uint8_t ledIndex, uint8_t cmd, uint16_t onTime, uint16_t offTime, uint16_t startDelay, bool startOn);
uint8_t GetButtonState(uint8_t buttonIndex);
uint16_t GetButtonTimeInState(uint8_t buttonIndex);
//uint8_t GetSimpleButtonState(uint8_t index);
void UpdateButtons(uint8_t dummy);

// These functions are required to be implemented in the hardware-specific project
bool GetHardwareButtonState(uint8_t buttonIndex);
void SetLedHardware(uint8_t ledIndex, bool turnOn);
void ButtonStateChange(void);
