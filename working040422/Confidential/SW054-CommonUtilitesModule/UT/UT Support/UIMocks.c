
#include <stdlib.h>
#include "UIMocks.h"
#include "CommonUIHeader.h"

static uint8_t numButtons, numLeds;
bool* ledState = 0;
bool* buttonState = 0;
bool stateChangeCallbackCalled = FALSE;

void UIMockInit(uint8_t totalNumButtons, uint8_t totalNumLeds)
{
	numButtons = totalNumButtons;
	if (numButtons > 0)
		buttonState = (bool*)malloc(numButtons);
	else
		buttonState = 0;
	
	numLeds = totalNumLeds;
	if (numLeds > 0)
		ledState = (bool*)malloc(numLeds);
	else
		ledState = 0;
	
	stateChangeCallbackCalled = FALSE;
}

void ResetUIMocks(void)
{
	numButtons = 0;
	if (buttonState != 0)
		free(buttonState);
	buttonState = 0;
	
	numLeds = 0;
	if (ledState != 0)
		free(ledState);
	ledState = 0;
	
	stateChangeCallbackCalled = FALSE;
}

bool GetHardwareButtonState(uint8_t buttonIndex)
{
	return buttonState[buttonIndex];
}

void SetLedHardware(uint8_t ledIndex, bool turnOn)
{
	ledState[ledIndex] = turnOn;
}

void ButtonStateChange(void)
{
	stateChangeCallbackCalled = TRUE;
}