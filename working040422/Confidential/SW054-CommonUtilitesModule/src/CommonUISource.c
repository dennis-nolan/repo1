
//#include "MemoryMgmtHeader.h"
#include "CommonUIHeader.h"
#include "TimersHeader.h"

#define MAX_BUTTONS	4
#define MAX_LEDS	4

#define IS_VALID_BUTTON_INDEX(x) (x<MAX_BUTTONS)
#define IS_VALID_LED_INDEX(x) (x<MAX_LEDS)

typedef struct {
	uint8_t history;	// Stores a history of 8 reads, pre-adjusted for active high/low so that 1 is active
	//uint8_t simpleState;
	//bool doublePress;
	uint8_t state;
	uint16_t updatesInState;
} Button;

typedef struct {
	bool on;
	uint8_t cmd;
	uint16_t onTime;
	uint16_t offTime;
} Led;

static Button buttons[MAX_BUTTONS];
static Led leds[MAX_LEDS];
static uint8_t numButtons=0;
static uint8_t pressSampleMask = 0x1F;
static uint8_t pressReq = 5;
//static uint16_t pressTimeoutLen = 350; //ms
//static uint16_t deadtimeLen = 500; //ms
static uint8_t sampleRate = 2;
static uint8_t previous;
static bool buttonsChanged;
static uint8_t count;


// These should be overidden
__weak void ButtonStateChange(void){}
__weak bool GetHardwareButtonState(uint8_t buttonIndex){return FALSE;}
__weak void SetLedHardware(uint8_t ledIndex, bool turnOn){}


static uint8_t CountSetBits(uint8_t value)
{
	// Only iterates as many times as there are bits set
	for(count = 0; value; count++)
	{ value &= value - 1; }
	
	return count;
}

/*uint8_t GetSimpleButtonState(uint8_t index)
{
	if (!IS_VALID_BUTTON_INDEX(index))
		return UNDETERMINED;
	
	return buttons[index].simpleState;
}*/

void UpdateButtons(uint8_t dummy)
{
	uint8_t count;
	buttonsChanged = FALSE;
	
	for (uint8_t i=0; i<numButtons; i++)
	{
		////////////////// Update history //////////////////
		// Shift a 0 in from the right
		buttons[i].history <<= 1;
		
		// If the HW state is active then update this read to a 1
		if (GetHardwareButtonState(i))
			buttons[i].history |= 1;
		
		/////////////// Update simple state ////////////////
		previous = buttons[i].state;
		count = CountSetBits(buttons[i].history & pressSampleMask);
		if (count >= pressReq)
			buttons[i].state = PRESSED;
		else if ((8-count) >= pressReq)
			buttons[i].state = RELEASED;
		else
			buttons[i].state = UNDETERMINED;
		
		/////////////// Update complex state ///////////////
		/*previous = buttons[i].state;
		
		if (GetSimpleButtonState(i) == PRESSED)
		{ // verified press
			switch (buttons[i].state)
			{
				case NO_VALID_PRESS:
					buttons[i].state = PRESS_IMMEDIATE;
					break;
				case PRESS_IMMEDIATE:
					// if pressTimeoutLen expires then the button has been held
					if (buttons[i].updatesInState * sampleRate >= pressTimeoutLen)
					{
						buttons[i].state = BUTTON_HELD;
					}
					//else we are waiting for a release
					break;
				case SINGLE_PRESS_IMMEDIATE:
					// a single press has been completed and another press has occurred before the timeout
					if (buttons[i].updatesInState * sampleRate < pressTimeoutLen)
					{
						buttons[i].state = DOUBLE_PRESS;
					}
					break;
				// do nothing for presses during deadtime after DOUBLE_PRESS
				// do nothing while held
			}
		}
		else if (GetSimpleButtonState(i) == RELEASED)
		{ // release
			switch (buttons[i].state)
			{
				// no action for NO_VALID_PRESS
				case PRESS_IMMEDIATE:
					// if we are still in this state and there is a release this is a press/release
					buttons[i].state = SINGLE_PRESS_IMMEDIATE;
					break;
				case SINGLE_PRESS_IMMEDIATE:
					if (buttons[i].updatesInState * sampleRate >= pressTimeoutLen)
					{
						// can't be a double anymore.
						buttons[i].state = SINGLE_PRESS_FINAL;
					}
					break;
				case SINGLE_PRESS_FINAL:
					buttons[i].state = NO_VALID_PRESS;
					break;
				case DOUBLE_PRESS:
					if (buttons[i].updatesInState * sampleRate >= deadtimeLen)
					{
						// while in this state we've been ignoring presses.
						// now that deadtime is over we start fresh
						buttons[i].state = NO_VALID_PRESS;
					}
					break;
				case BUTTON_HELD:
					buttons[i].state = BUTTON_RELEASED;
					break;
				case BUTTON_RELEASED:
					///////////////////TODO: should this really be immediate?
					buttons[i].state = NO_VALID_PRESS;
					break;
			}
		}
		//else undetermined. wait for button to settle.
		*/
		
		// keep track of how long we've been in this state
		if (previous == buttons[i].state)
			buttons[i].updatesInState += (buttons[i].updatesInState<0xFFFF);
		else
		{
			buttons[i].updatesInState = 0;
			
			//if (buttons[i].state != UNDETERMINED)
				buttonsChanged = TRUE;
		}
	}
	
	if (buttonsChanged)
		ButtonStateChange(); //inform app
}

static void LedTimeout(uint8_t ledIndex)
{
	if (!IS_VALID_LED_INDEX(ledIndex) || leds[ledIndex].cmd != LED_TOGGLE)
		return;
	
	// Toggle LED & re-register timer
	leds[ledIndex].on = (bool)!leds[ledIndex].on;
	SetLedHardware(ledIndex, leds[ledIndex].on);
	if (leds[ledIndex].on)
		RegisterTimer(LedTimeout, ledIndex, leds[ledIndex].onTime, FALSE, FALSE);
	else
		RegisterTimer(LedTimeout, ledIndex, leds[ledIndex].offTime, FALSE, FALSE);
}

void InitUI(uint8_t totalNumButtons, uint8_t sampleRateMs, uint8_t numSamples, uint8_t pressRequirement)//, uint16_t pressTimeout, uint16_t deadtime)
{
	if (IS_VALID_BUTTON_INDEX(totalNumButtons - 1))
		numButtons = totalNumButtons;
	
	// Fill in button structures
	for (uint8_t i=0; i<numButtons; i++)
	{
		buttons[i].history = 0;
		buttons[i].state = UNDETERMINED;//NO_VALID_PRESS;
		buttons[i].updatesInState = 0;
	}
	
	// Fill in LED structures
	for (uint8_t i=0; i<MAX_LEDS; i++)
	{
		SetLedHardware(i, FALSE);
		leds[i].on = FALSE;
		leds[i].cmd = LED_OFF;
		leds[i].onTime = 0;
		leds[i].offTime = 0;
	}
	
	// numSamples must be in the range 1-8 inclusive
	if (numSamples > 8)
		numSamples = 8;
	else if (numSamples == 0)
		numSamples = 1;
	
	// calculate sample mask
	pressSampleMask = 0xFF>>(8-numSamples);
	
	// press requirement must be in the range 1-numSamples inclusive
	if (pressRequirement > numSamples)
		pressReq = numSamples;
	else if (pressRequirement == 0)
		pressReq = 1;
	else
		pressReq = pressRequirement;
	
	// Set the timeouts
	//pressTimeoutLen = pressTimeout;
	//deadtimeLen = deadtime;
	
	// set button sampling timer
	sampleRate = sampleRateMs;
	RegisterTimer(UpdateButtons, 0, sampleRate, TRUE, TRUE);
}

bool LedPatternSet(uint8_t ledIndex, uint8_t cmd, uint16_t rate)
{
	/*if (cmd > LED_TOGGLE || ((onTime == 0 || offTime == 0) && cmd == LED_TOGGLE) || ledIndex >= MAX_LEDS)
		return FALSE;
	
	leds[ledIndex].cmd = cmd;
	leds[ledIndex].onTime = rate;
	leds[ledIndex].offTime = rate;
	
	if (cmd == LED_ON)
	{
		SetLedHardware(ledIndex, TRUE);
		leds[ledIndex].on = TRUE;
	}
	else if (cmd == LED_OFF)
	{
		SetLedHardware(ledIndex, FALSE);
		leds[ledIndex].on = FALSE;
	}
	else if (cmd == LED_TOGGLE)
		RegisterTimer(LedTimeout, ledIndex, leds[ledIndex].on?onTime:offTime, FALSE, FALSE);
	
	return TRUE;*/

	return LedPatternSetAdvanced(ledIndex, cmd, rate, rate, 0, TRUE);
}

bool LedPatternSetAdvanced(uint8_t ledIndex, uint8_t cmd, uint16_t onTime, uint16_t offTime, uint16_t startDelay, bool startOn)
{
	if (cmd > LED_TOGGLE || ((onTime == 0 || offTime == 0) && cmd == LED_TOGGLE) || !IS_VALID_LED_INDEX(ledIndex))
		return FALSE;
	
	leds[ledIndex].cmd = cmd;
	leds[ledIndex].onTime = onTime;
	leds[ledIndex].offTime = offTime;
	
	if (cmd == LED_ON)
	{
		SetLedHardware(ledIndex, TRUE);
		leds[ledIndex].on = TRUE;
	}
	else if (cmd == LED_OFF)
	{
		SetLedHardware(ledIndex, FALSE);
		leds[ledIndex].on = FALSE;
	}
	else if (cmd == LED_TOGGLE)
	{
		SetLedHardware(ledIndex, startOn);
		leds[ledIndex].on = startOn;
		
		if (startDelay > 0)
			RegisterTimer(LedTimeout, ledIndex, startDelay, FALSE, FALSE);
		else if (leds[ledIndex].on)
			RegisterTimer(LedTimeout, ledIndex, onTime, FALSE, FALSE);
		else
			RegisterTimer(LedTimeout, ledIndex, offTime, FALSE, FALSE);
	}
	
	return TRUE;
}

uint8_t GetButtonState(uint8_t buttonIndex)
{
	if (!IS_VALID_BUTTON_INDEX(buttonIndex))
		return UNDETERMINED;//NO_VALID_PRESS;
	
	return buttons[buttonIndex].state;
}

uint16_t GetButtonTimeInState(uint8_t buttonIndex)
{
	if (!IS_VALID_BUTTON_INDEX(buttonIndex))
		return 0xFFFF;
	
	return (sampleRate * buttons[buttonIndex].updatesInState);
}