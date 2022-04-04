extern "C"
{
#include "ThreadingHeader.h"
#include "TimersHeader.h"
#include "CommonUIHeader.h"
#include "UIMocks.h"
}

#include "CppUTest/TestHarness.h"

extern bool* buttonState;
extern bool stateChangeCallbackCalled;

TEST_GROUP(CommonUITestGroup_Buttons)
{
	void setup() {
		ThreadingInit();
		TimerInit();
	}
	
	void teardown() {
		ResetUIMocks();
		//ResetCommonUI();
	}
};

TEST(CommonUITestGroup_Buttons, TestMocks)
{
	UIMockInit(9, 12);
	CHECK(stateChangeCallbackCalled == FALSE);
}

TEST(CommonUITestGroup_Buttons, TestButtons)
{
	UIMockInit(4, 0);
	
	//init button values
	buttonState[0] = FALSE;
	buttonState[1] = FALSE;
	buttonState[2] = FALSE;
	buttonState[3] = FALSE;
	
	//config buttons & sampling
	InitUI(4, 2, 3, 2, 200, 400);
	//bool activeHigh[4] = {TRUE,TRUE,TRUE,TRUE};
	//ConfigureButtons(4, activeHigh);
	
	// Run timer/thread stuff
	for(int i=0; i<300; i++)
	{ Tick(); RunAllThreads();}
	
	// check no presses registered
	CHECK(GetButtonState(0)==NO_VALID_PRESS);
	CHECK(GetButtonState(1)==NO_VALID_PRESS);
	CHECK(GetButtonState(2)==NO_VALID_PRESS);
	CHECK(GetButtonState(3)==NO_VALID_PRESS);
	CHECK(stateChangeCallbackCalled == FALSE);
	
	//set new button values
	buttonState[0] = FALSE;
	buttonState[1] = TRUE;
	buttonState[2] = TRUE;
	buttonState[3] = FALSE;
	
	// we are set to 2 ticks/sample, 2 out of 3 samples is a press so 4 should do it
	for(int i=0; i<3; i++)
	{ Tick(); RunAllThreads();}
	
	// check no presses registered
	CHECK(GetButtonState(0)==NO_VALID_PRESS);
	CHECK(GetButtonState(1)==NO_VALID_PRESS);
	CHECK(GetButtonState(2)==NO_VALID_PRESS);
	CHECK(GetButtonState(3)==NO_VALID_PRESS);
	CHECK(stateChangeCallbackCalled == FALSE);
	
	// one more time and they should be updated
	Tick(); RunAllThreads();
	
	// verify presses
	CHECK(GetButtonState(0)==NO_VALID_PRESS);
	CHECK(GetButtonState(1)==PRESS_IMMEDIATE);
	CHECK(GetButtonState(2)==PRESS_IMMEDIATE);
	CHECK(GetButtonState(3)==NO_VALID_PRESS);
	CHECK(stateChangeCallbackCalled == TRUE);
	stateChangeCallbackCalled = FALSE;
	
	//set new button values
	buttonState[0] = TRUE;
	buttonState[1] = FALSE;
	buttonState[2] = FALSE;
	buttonState[3] = TRUE;
	
	for(int i=0; i<4; i++)
	{ Tick(); RunAllThreads();}
	
	// verify presses
	CHECK(GetButtonState(0)==PRESS_IMMEDIATE);
	CHECK(GetButtonState(1)==SINGLE_PRESS_IMMEDIATE);
	CHECK(GetButtonState(2)==SINGLE_PRESS_IMMEDIATE);
	CHECK(GetButtonState(3)==PRESS_IMMEDIATE);
	CHECK(stateChangeCallbackCalled == TRUE);
	stateChangeCallbackCalled = FALSE;
	
	//set new button values
	buttonState[0] = TRUE; // will be held
	buttonState[1] = TRUE; // double press
	buttonState[2] = FALSE; // back to nothing
	buttonState[3] = FALSE; // single press
	
	for(int i=0; i<4; i++)
	{ Tick(); RunAllThreads();}
	
	// verify presses
	CHECK(GetButtonState(0)==PRESS_IMMEDIATE);
	CHECK(GetButtonState(1)==DOUBLE_PRESS);
	CHECK(GetButtonState(2)==SINGLE_PRESS_IMMEDIATE);
	CHECK(GetButtonState(3)==SINGLE_PRESS_IMMEDIATE);
	CHECK(stateChangeCallbackCalled == TRUE);
	stateChangeCallbackCalled = FALSE;
	
	//set new button values
	buttonState[0] = TRUE; // will be held
	buttonState[1] = FALSE; // back to nothing
	buttonState[2] = FALSE; // back to nothing
	buttonState[3] = FALSE; // back to nothing
	
	//wait for held timeout and double press deadtime
	for(int i=0; i<510; i++)
	{ Tick(); RunAllThreads();}
	
	// verify presses
	CHECK(GetButtonState(0)==BUTTON_HELD);
	CHECK(GetButtonState(1)==NO_VALID_PRESS);
	CHECK(GetButtonState(2)==NO_VALID_PRESS);
	CHECK(GetButtonState(3)==NO_VALID_PRESS);
	CHECK(stateChangeCallbackCalled == TRUE);
	stateChangeCallbackCalled = FALSE;
	
	//set new button values
	buttonState[0] = FALSE; // released
	buttonState[1] = FALSE;
	buttonState[2] = FALSE;
	buttonState[3] = FALSE;
	
	for(int i=0; i<4; i++)
	{ Tick(); RunAllThreads();}
	
	// verify presses
	CHECK(GetButtonState(0)==BUTTON_RELEASED);
	CHECK(GetButtonState(1)==NO_VALID_PRESS);
	CHECK(GetButtonState(2)==NO_VALID_PRESS);
	CHECK(GetButtonState(3)==NO_VALID_PRESS);
	CHECK(stateChangeCallbackCalled == TRUE);
	stateChangeCallbackCalled = FALSE;
	
	for(int i=0; i<4; i++)
	{ Tick(); RunAllThreads();}
	
	// verify presses
	CHECK(GetButtonState(0)==NO_VALID_PRESS);
	CHECK(GetButtonState(1)==NO_VALID_PRESS);
	CHECK(GetButtonState(2)==NO_VALID_PRESS);
	CHECK(GetButtonState(3)==NO_VALID_PRESS);
	CHECK(stateChangeCallbackCalled == TRUE);
	stateChangeCallbackCalled = FALSE;
	
	//check no further state changes
	for(int i=0; i<4000; i++)
	{ Tick(); RunAllThreads();}
	CHECK(GetButtonState(0)==NO_VALID_PRESS);
	CHECK(GetButtonState(1)==NO_VALID_PRESS);
	CHECK(GetButtonState(2)==NO_VALID_PRESS);
	CHECK(GetButtonState(3)==NO_VALID_PRESS);
	CHECK(stateChangeCallbackCalled == FALSE);
}

/*TEST(CommonUITestGroup_Buttons, TestButtons2)
{
	uint8_t testvector1[] = {0,0,1,0,0,1,1,0,1,0,1,1,0,1,0,0,1,0,0,0,1,0,0,0};
	//						{off                    ,on
	UIMockInit(2, 0);
	
	//init button values
	buttonState[0] = FALSE;
	buttonState[1] = FALSE;
	
	//config buttons & sampling
	InitUI(2, 1, 7, 5, 500, 500);
	
	// Run timer/thread stuff
	for(int i=0; i<300; i++)
	{ Tick(); RunAllThreads();}
	
	// check no presses registered
	CHECK(GetButtonState(0)==NO_VALID_PRESS);
	CHECK(GetButtonState(1)==NO_VALID_PRESS);
	CHECK(GetButtonState(2)==NO_VALID_PRESS);
	CHECK(GetButtonState(3)==NO_VALID_PRESS);
	CHECK(stateChangeCallbackCalled == FALSE);
	
	//set new button values
	buttonState[0] = FALSE;
	buttonState[1] = TRUE;
}*/