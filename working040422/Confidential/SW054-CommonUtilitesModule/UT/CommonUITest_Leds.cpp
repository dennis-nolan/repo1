extern "C"
{
#include "ThreadingHeader.h"
#include "TimersHeader.h"
#include "CommonUIHeader.h"
#include "UIMocks.h"
}

#include "CppUTest/TestHarness.h"

extern bool* ledState;
extern bool stateChangeCallbackCalled;

static int i;

TEST_GROUP(CommonUITestGroup_Leds)
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

/*TEST(CommonUITestGroup_Leds, TestMocks)
{
	UIMockInit(9, 12);
	CHECK(stateChangeCallbackCalled == FALSE);
}

TEST(CommonUITestGroup_Leds, TestLedsSimple)
{
	UIMockInit(0, 2);
	//ConfigureLeds(2);
	
	// check init to OFF
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == FALSE);
	
	// cycle through on/off states and verify
	LedPatternSet(0, LED_ON, 0);
	CHECK(ledState[0] == TRUE);
	CHECK(ledState[1] == FALSE);
	LedPatternSet(1, LED_ON, 0);
	CHECK(ledState[0] == TRUE);
	CHECK(ledState[1] == TRUE);
	LedPatternSet(0, LED_OFF, 0);
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == TRUE);
	LedPatternSet(1, LED_OFF, 0);
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == FALSE);
}*/

TEST(CommonUITestGroup_Leds, TestLedsPattern)
{
	UIMockInit(0, 2);
	//ConfigureLeds(2);
	
	// check init to OFF
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == FALSE);
	
	// setup pattern for LED 0
	LedPatternSet(0, LED_TOGGLE, 150);
	
	// setup pattern for LED 1
	LedPatternSet(1, LED_TOGGLE, 200);
	
	// Verify still off
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == FALSE);
	
	// Run timer/thread stuff
	for(i=0; i<150; i++)
	{ Tick(); RunAllThreads();}
	
	// Verify 0 turned on
	CHECK(ledState[0] == TRUE);
	CHECK(ledState[1] == FALSE);
	
	// Run timer/thread stuff
	for(i=0; i<50; i++)
	{ Tick(); RunAllThreads();}
	
	// Verify 1 turned on
	CHECK(ledState[0] == TRUE);
	CHECK(ledState[1] == TRUE);
	
	// Run timer/thread stuff
	for(i=0; i<100; i++)
	{ Tick(); RunAllThreads();}
	
	// Verify 0 turned off
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == TRUE);
	
	// Run timer/thread stuff
	for(i=0; i<100; i++)
	{ Tick(); RunAllThreads();}
	
	// Verify 1 turned off
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == FALSE);
	
	// Check that we can properly stop the pattern
	LedPatternSet(0, LED_ON, 0);
	LedPatternSet(1, LED_ON, 0);
	CHECK(ledState[0] == TRUE);
	CHECK(ledState[1] == TRUE);
	
	// Run timer/thread stuff
	for(i=0; i<300; i++)
	{ Tick(); RunAllThreads();}
	
	LedPatternSet(0, LED_OFF, 0);
	LedPatternSet(1, LED_OFF, 0);
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == FALSE);
}

TEST(CommonUITestGroup_Leds, TestLedsPatternAdv)
{
	UIMockInit(0, 4);
	//ConfigureLeds(2);
	
	// check init to OFF
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == FALSE);
	CHECK(ledState[2] == FALSE);
	CHECK(ledState[3] == FALSE);
	
	// setup pattern for LED 2
	LedPatternSetAdvanced(2, LED_TOGGLE, 997, 73, 0, FALSE);
	
	for (i=0; i<3; i++)
	{
		// Verify still off
		CHECK(ledState[0] == FALSE);
		CHECK(ledState[1] == FALSE);
		CHECK(ledState[2] == FALSE);
		CHECK(ledState[3] == FALSE);
		
		// Run timer/thread stuff
		for(i=0; i<72; i++)
		{ Tick(); RunAllThreads();}
		
		// Verify still off
		CHECK(ledState[0] == FALSE);
		CHECK(ledState[1] == FALSE);
		CHECK(ledState[2] == FALSE);
		CHECK(ledState[3] == FALSE);
		
		//last tick
		Tick();
		RunAllThreads();
		
		// Verify 2 turned on
		CHECK(ledState[0] == FALSE);
		CHECK(ledState[1] == FALSE);
		CHECK(ledState[2] == TRUE);
		CHECK(ledState[3] == FALSE);
		
		// Run timer/thread stuff
		for(i=0; i<996; i++)
		{ Tick(); RunAllThreads();}
		
		// Verify 2 still on
		CHECK(ledState[0] == FALSE);
		CHECK(ledState[1] == FALSE);
		CHECK(ledState[2] == TRUE);
		CHECK(ledState[3] == FALSE);
		
		//last tick
		Tick();
		RunAllThreads();
		
		// Verify 2 turned off
		CHECK(ledState[0] == FALSE);
		CHECK(ledState[1] == FALSE);
		CHECK(ledState[2] == FALSE);
		CHECK(ledState[3] == FALSE);
	}
	
	// Check that we can properly stop the pattern
	LedPatternSet(2, LED_ON, 0);
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == FALSE);
	CHECK(ledState[2] == TRUE);
	CHECK(ledState[3] == FALSE);
	
	// Run timer/thread stuff
	for(i=0; i<1500; i++)
	{ Tick(); RunAllThreads();}
	
	// Verify 2 still on
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == FALSE);
	CHECK(ledState[2] == TRUE);
	CHECK(ledState[3] == FALSE);
	
	//shut off 2
	LedPatternSet(2, LED_OFF, 0);
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == FALSE);
	CHECK(ledState[2] == FALSE);
	CHECK(ledState[3] == FALSE);
	
	// Run timer/thread stuff
	for(i=0; i<1500; i++)
	{ Tick(); RunAllThreads();}
	
	// Verify 2 still off
	CHECK(ledState[0] == FALSE);
	CHECK(ledState[1] == FALSE);
	CHECK(ledState[2] == FALSE);
	CHECK(ledState[3] == FALSE);
}

TEST(CommonUITestGroup_Leds, TestLedsPatternOverwrite)
{
	UIMockInit(0, 1);
	
	// check init to OFF
	CHECK(ledState[0] == FALSE);
	
	// setup pattern for LED 0
	LedPatternSet(0, LED_TOGGLE, 150);
	
	// off for 150
	for(i=0; i<150; i++)
	{
		CHECK(ledState[0] == FALSE);
		Tick();
		RunAllThreads();
	}
	
	// on for 150
	for(i=0; i<150; i++)
	{
		CHECK(ledState[0] == TRUE);
		Tick();
		RunAllThreads();
	}
	
	// off for 59
	for(i=0; i<59; i++)
	{
		CHECK(ledState[0] == FALSE);
		Tick();
		RunAllThreads();
	}
	
	// setup NEW pattern for LED 0
	LedPatternSet(0, LED_TOGGLE, 200);
	
	// off for 200
	for(i=0; i<200; i++)
	{
		CHECK(ledState[0] == FALSE);
		Tick();
		RunAllThreads();
	}
	
	// on for 200
	for(i=0; i<200; i++)
	{
		CHECK(ledState[0] == TRUE);
		Tick();
		RunAllThreads();
	}
	
	// off for 200
	for(i=0; i<200; i++)
	{
		CHECK(ledState[0] == FALSE);
		Tick();
		RunAllThreads();
	}
	
	// on for 67
	for(i=0; i<67; i++)
	{
		CHECK(ledState[0] == TRUE);
		Tick();
		RunAllThreads();
	}
	
	// Check that we can properly stop the pattern
	LedPatternSet(0, LED_OFF, 0);
	CHECK(ledState[0] == FALSE);
	
	// check that it doesn't come back on
	for(i=0; i<300; i++)
	{
		CHECK(ledState[0] == FALSE);
		Tick();
		RunAllThreads();
	}
}

