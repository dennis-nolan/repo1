extern "C"
{
#include "TimersHeader.h"
#include "CallbackTestHelper.h"
}

#include "CppUTest/TestHarness.h"

TEST_GROUP(TimersTestGroup)
{
	void setup()
	{
		ResetCallbackCalled();
		TimerInit();
	}
	
	void teardown() {}
};

TEST(TimersTestGroup, TestTimerInit)
{
	CHECK(1);
}

TEST(TimersTestGroup, TestTickWithNoTimers)
{
	for(int i=0; i<500; i++)
		Tick();
	RunTimerBackgroundTasks();
	
	CHECK(AnyCallbacksCalled() == 0);
}

TEST(TimersTestGroup, TestTickWith1Timer_notpersistent)
{
	RegisterTimer(TestCallback1_param, 16, 350, FALSE, FALSE);
	
	CHECK(GetCallbackCalled(1) == 0);
	
	for(int i=0; i<349; i++)
		Tick();
	RunTimerBackgroundTasks();
	
	CHECK(GetCallbackCalled(1) == 0);
	
	Tick();
	RunTimerBackgroundTasks();
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetLatestCallbackID() == 16);
	
	// make sure it doesn't get called again
	SetCallbackCalled(1, 0);
	
	for(int i=0; i<400; i++)
		Tick();
	RunTimerBackgroundTasks();
	
	CHECK(GetCallbackCalled(1) == 0);
}

TEST(TimersTestGroup, TestTickWith1Timer_notpersistent_immediate)
{
	RegisterTimer(TestCallback1_param, 16, 350, FALSE, TRUE);
	
	CHECK(GetCallbackCalled(1) == 0);
	
	for(int i=0; i<349; i++)
		Tick();
	RunTimerBackgroundTasks();
	
	CHECK(GetCallbackCalled(1) == 0);
	
	Tick();
	// Timer should execute in Tick context rather than background task
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetLatestCallbackID() == 16);
	
	// make sure it doesn't get called again
	SetCallbackCalled(1, 0);
	
	for(int i=0; i<400; i++)
		Tick();
	RunTimerBackgroundTasks();
	
	CHECK(GetCallbackCalled(1) == 0);
}

TEST(TimersTestGroup, TestTickWith1Timer_persistent)
{
	RegisterTimer(TestCallback1_param, 65, 350, TRUE, FALSE);
	
	CHECK(GetCallbackCalled(1) == 0);
	
	for(int i=0; i<349; i++)
		Tick();
	RunTimerBackgroundTasks();
	
	CHECK(GetCallbackCalled(1) == 0);
	
	Tick();
	RunTimerBackgroundTasks();
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetLatestCallbackID() == 65);
	
	// make sure it DOES get called again
	SetCallbackCalled(1, 0);
	
	for(int i=0; i<349; i++)
		Tick();
	RunTimerBackgroundTasks();
	CHECK(GetCallbackCalled(1) == 0);
	
	Tick();
	RunTimerBackgroundTasks();
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetLatestCallbackID() == 65);
}

TEST(TimersTestGroup, TestTickWith1Timer_persistent_immediate)
{
	RegisterTimer(TestCallback1_param, 65, 350, TRUE, TRUE);
	
	CHECK(GetCallbackCalled(1) == 0);
	
	for(int i=0; i<349; i++)
		Tick();
	RunTimerBackgroundTasks();
	
	CHECK(GetCallbackCalled(1) == 0);
	
	Tick();
	// Timer should execute in Tick context rather than background task
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetLatestCallbackID() == 65);
	
	// Makes sure it doesn't run again when background tasks run
	SetCallbackCalled(1, 0);
	RunTimerBackgroundTasks();
	CHECK(GetCallbackCalled(1) == 0);
	
	// get close to expiration
	for(int i=0; i<349; i++)
		Tick();
	RunTimerBackgroundTasks();
	CHECK(GetCallbackCalled(1) == 0);
	
	// expire again
	Tick();
	// Timer should execute in Tick context rather than background task
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetLatestCallbackID() == 65);
}

TEST(TimersTestGroup, TestTickWith2Timers_1pers1notpers)
{
	RegisterTimer(TestCallback1_param, 3, 350, FALSE, FALSE);
	RegisterTimer(TestCallback2_param, 104, 400, TRUE, FALSE);
	
	CHECK(AnyCallbacksCalled() == 0);
	
	for(int i=0; i<349; i++)
		Tick();
	RunTimerBackgroundTasks();
	CHECK(AnyCallbacksCalled() == 0);
	
	Tick();
	RunTimerBackgroundTasks();
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetLatestCallbackID() == 3);
	CHECK(GetCallbackCalled(2) == 0);
	
	// make sure #1 doesn't get called again
	SetCallbackCalled(1, 0);
	
	for(int i=0; i<49; i++)
		Tick();
	RunTimerBackgroundTasks();
	CHECK(AnyCallbacksCalled() == 0);
	
	Tick();
	RunTimerBackgroundTasks();
	CHECK(GetCallbackCalled(1) == 0);
	CHECK(GetCallbackCalled(2) == 1);
	CHECK(GetLatestCallbackID() == 104);
	
	// make sure #2 DOES get called again
	SetCallbackCalled(2, 0);
	
	for(int i=0; i<399; i++)
		Tick();
	RunTimerBackgroundTasks();
	CHECK(AnyCallbacksCalled() == 0);
	
	Tick();
	RunTimerBackgroundTasks();
	CHECK(GetCallbackCalled(1) == 0);
	CHECK(GetCallbackCalled(2) == 1);
	CHECK(GetLatestCallbackID() == 104);
}

TEST(TimersTestGroup, TestTooManyTimers)
{
	// Should not crash if we can't add the timer
	for(int i=0; i<20; i++)
	{
		RegisterTimer(TestCallback1_param, 0, 190, FALSE, FALSE);
		RegisterTimer(TestCallback1_param, 1, 768, FALSE, TRUE);
		RegisterTimer(TestCallback1_param, 2, 47,  TRUE,  FALSE);
		RegisterTimer(TestCallback1_param, 3, 558, TRUE,  TRUE);
	}
	
	CHECK(1);
}

TEST(TimersTestGroup, TestSimpleTimeout)
{
	bool flag = FALSE;
	RegisterTimeout(&flag, 55);
	
	int i=0;
	
	while (!flag && i < 60)
	{
		i++;
		Tick();
	}
	
	CHECK(i == 55);
}

TEST(TimersTestGroup, TestTimeoutBadParam)
{
	RegisterTimeout(0, 5);
	
	for(int i=0; i<10; i++)
	{ Tick(); }
	
	//just check that there is no crash when it would have expired
	CHECK(1);
}

TEST(TimersTestGroup, TestMixedTimeouts)
{
	bool flag1 = FALSE;
	bool flag2 = FALSE;
	bool flag3 = FALSE;
	RegisterTimeout(&flag1, 12);
	RegisterTimeout(&flag2, 36);
	RegisterTimeout(&flag3, 25);
	
	int i1=0;
	int i2=0;
	int i3=0;
	
	for(int i=0; i<40; i++)
	{
		if (!flag1)
			i1++;
		if (!flag2)
			i2++;
		if (!flag3)
			i3++;
		
		Tick();
	}
	
	CHECK(i1 == 12);
	CHECK(i2 == 36);
	CHECK(i3 == 25);
}

TEST(TimersTestGroup, TestTimeoutOverwrite)
{
	bool flag = FALSE;
	RegisterTimeout(&flag, 55);
	
	for(int i=0; i<10; i++)
	{ Tick(); }
	
	//mid-timeout change the wait time
	CHECK(!flag);
	RegisterTimeout(&flag, 130);
	CHECK(!flag);
	
	//verify it waits the full new wait
	for(int i=0; i<130; i++)
	{
		CHECK(!flag);
		Tick();
	}
	
	CHECK(flag);
}

TEST(TimersTestGroup, TestTimeoutCancel)
{
	bool* flag = (bool*)malloc(1);
	*flag = FALSE;
	RegisterTimeout(flag, 55);
	
	for(int i=0; i<10; i++)
	{ Tick(); }
	CHECK(!*flag);
	
	//cancel before it completes
	CancelTimeout(flag);
	CHECK(!*flag);
	free(flag);
	
	for(int i=0; i<60; i++)
	{ Tick(); }
	
	//just check that there is no crash when it would have expired
	CHECK(1);
}