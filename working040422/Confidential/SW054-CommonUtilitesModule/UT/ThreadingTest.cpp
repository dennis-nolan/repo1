extern "C"
{
#include "ThreadingHeader.h"
#include "CallbackTestHelper.h"
}

#include "CppUTest/TestHarness.h"

TEST_GROUP(ThreadingTestGroup)
{
	void setup()
	{
		ResetCallbackCalled();
		ThreadingInit();
	}
	
	void teardown() {}
};

TEST(ThreadingTestGroup, TestThreadingInit)
{
	CHECK(1);
}

TEST(ThreadingTestGroup, TestNoThreads)
{
	RunAllThreads();
	
	for(int i=0; i<80; i++)
		RunNextThread();
	
	CHECK(1);
}

TEST(ThreadingTestGroup, Test1Thread)
{
	CHECK(AnyCallbacksCalled() == 0);
	
	// Register
	RegisterThread(TestCallback1, 0x0F);
	
	CHECK(AnyCallbacksCalled() == 0);
	
	// Run everything
	RunAllThreads();
	
	// Check that it ran
	CHECK(GetCallbackCalled(1) == 1);
	
	// Test again using RunNextThread
	ResetCallbackCalled();
	RunNextThread();
	
	CHECK(GetCallbackCalled(1) == 1);
}

TEST(ThreadingTestGroup, Test2Threads_inorder)
{
	CHECK(AnyCallbacksCalled() == 0);
	
	// Register in priority order
	RegisterThread(TestCallback1, 0x0F);
	RegisterThread(TestCallback2, 0x10);
	
	CHECK(AnyCallbacksCalled() == 0);
	
	// Run everything
	RunAllThreads();
	
	// Check that they ran
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 1);
	
	// Test again using RunNextThread
	ResetCallbackCalled();
	RunNextThread();
	
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 0);
	
	RunNextThread();
	
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 1);
}

TEST(ThreadingTestGroup, Test2Threads_outoforder)
{
	CHECK(AnyCallbacksCalled() == 0);
	
	// Register in reverse of priority order
	RegisterThread(TestCallback2, 0x10);
	RegisterThread(TestCallback1, 0x0F);
	
	CHECK(AnyCallbacksCalled() == 0);
	
	// Run everything
	RunAllThreads();
	
	// Check that they ran
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 1);
	
	// Test again using RunNextThread
	ResetCallbackCalled();
	RunNextThread();
	
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 0);
	
	RunNextThread();
	
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 1);
}

TEST(ThreadingTestGroup, TestTooManyThreads)
{
	RegisterThread(TestCallback1, 0x00);
	RegisterThread(TestCallback3, 0xEE);
	
	for(unsigned char i=10; i<90; i++)
		RegisterThread(TestCallback2, i);
	
	// Run everything
	RunAllThreads();
	
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 1);
	CHECK(GetCallbackCalled(3) == 1);
	
	// Test again using RunNextThread
	ResetCallbackCalled();
	
	RunNextThread();
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 0);
	CHECK(GetCallbackCalled(3) == 0);
	
	RunNextThread();
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 1);
	CHECK(GetCallbackCalled(3) == 0);
	
	for(int i=2; i<15; i++)
		RunNextThread();
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 1);
	CHECK(GetCallbackCalled(3) == 0);
	
	RunNextThread();
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 1);
	CHECK(GetCallbackCalled(3) == 1);
}

TEST(ThreadingTestGroup, TestTooManyThreads_reversed)
{
	RegisterThread(TestCallback3, 0xEE);
	RegisterThread(TestCallback1, 0x00);
	
	for(unsigned char i=90; i>10; i--)
		RegisterThread(TestCallback2, i);
	
	// Run everything
	RunAllThreads();
	
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 1);
	CHECK(GetCallbackCalled(3) == 1);
	
	// Test again using RunNextThread
	ResetCallbackCalled();
	
	RunNextThread();
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 0);
	CHECK(GetCallbackCalled(3) == 0);
	
	RunNextThread();
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 1);
	CHECK(GetCallbackCalled(3) == 0);
	
	for(int i=2; i<15; i++)
		RunNextThread();
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 1);
	CHECK(GetCallbackCalled(3) == 0);
	
	RunNextThread();
	CHECK(GetCallbackCalled(1) == 1);
	CHECK(GetCallbackCalled(2) == 1);
	CHECK(GetCallbackCalled(3) == 1);
}