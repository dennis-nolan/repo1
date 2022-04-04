extern "C"
{
#include "RawCircularBufferHeader.h"
}

#include "CppUTest/TestHarness.h"

TEST_GROUP(RawCircularBufferTestGroup)
{
	void setup() {}
	void teardown() {}
};

TEST(RawCircularBufferTestGroup, TestBufferInitDeinit)
{
	uint8_t actualbuf1[293];
	RawCircularBuffer testBuf1;
	RawCircularBufferInit(&testBuf1, actualbuf1, 1, 293);
	
	uint32_t actualbuf2[67];
	RawCircularBuffer testBuf2;
	RawCircularBufferInit(&testBuf2, actualbuf2, 4, 67);
	
	CHECK(1);
}

TEST(RawCircularBufferTestGroup, TestReadEmpty)
{
	uint32_t actualbuf1[12];
	RawCircularBuffer testBuf1;
	RawCircularBufferInit(&testBuf1, actualbuf1, 4, 12);
	
	uint32_t item = 0;
	CHECK(RawCircularBufferRead(&testBuf1, 1, &item, TRUE) == 0);
	CHECK(item == 0);
}

TEST(RawCircularBufferTestGroup, TestWriteReadRead)
{
	uint16_t actualbuf1[27];
	RawCircularBuffer testBuf1;
	RawCircularBufferInit(&testBuf1, actualbuf1, 2, 27);
	
	uint16_t testItem1 = 0xA82E;
	uint16_t readItem = 0;
	
	//successful write
	CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem1) == 1);
	CHECK(testBuf1.count == 1);
	
	//peek at the value
	CHECK(RawCircularBufferPeek(&testBuf1, 0, &readItem));
	CHECK(readItem == testItem1);
	
	//successful read
	readItem = 0;
	CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, 1) == 1);
	CHECK(readItem == testItem1);
	CHECK(testBuf1.count == 0);
	
	//failed read, buffer empty
	readItem = 0;
	CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, TRUE) == 0);
	CHECK(readItem == 0);
}

TEST(RawCircularBufferTestGroup, TestWriteFull)
{
	uint32_t actualbuf1[2];
	RawCircularBuffer testBuf1;
	RawCircularBufferInit(&testBuf1, actualbuf1, 4, 2);
	
	uint32_t testItem1 = 0x1C056149;
	uint32_t testItem2 = 0x12345678;
	uint32_t testItem3 = 0xADADCBCB;
	uint32_t readItem = 0;
	
	//successful write
	CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem1) == 1);
	CHECK(testBuf1.count == 1);
	
	//successful write
	CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem2) == 1);
	CHECK(testBuf1.count == 2);
	
	//failed write, full
	CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem3) == 0);
	
	//peek at the values
	CHECK(RawCircularBufferPeek(&testBuf1, 0, &readItem));
	CHECK(readItem == testItem1);
	CHECK(RawCircularBufferPeek(&testBuf1, 1, &readItem));
	CHECK(readItem == testItem2);
}

TEST(RawCircularBufferTestGroup, TestFailureRecovery)
{
	uint32_t actualbuf1[3];
	RawCircularBuffer testBuf1;
	RawCircularBufferInit(&testBuf1, actualbuf1, 4, 3);
	
	uint32_t testItem1 = 0xABCDEFFF;
	uint32_t testItem2 = 0x86521654;
	uint32_t testItem3 = 0x23553211;
	uint32_t testItem4 = 0xCDCDCDCD;
	uint32_t readItem = 0;
	
	//successful write
	CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem1) == 1);
	CHECK(testBuf1.count == 1);
	
	//successful write
	CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem2) == 1);
	CHECK(testBuf1.count == 2);
	
	//successful write
	CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem3) == 1);
	CHECK(testBuf1.count == 3);
	
	//failed write, full
	CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem4) == 0);
	CHECK(testBuf1.count == 3);
	
	//successful read
	CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, 1) == 1);
	CHECK(readItem == testItem1);
	CHECK(testBuf1.count == 2);
	
	//successful write
	CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem4) == 1);
	CHECK(testBuf1.count == 3);
	
	//successful reads
	CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, 1) == 1);
	CHECK(readItem == testItem2);
	CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, 1) == 1);
	CHECK(readItem == testItem3);
	CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, 1) == 1);
	CHECK(readItem == testItem4);
	CHECK(testBuf1.count == 0);
}

TEST(RawCircularBufferTestGroup, TestWriteReadLoop)
{
	uint32_t actualbuf1[9];
	RawCircularBuffer testBuf1;
	RawCircularBufferInit(&testBuf1, actualbuf1, 4, 9);
	
	uint32_t testItem1 = 0x99346571;
	uint32_t testItem2 = 0x21215470;
	uint32_t testItem3 = 0xACBCEFDD;
	uint32_t testItem4 = 0x12CDEF34;
	uint32_t readItem = 0;
	
	//scoot through entire buffer multiple times
	for(uint8_t i=0; i<17; i++)
	{
		//successful write
		CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem1) == 1);
		CHECK(testBuf1.count == 1);
		
		//successful write
		CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem2) == 1);
		CHECK(testBuf1.count == 2);
		
		//successful read
		CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, 1) == 1);
		CHECK(readItem == testItem1);
		CHECK(testBuf1.count == 1);
		
		//peek at the value
		CHECK(RawCircularBufferPeek(&testBuf1, 0, &readItem));
		CHECK(readItem == testItem2);
		
		//successful write
		CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem3) == 1);
		CHECK(testBuf1.count == 2);
		
		//peek at the value
		CHECK(RawCircularBufferPeek(&testBuf1, 1, &readItem));
		CHECK(readItem == testItem3);
		
		//successful read
		CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, 1) == 1);
		CHECK(readItem == testItem2);
		CHECK(testBuf1.count == 1);
		
		//successful read
		CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, 1) == 1);
		CHECK(readItem == testItem3);
		CHECK(testBuf1.count == 0);
	}
	
	//failed read, buffer empty
	readItem = 0;
	CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, TRUE) == 0);
	CHECK(readItem == 0);
	
	//successful write
	CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem4) == 1);
	CHECK(testBuf1.count == 1);
	
	//peek at the value
	CHECK(RawCircularBufferPeek(&testBuf1, 0, &readItem));
	CHECK(readItem == testItem4);
	
	//successful read
	CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, 1) == 1);
	CHECK(readItem == testItem4);
	CHECK(testBuf1.count == 0);
}

TEST(RawCircularBufferTestGroup, TestWriteReadLoop_WeirdSize)
{
	uint8_t actualbuf1[25], j;
	RawCircularBuffer testBuf1;
	RawCircularBufferInit(&testBuf1, actualbuf1, 5, 5);
	
	uint8_t testItem1[5] = {1,2,3,4,5};
	uint8_t testItem2[5] = {88,71,63,47,59};
	uint8_t readItem[5] = {0};
	
	//scoot through entire buffer multiple times
	for(uint8_t i=0; i<27; i++)
	{
		//successful write
		CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem1) == 1);
		CHECK(testBuf1.count == 1);
		
		//peek at the value
		CHECK(RawCircularBufferPeek(&testBuf1, 0, &readItem));
		for (j=0; j<5; j++)
		{ CHECK(readItem[j] == testItem1[j]); }
		CHECK(testBuf1.count == 1);
		
		//successful read
		CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, 1) == 1);
		for (j=0; j<5; j++)
		{ CHECK(readItem[j] == testItem1[j]); }
		CHECK(testBuf1.count == 0);
	}
	
	//failed read, buffer empty
	for (uint8_t i=0; i<5; i++)
	{ readItem[i] = 0; }
	CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, TRUE) == 0);
	for (uint8_t i=0; i<5; i++)
	{ CHECK(readItem[i] == 0); }
	
	//successful write
	CHECK(RawCircularBufferWrite(&testBuf1, 1, &testItem2) == 1);
	CHECK(testBuf1.count == 1);
	
	//peek at the value
	CHECK(RawCircularBufferPeek(&testBuf1, 0, &readItem));
	for (uint8_t i=0; i<5; i++)
	{ CHECK(readItem[i] == testItem2[i]); }
	
	//successful read
	CHECK(RawCircularBufferRead(&testBuf1, 1, &readItem, 1) == 1);
	for (uint8_t i=0; i<5; i++)
	{ CHECK(readItem[i] == testItem2[i]); }
	CHECK(testBuf1.count == 0);
}

TEST(RawCircularBufferTestGroup, TestWriteReadLoop_Array)
{
	uint8_t actualbuf1[29], j;
	RawCircularBuffer testBuf1;
	RawCircularBufferInit(&testBuf1, actualbuf1, 1, 29);
	
	uint8_t testItem1[11] = {88,71,63,47,59,11,36,94,23,87,33};
	uint8_t readItem[11] = {0};
	
	//scoot through entire buffer multiple times
	for(uint8_t i=0; i<27; i++)
	{
		//successful write
		CHECK(RawCircularBufferWrite(&testBuf1, 11, &testItem1) == 1);
		CHECK(testBuf1.count == 11);
		
		//successful peeking
		for (j=0; j<11; j++)
		{
			CHECK(RawCircularBufferPeek(&testBuf1, j, &readItem));
			CHECK(readItem[0] == testItem1[j]);
		}
		
		//successful read
		CHECK(RawCircularBufferRead(&testBuf1, 11, &readItem, 1) == 11);
		for (j=0; j<11; j++)
		{ CHECK(readItem[j] == testItem1[j]); }
		CHECK(testBuf1.count == 0);
	}
	
	//failed read, buffer empty
	for (uint8_t i=0; i<11; i++)
	{ readItem[i] = 0; }
	CHECK(RawCircularBufferRead(&testBuf1, 11, &readItem, TRUE) == 0);
	for (uint8_t i=0; i<11; i++)
	{ CHECK(readItem[i] == 0); }
	
	//successful write
	CHECK(RawCircularBufferWrite(&testBuf1, 11, &testItem1) == 1);
	CHECK(testBuf1.count == 11);
	
	//successful read
	CHECK(RawCircularBufferRead(&testBuf1, 11, &readItem, 1) == 11);
	for (uint8_t i=0; i<11; i++)
	{ CHECK(readItem[i] == testItem1[i]); }
	CHECK(testBuf1.count == 0);
}
