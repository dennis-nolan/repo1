extern "C"
{
#include "MemoryMgmtHeader.h"
#include "CircularBufferHeader.h"
}

#include "CppUTest/TestHarness.h"

TEST_GROUP(CircularBufferTestGroup)
{
	void setup() {}
	void teardown() {}
};

TEST(CircularBufferTestGroup, TestBufferInitDeinit)
{
	CircularBuffer* testBuf1;
	CircularBuffer* testBuf2;
	
	testBuf1 = BufferInit(16);
	testBuf2 = BufferInit(21);
	BufferDeinit(testBuf1, FALSE);
	BufferDeinit(testBuf2, FALSE);
	
	CHECK(1);
}

TEST(CircularBufferTestGroup, TestReadEmpty)
{
	CircularBuffer* testBuf1 = BufferInit(9);
	void* item;
	
	CHECK(BufferRead(testBuf1, &item) == 0);
	CHECK(item == 0);
	
	BufferDeinit(testBuf1, FALSE);
}

TEST(CircularBufferTestGroup, TestWriteReadRead)
{
	CircularBuffer* testBuf1 = BufferInit(112);
	uint8_t testItem1 = 0xA8;
	uint8_t* readItem = 0;
	
	//successful write
	CHECK(BufferWrite(testBuf1, &testItem1) == 1);
	CHECK(GetNumItems(testBuf1) == 1);
	
	// Do an offset read
	CHECK(BufferReadOffset(testBuf1, (void**)&readItem, 0));
	CHECK(*readItem == testItem1);
	
	//successful read
	CHECK(BufferRead(testBuf1, (void**)&readItem) == 1);
	CHECK(*readItem == testItem1);
	CHECK(GetNumItems(testBuf1) == 0);
	
	//failed read, buffer empty
	CHECK(BufferRead(testBuf1, (void**)&readItem) == 0);
	CHECK(readItem == 0);
	CHECK(BufferReadOffset(testBuf1, (void**)&readItem, 0) == FALSE);
	
	BufferDeinit(testBuf1, FALSE);
}

TEST(CircularBufferTestGroup, TestWriteFull)
{
	CircularBuffer* testBuf1 = BufferInit(2);
	void* item;
	
	//successful write
	CHECK(BufferWrite(testBuf1, &item) == 1);
	CHECK(GetNumItems(testBuf1) == 1);
	
	//successful write
	CHECK(BufferWrite(testBuf1, &item) == 1);
	CHECK(GetNumItems(testBuf1) == 2);
	
	//failed write, full
	CHECK(BufferWrite(testBuf1, &item) == 0);
	
	BufferDeinit(testBuf1, FALSE);
}

TEST(CircularBufferTestGroup, TestFailureRecovery)
{
	CircularBuffer* testBuf1 = BufferInit(3);
	uint8_t testItem1 = 0xA8;
	uint8_t testItem2 = 0xB2;
	uint8_t testItem3 = 0x11;
	uint8_t testItem4 = 0x9F;
	uint8_t* readItem = 0;
	
	//failed read, buffer empty
	CHECK(BufferRead(testBuf1, (void**)&readItem) == 0);
	CHECK(readItem == 0);
	CHECK(GetNumItems(testBuf1) == 0);
	
	//successful writes
	CHECK(BufferWrite(testBuf1, &testItem1) == 1);
	CHECK(BufferWrite(testBuf1, &testItem2) == 1);
	CHECK(BufferWrite(testBuf1, &testItem3) == 1);
	CHECK(GetNumItems(testBuf1) == 3);
	
	//failed write, full
	CHECK(BufferWrite(testBuf1, &testItem1) == 0);
	
	//successful read
	CHECK(BufferRead(testBuf1, (void**)&readItem) == 1);
	CHECK(*readItem == testItem1);
	CHECK(GetNumItems(testBuf1) == 2);
	
	//successful write
	CHECK(BufferWrite(testBuf1, &testItem4) == 1);
	CHECK(GetNumItems(testBuf1) == 3);
	
	//successful reads
	CHECK(BufferRead(testBuf1, (void**)&readItem) == 1);
	CHECK(*readItem == testItem2);
	CHECK(BufferRead(testBuf1, (void**)&readItem) == 1);
	CHECK(*readItem == testItem3);
	CHECK(BufferRead(testBuf1, (void**)&readItem) == 1);
	CHECK(*readItem == testItem4);
	CHECK(GetNumItems(testBuf1) == 0);
	
	BufferDeinit(testBuf1, FALSE);
}

TEST(CircularBufferTestGroup, TestWriteReadLoop)
{
	CircularBuffer* testBuf1 = BufferInit(5);
	uint8_t testItem1 = 0xA8;
	uint8_t testItem2 = 0x67;
	uint8_t* readItem = 0;
	
	//scoot through entire buffer multiple times
	for(uint8_t i=0; i<32; i++)
	{
		//successful write
		CHECK(BufferWrite(testBuf1, &testItem1) == 1);
		CHECK(GetNumItems(testBuf1) == 1);
		
		//successful read
		CHECK(BufferRead(testBuf1, (void**)&readItem) == 1);
		CHECK(*readItem == testItem1);
		CHECK(GetNumItems(testBuf1) == 0);
	}
	
	//failed read, buffer empty
	CHECK(BufferRead(testBuf1, (void**)&readItem) == 0);
	CHECK(readItem == 0);
	
	//successful write
	CHECK(BufferWrite(testBuf1, &testItem2) == 1);
	CHECK(GetNumItems(testBuf1) == 1);
	
	//successful read
	CHECK(BufferRead(testBuf1, (void**)&readItem) == 1);
	CHECK(*readItem == testItem2);
	CHECK(GetNumItems(testBuf1) == 0);
	
	BufferDeinit(testBuf1, FALSE);
}

TEST(CircularBufferTestGroup, TestWriteReadLoop_Pointers)
{
	CircularBuffer* testBuf1 = BufferInit(5);
	uint8_t* testItem1 = (uint8_t*)GetMem(3);
	testItem1[0] = 0xFD;
	testItem1[1] = 0x23;
	testItem1[2] = 0x4C;
	uint8_t* testItem2 = (uint8_t*)GetMem(2);
	testItem2[0] = 0x98;
	testItem2[1] = 0x71;
	uint8_t* readItem = 0;
	
	CHECK(BufferReadOffset(testBuf1, (void**)&readItem, 0) == FALSE);
	
	//scoot through entire buffer multiple times
	for(uint8_t i=0; i<32; i++)
	{
		//successful write
		CHECK(BufferWrite(testBuf1, testItem1) == 1);
		CHECK(GetNumItems(testBuf1) == 1);
		CHECK(BufferReadOffset(testBuf1, (void**)&readItem, 0) == TRUE);
		
		//successful read
		CHECK(BufferRead(testBuf1, (void**)&readItem) == 1);
		CHECK(readItem == testItem1);
		CHECK((readItem)[0] == testItem1[0]);
		CHECK((readItem)[1] == testItem1[1]);
		CHECK((readItem)[2] == testItem1[2]);
		CHECK(GetNumItems(testBuf1) == 0);
	}
	
	//failed read, buffer empty
	CHECK(BufferRead(testBuf1, (void**)&readItem) == 0);
	CHECK(readItem == 0);
	
	//successful write
	CHECK(BufferWrite(testBuf1, testItem2) == 1);
	
	//successful read
	CHECK(BufferRead(testBuf1, (void**)&readItem) == 1);
	CHECK(readItem == testItem2);
	CHECK((readItem)[0] == testItem2[0]);
	CHECK((readItem)[1] == testItem2[1]);
	
	//write items back
	CHECK(BufferWrite(testBuf1, testItem2) == 1);
	CHECK(BufferWrite(testBuf1, testItem1) == 1);
	CHECK(GetNumItems(testBuf1) == 2);
	
	//make sure the items get cleaned up
	BufferDeinit(testBuf1, TRUE);
}