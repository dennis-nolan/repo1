extern "C"
{
#include "MemoryMgmtHeader.h"
}

#include "CppUTest/TestHarness.h"

TEST_GROUP(MemoryTestGroup)
{
	void setup()
	{
		MemInit();
	}
	
	void teardown() {}
};

TEST(MemoryTestGroup, TestSimpleGetFree)
{
	void* data1 = GetMem(5);
	CHECK(data1 != 0)
	CHECK(GetMemUsed() == 5);
	
	void* data2 = GetMem(12);
	CHECK(data2 != 0)
	CHECK(GetMemUsed() == 17);
	
	FreeMem(&data1);
	CHECK(data1 == 0)
	CHECK(GetMemUsed() == 12);
	
	FreeMem(&data2);
	CHECK(data2 == 0)
	CHECK(GetMemUsed() == 0);
}

TEST(MemoryTestGroup, TestComplexGetFree)
{
	void* data1 = GetMem(16);
	CHECK(data1 != 0)
	CHECK(GetMemUsed() == 16);
	
	void* data2 = GetMem(2);
	CHECK(data2 != 0)
	CHECK(GetMemUsed() == 18);
	
	FreeMem(&data1);
	CHECK(data1 == 0)
	CHECK(GetMemUsed() == 2);
	
	void* data3 = GetMem(57);
	CHECK(data3 != 0)
	CHECK(GetMemUsed() == 59);
	
	FreeMem(&data2);
	CHECK(data2 == 0)
	CHECK(GetMemUsed() == 57);
	
	FreeMem(&data3);
	CHECK(data3 == 0)
	CHECK(GetMemUsed() == 0);
}
