extern "C"
{
#include "MemoryMgmtHeader.h"
#include "CM2DiagnosticsHeader.h"
}

#include "CppUTest/TestHarness.h"

TEST_GROUP(DiagnosticsTestGroup)
{
	void setup() {}
	void teardown() {}
};

TEST(DiagnosticsTestGroup, TestDiagInitDeinit)
{
	DiagnosticsInit(3);
	DiagnosticsDeinit();
	CHECK(1);
}

TEST(DiagnosticsTestGroup, TestDiagSingleWrite)
{
	DiagnosticsInit(9);
	WriteDiagnosticLog(907, SEVERITY_DEBUG, 15, FALSE, (uint8_t*)"THIS IS A TEST!", FALSE);
	DiagnosticsDeinit();
}

TEST(DiagnosticsTestGroup, TestDiagReadWriteWriteRead)
{
	DiagnosticsInit(3);
	
	uint8_t** logs = 0;
	uint8_t numLogs = 0;
	ReadDiagnosticLogs(DIAG_STORAGE_RAM, &logs, &numLogs);
	CHECK(numLogs == 0);
	CHECK(logs == 0)
	
	WriteDiagnosticLog(907, SEVERITY_DEBUG, 15, FALSE, (uint8_t*)"THIS IS A TEST!", FALSE);
	WriteDiagnosticLog(598, SEVERITY_WARNING, 10, FALSE, (uint8_t*)"warning 36", FALSE);
	CHECK(1);
	
	ReadDiagnosticLogs(DIAG_STORAGE_RAM, &logs, &numLogs);
	CHECK(numLogs == 2);
	CHECK(logs != 0);
	CHECK(logs[0][0] == 907 >> 2);
	CHECK(logs[0][17] == '!');
	CHECK(logs[1][0] == 598 >> 2);
	CHECK(logs[1][12] == '6');
	FreeMem((void**)&logs);
	
	DiagnosticsDeinit();
}

TEST(DiagnosticsTestGroup, TestDiagReadWriteIncrementRead)
{
	DiagnosticsInit(3);
	
	uint8_t** logs = 0;
	uint8_t numLogs = 0;
	ReadDiagnosticLogs(DIAG_STORAGE_RAM, &logs, &numLogs);
	CHECK(numLogs == 0);
	CHECK(logs == 0)
	
	WriteDiagnosticLog(907, SEVERITY_DEBUG, 15, FALSE, (uint8_t*)"THIS IS A TEST!", FALSE);
	WriteDiagnosticLog(907, SEVERITY_DEBUG, 15, FALSE, (uint8_t*)"THIS IS A TEST!", FALSE);
	WriteDiagnosticLog(907, SEVERITY_DEBUG, 15, FALSE, (uint8_t*)"THIS IS A TEST!", FALSE);
	CHECK(1);
	
	ReadDiagnosticLogs(DIAG_STORAGE_RAM, &logs, &numLogs);
	CHECK(numLogs == 1);
	CHECK(logs != 0);
	CHECK(logs[0][2] == 2); // #occ-1
	FreeMem((void**)&logs);
	
	DiagnosticsDeinit();
}


TEST(DiagnosticsTestGroup, TestDiagOverwrite)
{
	DiagnosticsInit(2);
	
	uint8_t** logs = 0;
	uint8_t numLogs = 0;
	ReadDiagnosticLogs(DIAG_STORAGE_RAM, &logs, &numLogs);
	CHECK(numLogs == 0);
	CHECK(logs == 0);
	
	WriteDiagnosticLog(907, SEVERITY_DEBUG, 15, FALSE, (uint8_t*)"THIS IS A TEST!", FALSE);
	WriteDiagnosticLog(598, SEVERITY_WARNING, 10, FALSE, (uint8_t*)"warning 36", FALSE);
	WriteDiagnosticLog(206, SEVERITY_RECOVERABLE_ERROR, 17, FALSE, (uint8_t*)"restarting spirit", FALSE);
	WriteDiagnosticLog(191, SEVERITY_FATAL_ERROR, 21, FALSE, (uint8_t*)"SW crash. WD timeout.", FALSE);
	CHECK(1);
	
	ReadDiagnosticLogs(DIAG_STORAGE_RAM, &logs, &numLogs);
	CHECK(numLogs == 2);
	CHECK(logs != 0);
	//////////////////////TODO: check contents
	FreeMem((void**)&logs);
	
	DiagnosticsDeinit();
}
