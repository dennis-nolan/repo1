
void ResetCallbackCalled(void);
void SetCallbackCalled(int index, int val);
int GetCallbackCalled(int index);
unsigned char GetLatestCallbackID(void);
int AnyCallbacksCalled(void);
void TestCallback1(void);
void TestCallback2(void);
void TestCallback3(void);
void TestCallback1_param(uint8_t timerID);
void TestCallback2_param(uint8_t timerID);
void TestCallback3_param(uint8_t timerID);