
int callbackCalled[3] = {0};
unsigned char latestCallbackID = 0;

void ResetCallbackCalled(void)
{
	for(int i=0; i<3; i++)
		callbackCalled[i] = 0;
	
	latestCallbackID = 0;
}

void SetCallbackCalled(int index, int val)
{
	callbackCalled[index-1] = val;
}

int GetCallbackCalled(int index)
{
	return callbackCalled[index-1];
}

unsigned char GetLatestCallbackID(void)
{
	return latestCallbackID;
}

int AnyCallbacksCalled(void)
{
	for(int i=0; i<3; i++)
	{
		if (callbackCalled[i] != 0)
			return 1;
	}
	
	return 0;
}

void TestCallback1(void)
{
	callbackCalled[0] = 1;
}

void TestCallback2(void)
{
	callbackCalled[1] = 1;
}

void TestCallback3(void)
{
	callbackCalled[2] = 1;
}

void TestCallback1_param(unsigned char ID)
{
	callbackCalled[0] = 1;
	latestCallbackID = ID;
}

void TestCallback2_param(unsigned char ID)
{
	callbackCalled[1] = 1;
	latestCallbackID = ID;
}

void TestCallback3_param(unsigned char ID)
{
	callbackCalled[2] = 1;
	latestCallbackID = ID;
}
