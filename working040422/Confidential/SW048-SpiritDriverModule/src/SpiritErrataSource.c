
#include "SpiritErrataHeader.h"
#include "MCU_Interface.h"
#include "SPIRIT_Commands.h"
#include "JL_SpiritDriver.h"

#define XTAL_FREQ			50000000 //Hz

#define HIGH_BAND			0x00  /*!< High_Band selected: from 779 MHz to 915 MHz */
#define MIDDLE_BAND			0x01  /*!< Middle Band selected: from 387 MHz to 470 MHz */
#define LOW_BAND			0x02  /*!< Low Band selected: from 300 MHz to 348 MHz */
#define VERY_LOW_BAND		0x03  /*!< Vary low Band selected: from 150 MHz to 174 MHz */

#define HIGH_BAND_FACTOR      6       /*!< Band select factor for high band. Factor B in the equation 2 */
#define MIDDLE_BAND_FACTOR    12      /*!< Band select factor for middle band. Factor B in the equation 2 */
#define LOW_BAND_FACTOR       16      /*!< Band select factor for low band. Factor B in the equation 2 */
#define VERY_LOW_BAND_FACTOR  32      /*!< Band select factor for very low band. Factor B in the equation 2 */

#define HIGH_BAND_LOWER_LIMIT         778000000   /*!< Lower limit of the high band: 779 MHz */
#define HIGH_BAND_UPPER_LIMIT         957100000   /*!< Upper limit of the high band: 956 MHz */
#define MIDDLE_BAND_LOWER_LIMIT       386000000   /*!< Lower limit of the middle band: 387 MHz */
#define MIDDLE_BAND_UPPER_LIMIT       471100000   /*!< Upper limit of the middle band: 470 MHz */
#define LOW_BAND_LOWER_LIMIT          299000000   /*!< Lower limit of the low band: 300 MHz */
#define LOW_BAND_UPPER_LIMIT          349100000   /*!< Upper limit of the low band: 348 MHz */
#define VERY_LOW_BAND_LOWER_LIMIT     149000000   /*!< Lower limit of the very low band: 150 MHz */
#define VERY_LOW_BAND_UPPER_LIMIT     175100000   /*!< Upper limit of the very low band: 174 MHz */

#define IS_FREQUENCY_BAND_HIGH(FREQUENCY) ((FREQUENCY)>=HIGH_BAND_LOWER_LIMIT && \
                                           (FREQUENCY)<=HIGH_BAND_UPPER_LIMIT)
#define IS_FREQUENCY_BAND_MIDDLE(FREQUENCY) ((FREQUENCY)>=MIDDLE_BAND_LOWER_LIMIT && \
                                             (FREQUENCY)<=MIDDLE_BAND_UPPER_LIMIT)
#define IS_FREQUENCY_BAND_LOW(FREQUENCY) ((FREQUENCY)>=LOW_BAND_LOWER_LIMIT && \
                                          (FREQUENCY)<=LOW_BAND_UPPER_LIMIT)
#define IS_FREQUENCY_BAND_VERY_LOW(FREQUENCY) ((FREQUENCY)>=VERY_LOW_BAND_LOWER_LIMIT && \
                                          (FREQUENCY)<=VERY_LOW_BAND_UPPER_LIMIT)
#define IS_FREQUENCY_BAND(FREQUENCY) (IS_FREQUENCY_BAND_HIGH(FREQUENCY)|| \
                                      IS_FREQUENCY_BAND_MIDDLE(FREQUENCY)|| \
                                      IS_FREQUENCY_BAND_LOW(FREQUENCY)|| \
                                      IS_FREQUENCY_BAND_VERY_LOW(FREQUENCY))

#define FBASE_DIVIDER		262144 /*!< 2^18 factor dividing fxo in fbase formula */
#define CHSPACE_DIVIDER		32768  /*!< 2^15 factor dividing fxo in channel space formula */

#define VCO_L 0x00	    /*!< VCO lower */
#define VCO_H 0x01		/*!< VCO higher */

#if 0
static const uint8_t s_vectcBHalfFactor[4]={(HIGH_BAND_FACTOR/2), (MIDDLE_BAND_FACTOR/2), (LOW_BAND_FACTOR/2), (VERY_LOW_BAND_FACTOR/2)};
static const uint8_t s_vectcBandRegValue[4]={SYNT0_BS_6, SYNT0_BS_12, SYNT0_BS_16, SYNT0_BS_32};
static const uint16_t s_vectnVCOFreq[16]={4644, 4708, 4772, 4836, 4902, 4966, 5030, 5095, 5161, 5232, 5303, 5375, 5448, 5519, 5592, 5663};


static uint32_t round(double input)
{
	if (input - (uint32_t)input >= 0.5)
		return (uint32_t)input + 1;
	return (uint32_t)input;
}

static uint32_t SpiritRadioGetFrequencyBase(CM2PortType portType)
{
	uint32_t synthWord;
	uint8_t band;

	// Reads the synth word
	uint8_t regArray[4];
	SpiReadRegisters(portType, SYNT3_BASE, 4, regArray);
	synthWord = ((((uint32_t)(regArray[0]&0x1F))<<21)+(((uint32_t)(regArray[1]))<<13)+(((uint32_t)(regArray[2]))<<5)+(((uint32_t)(regArray[3]))>>3));

	// Reads the operating band
	uint8_t tempRegValue;
	SpiReadRegisters(portType, SYNT0_BASE, 1, &tempRegValue);
	if((tempRegValue & 0x07) == SYNT0_BS_6)
		band = HIGH_BAND;
	else if ((tempRegValue & 0x07) == SYNT0_BS_12)
		band = MIDDLE_BAND;
	else if ((tempRegValue & 0x07) == SYNT0_BS_16)
		band = LOW_BAND;
	else
		band = VERY_LOW_BAND;

	// Read the reference divider
	SpiReadRegisters(portType, SYNTH_CONFIG1_BASE, 1, &tempRegValue);
	uint8_t cRefDiv = ((tempRegValue>>7)&0x1) + 1;

	/* Calculates the frequency base and return it */
	return (uint32_t)round(synthWord*(((double)XTAL_FREQ)/(FBASE_DIVIDER*cRefDiv*s_vectcBHalfFactor[band])));
}

static int32_t SpiritRadioGetFrequencyOffset(CM2PortType portType)
{
	uint8_t tempArray[2];
	int16_t xtalOffsetFactor;

	/* Reads the FC_OFFSET registers */
	SpiReadRegisters(portType, FC_OFFSET1_BASE, 2, tempArray);

	/* Calculates the Offset Factor */
	uint16_t xtalOffTemp = ((((uint16_t)tempArray[0])<<8)+((uint16_t)tempArray[1]));

	if(xtalOffTemp & 0x0800)
		xtalOffTemp = xtalOffTemp | 0xF000;
	else
		xtalOffTemp = xtalOffTemp & 0x0FFF;

	xtalOffsetFactor = *((int16_t*)(&xtalOffTemp));

	/* Calculates the frequency offset and return it */
	return ((int32_t)(xtalOffsetFactor*XTAL_FREQ)/FBASE_DIVIDER);
}

static void SpiritCalibrationSelectVco(CM2PortType portType, uint8_t xVco)
{
  uint8_t tempRegValue;
  SpiReadRegisters(portType, SYNTH_CONFIG1_BASE, 1, &tempRegValue);
  tempRegValue &= 0xF9;
  
  if(xVco == VCO_H)
    tempRegValue |= 0x02;
  else
    tempRegValue |= 0x04;
  
  SpiWriteRegisters(portType, SYNTH_CONFIG1_BASE, 1, &tempRegValue);  
}

static uint8_t SpiritRadioSearchWCP(uint32_t lFc)
{
  int8_t i;
  uint32_t vcofreq;
  uint8_t BFactor;
  
  /* Search the operating band */
  if(IS_FREQUENCY_BAND_HIGH(lFc))
  {
    BFactor = HIGH_BAND_FACTOR;
  }
  else if(IS_FREQUENCY_BAND_MIDDLE(lFc))
  {
    BFactor = MIDDLE_BAND_FACTOR;
  }
  else if(IS_FREQUENCY_BAND_LOW(lFc))
  {
    BFactor = LOW_BAND_FACTOR;
  }
  else if(IS_FREQUENCY_BAND_VERY_LOW(lFc))
  {
    BFactor = VERY_LOW_BAND_FACTOR;
  }
  
  /* Calculates the VCO frequency VCOFreq = lFc*B */
  vcofreq = (lFc/1000000)*BFactor;
  
  /* Search in the vco frequency array the charge pump word */
  if(vcofreq>=s_vectnVCOFreq[15])
  {
    i=15;
  }
  else
  {
    /* Search the value */
    for(i=0 ; i<15 && vcofreq>s_vectnVCOFreq[i] ; i++);
    
    /* Be sure that it is the best approssimation */
    if (i!=0 && s_vectnVCOFreq[i]-vcofreq>vcofreq-s_vectnVCOFreq[i-1])
      i--;
  }
  
  /* Return index */
  return (i%8);
}

static void SpiritManagementSetFrequencyBase(CM2PortType portType, uint32_t lFBase)
{
	uint32_t synthWord, Fc;
	uint8_t band, anaRadioRegArray[4], wcp;

	/* Search the operating band */
	if(IS_FREQUENCY_BAND_HIGH(lFBase))
		band = HIGH_BAND;
	else if(IS_FREQUENCY_BAND_MIDDLE(lFBase))
		band = MIDDLE_BAND;
	else if(IS_FREQUENCY_BAND_LOW(lFBase))
		band = LOW_BAND;
	else if(IS_FREQUENCY_BAND_VERY_LOW(lFBase))
		band = VERY_LOW_BAND;

	int32_t FOffset  = SpiritRadioGetFrequencyOffset(portType);
	
	//uint32_t lChannelSpace  = SpiritRadioGetChannelSpace();
	uint8_t channelSpaceFactor;
	SpiReadRegisters(portType, CHSPACE_BASE, 1, &channelSpaceFactor);
	uint32_t lChannelSpace = ((channelSpaceFactor*XTAL_FREQ)/CHSPACE_DIVIDER);
	
	//uint8_t cChannelNum = SpiritRadioGetChannel();
	uint8_t cChannelNum;
	SpiReadRegisters(portType, CHNUM_BASE, 1, &cChannelNum);

	/* Calculates the channel center frequency */
	Fc = lFBase + FOffset + lChannelSpace*cChannelNum;

	/* Reads the reference divider */
	//uint8_t cRefDiv = (uint8_t)SpiritRadioGetRefDiv()+1;
	uint8_t tempRegValue;
	SpiReadRegisters(portType, SYNTH_CONFIG1_BASE, 1, &tempRegValue);
	uint8_t cRefDiv = (tempRegValue>>7)&0x1 + 1;

	switch(band)
	{
	case VERY_LOW_BAND:
		if(Fc<161281250)
			SpiritCalibrationSelectVco(portType, VCO_L);
		else
			SpiritCalibrationSelectVco(portType, VCO_H);
		break;
		
	case LOW_BAND:
		if(Fc<322562500)
			SpiritCalibrationSelectVco(portType, VCO_L);
		else
			SpiritCalibrationSelectVco(portType, VCO_H);
		break;
		
	case MIDDLE_BAND:
		if(Fc<430083334)
			SpiritCalibrationSelectVco(portType, VCO_L);
		else
			SpiritCalibrationSelectVco(portType, VCO_H);
		break;
		
	case HIGH_BAND:
		if(Fc<860166667)
			SpiritCalibrationSelectVco(portType, VCO_L);
		else
			SpiritCalibrationSelectVco(portType, VCO_H);
	}

	/* Search the VCO charge pump word and set the corresponding register */
	wcp = SpiritRadioSearchWCP(Fc);

	synthWord = (uint32_t)(lFBase*(((double)(FBASE_DIVIDER*cRefDiv*s_vectcBHalfFactor[band]))/XTAL_FREQ));

	/* Build the array of registers values for the analog part */
	anaRadioRegArray[0] = (uint8_t)(((synthWord>>21)&(0x0000001F))|(wcp<<5));
	anaRadioRegArray[1] = (uint8_t)((synthWord>>13)&(0x000000FF));
	anaRadioRegArray[2] = (uint8_t)((synthWord>>5)&(0x000000FF));
	anaRadioRegArray[3] = (uint8_t)(((synthWord&0x0000001F)<<3)| s_vectcBandRegValue[band]);

	/* Configures the needed Analog Radio registers */
	SpiWriteRegisters(portType, SYNT3_BASE, 4, anaRadioRegArray);
}

void ManualVcoCalibration(CM2PortType portType)//SpiritDevice* device)
{
	/*
	1. Set the T split time to the longest value (3.47 ns) to facilitate calibrator operation, write
	1 in SEL_TSPLIT, register SYNTH_CONFIG[0] (register address 0x9F). It is
	recommended to set this register during radio initialization.
	2. If the reference clock is 48 MHz, 50 MHz or 52 MHz and the reference divider is not
	enabled, it must be enabled. Write 1 in the REFDIV bitfield, register SYNTH_CONFIG
	(register address 0x9E), and set the center frequency using the reference divider.
	3. The VCO current must be increased by writing 0x19 in the register VCO_CONFIG
	(register address 0xA1).
	4. Enable automatic calibration of the VCO, writing 1 in VCO_CALIBRATION, register
	PROTOCOL[2] (register address 0x50).
	5. This step must be carried out only when the device is used as a transmitter.
	a. Send a LOCKTX command and wait for the SPIRIT1 to go into LOCK state.
	b. Read the VCO calibration word from VCO_CALIBR_DATA, register
	RCO_VCO_CALIBR_OUT[0] (register address 0xE5). Write the value read into
	the VCO_CALIBR_TX, in register RCO_VCO_CALIBR_IN[1] (register address
	0x6E); optionally this value can be saved in the micro NVM.
	c. Send a READY command and wait for SPIRIT1 to go into READY state.
	6. This step must be carried out only if the device is used as a receiver.
	a. Send a LOCKRX command and wait for the SPIRIT1 to go into LOCK state.
	b. Read the VCO calibration word from VCO_CALIBR_DATA, register
	RCO_VCO_CALIBR_OUT[0] (register address 0xE5). Write the value read into
	the VCO_CALIBR_RX, in register RCO_VCO_CALIBR_IN[0] (register address
	0x6F); optionally this value can be saved in the micro NVM.
	c. Send a READY command and wait for SPIRIT1 to go into READY state.
	7. Disable the automatic calibration of the VCO, write 0 in VCO_CALIBRATION, register
	PROTOCOL[2] (register address 0x50).
	8. Restore the VCO current by writing 0x11 in the register VCO_CONFIG (register
	address 0xA1).
	9. If step 2 was executed, restore the reference divider state. Write 0 in the REFDIV
	bitfield, register SYNTH_CONFIG (register address 0x9E). Again, set the center
	frequency.
	*/
  
	uint8_t s_cVcoWordRx;
	uint8_t s_cVcoWordTx;
	uint32_t nFreq;
	uint8_t cStandby = 0;
	uint8_t tempRegValue;

	// Enable the reference divider if not enabled, and remember to turn it back on
	SpiReadRegisters(portType, SYNTH_CONFIG1_BASE, 1, &tempRegValue);
	if(((tempRegValue>>7)&0x1) == 0)
	{
		nFreq = SpiritRadioGetFrequencyBase(portType);
		
		// enable ref div
		SpiReadRegisters(portType, SYNTH_CONFIG1_BASE, 1, &tempRegValue);
		tempRegValue |= 0x80;
		SpiWriteRegisters(portType, SYNTH_CONFIG1_BASE, 1, &tempRegValue);
		
		SpiritManagementSetFrequencyBase(portType, nFreq);
	}
	nFreq = SpiritRadioGetFrequencyBase(portType);

	// Increase the VCO current
	tempRegValue = 0x19;
	SpiWriteRegisters(portType, 0xA1,1,&tempRegValue);

	// Read-modify-write the VCO calibration register value to enable it
	SpiReadRegisters(portType, PROTOCOL2_BASE, 1, &tempRegValue);
	tempRegValue |= PROTOCOL2_VCO_CALIBRATION_MASK;
	SpiWriteRegisters(portType, PROTOCOL2_BASE, 1, &tempRegValue);

	SpiritUpdateStatus_public(1);//SpiritRefreshStatus();
	if(GetSpiritState(1) == MC_STATE_STANDBY)
	{
		cStandby = 1;
		SpiCommandStrobe(portType, CMD_READY);//SpiritCmdStrobeReady();
		do{
			//SpiritRefreshStatus();
			SpiritUpdateStatus_public(1);
			if(GetSpiritState(1) == 0x13)
			{
				return;/////////////return 1;
			}
		}while(GetSpiritState(1) != MC_STATE_READY); 
	}

	SpiCommandStrobe(portType, CMD_LOCKTX);//SpiritCmdStrobeLockTx();

	do{
		//SpiritRefreshStatus();
		SpiritUpdateStatus_public(1);
		if(GetSpiritState(1) == 0x13)//(g_xStatus.MC_STATE == 0x13)
		{
			return;/////////////////////return 1;
		}
	}while(GetSpiritState(1) != MC_STATE_LOCK);//(g_xStatus.MC_STATE != MC_STATE_LOCK);

	//s_cVcoWordTx = SpiritCalibrationGetVcoCalData();
	// Build and returns the VCO calibration data value
	SpiReadRegisters(portType, RCO_VCO_CALIBR_OUT0_BASE, 1, &tempRegValue);
	s_cVcoWordTx = (tempRegValue & 0x7F);

	SpiCommandStrobe(portType, CMD_READY);//SpiritCmdStrobeReady();

	do{
		//SpiritRefreshStatus();
		SpiritUpdateStatus_public(1);
	}while(GetSpiritState(1) != MC_STATE_READY); 


	SpiCommandStrobe(portType, CMD_LOCKRX);//SpiritCmdStrobeLockRx();

	do{
		//SpiritRefreshStatus();
		SpiritUpdateStatus_public(1);
		if(GetSpiritState(1) == 0x13)//(g_xStatus.MC_STATE == 0x13)
		{
			return;/////////////////////return 1;
		}
	}while(GetSpiritState(1) != MC_STATE_LOCK);//(g_xStatus.MC_STATE != MC_STATE_LOCK);

	//s_cVcoWordRx = SpiritCalibrationGetVcoCalData();
	// Build and returns the VCO calibration data value
	SpiReadRegisters(portType, RCO_VCO_CALIBR_OUT0_BASE, 1, &tempRegValue);
	s_cVcoWordRx = (tempRegValue & 0x7F);

	//SpiritCmdStrobeReady();
	SpiCommandStrobe(portType, CMD_READY);

	do{
		//SpiritRefreshStatus();
		SpiritUpdateStatus_public(1);
		if(GetSpiritState(1) == 0x13)
		{
			return;/////////////return 1;
		}
	}while(GetSpiritState(1) != MC_STATE_READY); 

	if(cStandby == 1)
	{
		SpiCommandStrobe(portType, CMD_STANDBY);//SpiritCmdStrobeStandby();    
	}
	//SpiritCalibrationVco(S_DISABLE);
	// Read-modify-write to disable the VCO calibration
	SpiReadRegisters(portType, PROTOCOL2_BASE, 1, &tempRegValue);
	tempRegValue &= ~PROTOCOL2_VCO_CALIBRATION_MASK;
	SpiWriteRegisters(portType, PROTOCOL2_BASE, 1, &tempRegValue);

	//SpiritRadioSetRefDiv(S_DISABLE);
	// read-modify-write to disable the reference divider
	SpiReadRegisters(portType, SYNTH_CONFIG1_BASE, 1, &tempRegValue);
	tempRegValue &= 0x7F;
	SpiWriteRegisters(portType, SYNTH_CONFIG1_BASE, 1, &tempRegValue);
	
	SpiritManagementSetFrequencyBase(portType, nFreq);

	// Restore the VCO current
	tempRegValue = 0x11;
	SpiWriteRegisters(portType, 0xA1,1,&tempRegValue);

	//SpiritCalibrationSetVcoCalDataTx(s_cVcoWordTx);
	// read-modify-write the new value of calibration data in TX
	SpiReadRegisters(portType, RCO_VCO_CALIBR_IN1_BASE, 1, &tempRegValue);
	tempRegValue &= 0x80;
	tempRegValue |= s_cVcoWordTx;
	SpiWriteRegisters(portType, RCO_VCO_CALIBR_IN1_BASE, 1, &tempRegValue);

	//SpiritCalibrationSetVcoCalDataRx(s_cVcoWordRx);
	// read-modify-write the new value of calibration data in RX
	SpiReadRegisters(portType, RCO_VCO_CALIBR_IN0_BASE, 1, &tempRegValue);
	tempRegValue &= 0x80;
	tempRegValue |= s_cVcoWordRx;
	SpiWriteRegisters(portType, RCO_VCO_CALIBR_IN0_BASE, 1, &tempRegValue);
}
#endif
