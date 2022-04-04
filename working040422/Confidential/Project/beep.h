
 

// Note definitions

#define Cn5 92
#define Csharp5  87
#define D5 82
#define Dsharp5 77
#define E5 73
#define F5 69
#define Fsharp5 65
#define G5 61
#define Gsharp5 58
#define A5 55
#define Asharp5 52
#define B5 49
#define C6 46
#define Csharp6 43  
#define D6 41
#define Dsharp6 39
#define E6 36
#define F6 34
#define Fsharp6 32
#define G6 31
#define Gsharp6 29
#define A6 27
#define Asharp6 26
#define B6 24
#define rest 0

//#define MOT_EN GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_SET); motor_relay_enabled = TRUE;
//#define MOT_EN_FALSE GPIO_WriteBit(GPIOB,GPIO_Pin_12,Bit_RESET); motor_relay_enabled = FALSE;

//programming state

typedef enum
{   PPOLE, 
    SUCS, 
    FAIL,
    ERR,
    CHARGE,
    SCALE,
    TUNE_PROGRAM_ENTER
} Tune;


extern uint8_t tempo;
extern uint16_t debounceBlockingTimer; 


void Beep(uint16_t ,uint16_t);


void BeepInit(void);
void BeepDeInit(void);
void StartBeep(uint16_t divider);
void EndBeep(uint16_t divider);
void PlayTune(Tune which_tune);
void PlayTuneSuccess(void);
