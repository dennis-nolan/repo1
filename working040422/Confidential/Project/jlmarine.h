// file name jlmarine.h



extern signed short jlspeedcommand;

extern unsigned  char step;
extern unsigned  short hallangle;
extern unsigned  short steptimecounter;
extern unsigned  short hallperiod;
extern unsigned char goflag;
extern unsigned long goflagcounter;
extern signed long myspeedfb2;
extern signed long myspdint;
extern signed long speedest;

extern signed short jltorque;




void jlmreadhalls(void);
void jlminit( void);
void jlmarinemotorstartinit(void);






