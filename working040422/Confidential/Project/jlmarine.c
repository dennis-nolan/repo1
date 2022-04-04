// file name jlmarine.c


#include "jlmarine.h"
#include "Timebase.h"

#define halloffset 35536
#define jltorqlim 5000


signed short jlspeedcommand;


unsigned short raw;
unsigned short slew;


signed long rampdiff;

unsigned short rampstep;

signed short stepdiff;

signed long booster;
signed long myspeedfbhp;
signed long myspeedfblp;
signed long filter1;
signed long filter2;
signed long accel;

signed short jlspeedref;
unsigned long reftimer;
signed short jltorque;

signed short anglim;

signed long bigrawangle;
signed long bigrawanglesl;

unsigned short oldrawhallangle;
unsigned short rawhallangle;
unsigned  char step; // hall step 0,1,2,3,4,5 then repeat
unsigned  short hallangle; // 0 to 65536 for 0 to 360 degrees angle
signed long myspeedfb2; // speed estimate used for speed regulator
unsigned long fullperiod; // counts ticks in full 6 hall periods
unsigned long fullperiodcounter; // ticks in one full period (6 steps)

unsigned char reversal;
unsigned char hallfw;
signed long myspdint;

// runs once at power-up
void jlminit( void)
{
myspeedfb2=0;
fullperiod=0;
fullperiodcounter=0;
}

// runs every motor start
void jlmarinemotorstartinit(void)
{
jlspeedcommand=0;
booster=0;
filter1=0;
filter2=0;
myspeedfblp=0;
myspeedfbhp=0;
jlspeedref = 600;
reftimer=0;
myspeedfb2=0;
fullperiod=0;
fullperiodcounter=0;
hallfw=255;
reversal=0;
myspdint=0;

rawhallangle=0;
oldrawhallangle=0;
bigrawangle = 0;
bigrawanglesl = 0;
jltorque=0;
}


void jlmreadhalls(void)
{
unsigned short word0;
signed short sword0;
unsigned short word1;
unsigned long long0;
unsigned char byte0;
signed long slong0;
signed long slong1;
signed long posintlim;
signed long negintlim;


reftimer++;
if(reftimer< (20000 * 5) ) jlspeedref=1600;
else jlspeedref=-200;
if(reftimer> (20000 * 10 ) ) reftimer=0;

jlspeedref = jlspeedcommand;
if(jlspeedref>2000) jlspeedref=2000;
if(jlspeedref<-2000) jlspeedref=-2000;


// noise rejection
byte0=255;
while(byte0)
{
word0 = GPIOC->IDR;
word0= word0>>6;
word0= word0 & 0x0007;

long0=100;
while(long0>0) long0--;

word1 = GPIOC->IDR;
word1=  word1>>6;
word1=  word1 & 0x0007;

if(word0==word1) byte0=0;
}



fullperiodcounter++;
if(fullperiodcounter>fullperiod) fullperiod=fullperiodcounter;
if(fullperiod>1000000000) fullperiod=1000000000;

switch(word0)
{
case 0: // not legal
break;

case 1:
if(step==5)
{
if(hallfw) reversal=0;
else {hallfw=255; reversal=255; }
}
if(step==1)
{
if(hallfw==0) reversal=0;
else {hallfw=0; reversal=255; }
}

if(step!=0)
{
fullperiod=fullperiodcounter;
fullperiodcounter=0;
}
step=0;
break;

case 2:
//if(step==1) {hallfw=255; }
//if(step==3) {hallfw=0; }
step=2;
break;

case 3:
//if(step==0) {hallfw=255; }
//if(step==2) {hallfw=0; }
step=1;
break;

case 4:
//if(step==3) {hallfw=255; }
//if(step==5) {hallfw=0; }
step=4;
break;

case 5:
//if(step==4) {hallfw=255; }
//if(step==0) {hallfw=0; }
step=5;
break;

case 6:
//if(step==2) {hallfw=255; }
//if(step==4) {hallfw=0; }
step=3;
break;

case 7: // not legal
break;

} // end of switch statement

rawhallangle = halloffset + 5461 + (step*10923) ; 

// calculate speed
long0=383158;
long0 = long0/fullperiod;
if(hallfw) myspeedfb2=long0;
else myspeedfb2=-long0;

#define filter1tc 12
slong0 = myspeedfb2;
slong0 = slong0 - (filter1>>filter1tc);
filter1 = filter1 + slong0;
myspeedfblp = filter1>>filter1tc;
myspeedfbhp = myspeedfb2 - myspeedfblp;

#define filter2tc 10
slong0=myspeedfbhp;
slong0 = slong0 - (filter2>>filter2tc);
filter2 = filter2 + slong0;
booster = filter2>>filter2tc;

if(booster<0) booster = -booster;
if(booster>500) booster=500;


if(reversal) myspeedfb2=0;

// accumulate 16 bit rawhallangle into 32 bit bigrawangle
word0=rawhallangle;
word0 = word0 - oldrawhallangle;
oldrawhallangle = rawhallangle;
sword0=word0;
bigrawangle = bigrawangle + sword0;

// angle slew limit calculation
if(myspeedfb2<0) long0 = -myspeedfb2;
else long0 = myspeedfb2;
long0 = long0 + (booster/5);
if(reversal) long0=200;


//long0 = long0 * 110;
long0 = long0 * 110;

long0 = long0 / 600;

if(long0<25) long0=25;
//if(long0<50) long0=50;

if(long0>2000) long0=2000;
anglim=long0;

// apply slew limit to bigrawangle
// to get bigrawanglesl 
// and then pass on to hallangle
slong0 = bigrawangle;
slong0 = slong0 - bigrawanglesl;

rampdiff = slong0;

raw = bigrawangle;
slew = bigrawanglesl;

if(slong0>anglim) slong0=anglim;
if(slong0<-anglim) slong0=-anglim;
bigrawanglesl = bigrawanglesl + slong0;
hallangle = bigrawanglesl ;

rampstep=hallangle;
rampstep = rampstep-halloffset;
rampstep = rampstep-5461;
rampstep = rampstep/10923;

stepdiff = step - rampstep;

// speed regulator
slong0 = jlspeedref - myspeedfb2; // speed error
myspdint = myspdint + slong0; // raw speed error integral
slong0 = slong0*5;  // prop term

slong1 = jltorqlim ; // total max positive
slong1 = slong1 - slong0; // leaves allowable pos int
posintlim = slong1 * 1000; // account for gain of 1/1000
if(posintlim<0) posintlim=0;

slong1 = -jltorqlim ; // total max negative
slong1 = slong1 - slong0; // leaves allowable neg int
negintlim = slong1 * 1000; // account for gain of 1/1000
if(negintlim>0) negintlim=0;

if(myspdint>posintlim) myspdint = posintlim;
if(myspdint<negintlim) myspdint = negintlim;

slong0 = slong0 + myspdint/1000; // combine prop and int


if(slong0>jltorqlim) slong0=jltorqlim;
if(slong0<-jltorqlim) slong0=-jltorqlim;
jltorque=slong0;



DAC->DHR12L1 = fullperiod;
DAC->SWTRIGR=1;

} // end of read halls 



