/*
project_Quad_rotor v1.1  
1. Automatic  Takeoff 
2. 1 waypoint navigation
3. Landing
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
//PID By tinnakon_za@hotmail.com
*///Channel
int CH_THR;
int CH_AIL;
int CH_ELE;
int CH_RUD;
int AUX_1;
int AUX_2;

#define MINTHROTTLE 1120 
#define MAXTHROTTLE 1850
#define MINCOMMAND 1090
#define MAXCOMMAND 1900
#define MIDRC 1500
#define MINCHECK 1100
#define MAXCHECK 1900

//RX PIN assignment inside the port //for PORTK
#define THROTTLEPIN                2  //
#define ROLLPIN                    4  //
#define PITCHPIN                   5  //
#define YAWPIN                     6  //
#define AUX1PIN                    7  //
#define AUX2PIN                    7  //
#define CAM1PIN                    7  //
#define CAM2PIN                    7  //
#define ISR_UART                   ISR(USART0_UDRE_vect)

#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define CAMPITCH   6
#define CAMROLL    7      

#define TW_STATUS_MASK	(1<<TWS7) | (1<<TWS6) | (1<<TWS5) | (1<<TWS4) | (1<<TWS3)
#define TW_STATUS       (TWSR & TW_STATUS_MASK)

static uint8_t pinRcChannel[8] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,CAM1PIN,CAM2PIN};
volatile uint16_t rcPinValue[8] = {1500,1500,1500,1500,1500,1500,1500,1500};
static int16_t rcData[8] ;
static int16_t rcCommand[4] ; 
static int16_t rcHysteresis[8] ;
static int16_t rcData4Values[8][4];

volatile uint16_t rcValue[8] = {1500,1500,1500,1500,1500,1500,1500,1500};

unsigned long timer=0;

void configureReceiver() {

    for (uint8_t chan = 0; chan < 8; chan++){
      for (uint8_t a = 0; a < 4; a++){
        rcData4Values[chan][a] = 1500;
      }  
    }    
      //DDRK = 0; 
      PORTD   = (1<<2) | (1<<4) | (1<<5) | (1<<6) | (1<<7); 
      PCMSK2 |= (1<<2) | (1<<4) | (1<<5) | (1<<6) | (1<<7);
      PCICR   = 1<<2;
}

ISR(PCINT2_vect) { 
  uint8_t mask;
  uint8_t pin;
  uint16_t cTime,dTime;
  static uint16_t edgeTime[8];
  static uint8_t PCintLast;
  pin = PIND;            
  mask = pin ^ PCintLast;   
  sei();                   
  PCintLast = pin;     
  cTime = micros();        

  if (mask & 1<<2)          
    if (!(pin & 1<<2)) {    
      dTime = cTime-edgeTime[2]; if (900<dTime && dTime<2200) rcPinValue[2] = dTime; 
    } else edgeTime[2] = cTime;   
  if (mask & 1<<4)   
    if (!(pin & 1<<4)) {
      dTime = cTime-edgeTime[4]; if (900<dTime && dTime<2200) rcPinValue[4] = dTime;
    } else edgeTime[4] = cTime;
  if (mask & 1<<5)
    if (!(pin & 1<<5)) {
      dTime = cTime-edgeTime[5]; if (900<dTime && dTime<2200) rcPinValue[5] = dTime;
    } else edgeTime[5] = cTime;
  if (mask & 1<<6)
    if (!(pin & 1<<6)) {
      dTime = cTime-edgeTime[6]; if (900<dTime && dTime<2200) rcPinValue[6] = dTime;
    } else edgeTime[6] = cTime;
  if (mask & 1<<7)
    if (!(pin & 1<<7)) {
      dTime = cTime-edgeTime[7]; if (900<dTime && dTime<2200) rcPinValue[7] = dTime;
    } else edgeTime[7] = cTime;
}

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG;
  data = rcPinValue[pinRcChannel[chan]];
  SREG = oldSREG;
  return data; 
}
  
void computeRC() {
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;
  rc4ValuesIndex++;
  for (chan = 0; chan < 8; chan++) {
    rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);
    rcData[chan] = 0;
    for (a = 0; a < 4; a++){
      rcData[chan] += rcData4Values[chan][a];
    }
    rcData[chan]= (rcData[chan]+2)/4;
    if ( rcData[chan] < rcHysteresis[chan] -3)  rcHysteresis[chan] = rcData[chan]+2;
    if ( rcData[chan] > rcHysteresis[chan] +3)  rcHysteresis[chan] = rcData[chan]-2;
  }
    CH_THR = rcHysteresis[THROTTLE];
    CH_AIL = rcHysteresis[ROLL];
    CH_ELE = rcHysteresis[PITCH];
    CH_RUD = rcHysteresis[YAW];
    AUX_1 = rcHysteresis[AUX1];
}


