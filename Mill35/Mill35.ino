//#include <Wire.h>
//#include <WireIMXRT.h>
//#include <WireKinetis.h>

///
/// @mainpage   Stepper32
///
/// @details   Description of the project
/// @n
/// @n
/// @n @a      Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author      Ruedi Heimlicher
/// @author      Ruedi Heimlicher
/// @date      06.05.2020 21:02
/// @version   <#version#>
///
/// @copyright   (c) Ruedi Heimlicher, 2020
/// @copyright   GNU General Public Licence
///
/// @see      ReadMe.txt for references
///


///
/// @file      Stepper32.ino
/// @brief      Main sketch
///
/// @details   <#details#>
/// @n @a      Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author      Ruedi Heimlicher
/// @author      Ruedi Heimlicher
/// @date      06.05.2020 21:02
/// @version   1.1
///
/// @copyright   (c) Ruedi Heimlicher, 2020
/// @copyright   GNU General Public Licence
///
/// @see      ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu
#include "Arduino.h"
#include "TeensyStep.h"

#include "gpio_MCP23S17.h"
#include <SPI.h>
#include "lcd.h"
#include "settings.h"
//#include <Wire.h>
//#include <i2c_t3.h>
//#include <LiquidCrystal_I2C.h> // auch in Makefile angeben!!!
#include <TeensyThreads.h>

// Set parameters


// Include application, user and local libraries
// !!! Help http://bit.ly/2CL22Qp


// Define structures and classes


// Define variables and constants
uint8_t loopLED;
#define USB_DATENBREITE 64

#define TEST 0

int8_t r;

// USB
volatile uint8_t inbuffer[USB_DATENBREITE]={};
volatile uint8_t outbuffer[USB_DATENBREITE]={};
volatile uint16_t          usb_recv_counter=0;
volatile uint16_t          cnc_recv_counter=0;
// end USB


elapsedMillis sinceblink;
elapsedMillis sincelcd;
elapsedMillis sinceusb;

elapsedMillis sinceload; // Zeitdauer der Anzeige des Potentialwertes

elapsedMicros sincelaststep;

elapsedMillis sincelastthread;

// Prototypes

static volatile uint8_t buffer[USB_DATENBREITE]={};
static volatile uint8_t sendbuffer[USB_DATENBREITE]={};

// Ringbuffer
uint8_t                    CNCDaten[RINGBUFFERTIEFE][USB_DATENBREITE];
uint8_t                    CDCStringArray[RINGBUFFERTIEFE];

volatile uint16_t          abschnittnummer=0;
volatile uint16_t          endposition= 0xFFFF;
volatile uint16_t           ladeposition=0;

//volatile uint16_t          globalaktuelleladeposition = 0;
volatile uint16_t          aktuelleladeposition = 0;
volatile uint8_t           ringbufferstatus=0x00;   

uint16_t                   Abschnitte=0;
uint16_t                   AbschnittCounter=0;
volatile uint8_t           liniencounter= 0;
// end Ringbuffer
volatile uint16_t           steps= 0;

volatile uint16_t korrekturintervallx = 0;
volatile uint16_t korrekturintervally = 0;

volatile uint16_t korrekturintervallcounterx = 0;
volatile uint16_t korrekturintervallcountery = 0;

volatile uint16_t korrekturcounterx = 0;
volatile uint16_t korrekturcountery = 0;


volatile uint8_t vorzeichen = 0;

volatile uint8_t code = 0x00;

volatile uint16_t           loadtime= 0;


volatile uint8_t           timer0startwert=TIMER0_STARTWERT;

volatile uint16_t          timer2Counter=0;
volatile uint8_t           cncstatus=0x00;
volatile uint8_t           sendstatus=0x00;

volatile uint8_t           drillstatus=0x00;
volatile uint8_t           tabledatastatus=0x00;


volatile uint8_t           usbstatus=0x00;
static volatile uint8_t    motorstatus=0x00;
volatile uint8_t           anschlagstatus=0x00;

volatile int16_t           anschlagcounter = 0;

volatile uint8_t           timerstatus=0;

volatile uint8_t           repeatstatus=0; // Bits fuer repeat Pfeiltasten

volatile uint8_t           status=0;

volatile uint8_t           PWM=0;
static volatile uint8_t    pwmposition=0;
static volatile uint8_t    pwmdivider=0;

// CNC

volatile uint16_t          CounterA=0;         // Zaehler fuer Delay von Motor A 
volatile uint16_t          CounterB=0;         // Zaehler fuer Delay von Motor B
volatile uint16_t          CounterC=0;         // Zaehler fuer Delay von Motor C 
volatile uint16_t          CounterD=0;         // Zaehler fuer Delay von Motor D

volatile uint32_t          DelayA=24;         // Delay von Motor A 
volatile uint32_t          DelayB=24;         // Delay von Motor B 
volatile uint32_t          DelayC=24;         // Delay von Motor C 
volatile uint32_t          DelayD=24;         // Delay von Motor D 

volatile uint32_t          StepCounterA=0;   // Zaehler fuer Schritte von Motor A 
volatile uint32_t          StepCounterB=0;   // Zaehler fuer Schritte von Motor B
volatile uint32_t          StepCounterC=0;   // Zaehler fuer Schritte von Motor C 
volatile uint32_t          StepCounterD=0;   // Zaehler fuer Schritte von Motor D

volatile uint8_t           richtung=0;

volatile uint8_t           parallelcounter=0;
volatile uint8_t           parallelstatus=0; // Status des Thread

volatile uint16_t          timerintervall = TIMERINTERVALL;
volatile uint16_t          timerintervall_SLOW = 0; // Intervall klein
volatile uint16_t          timerintervall_FAST = 0; // Intervall gross


// Ramp

volatile uint16_t          ramptimerintervall = TIMERINTERVALL;

volatile uint8_t           rampstatus=0;
volatile uint8_t           RampZeit = RAMPZEIT;
volatile uint8_t           RampFaktor = RAMPFAKTOR;
volatile uint32_t          rampstepstart=0; // Stepcounter am Anfang
volatile uint32_t          ramptimercounter=0;  // laufender counter  fuer Rampanpassung
volatile uint32_t          ramptimerdelay = 200;  // Takt fuer Rampanpassung
volatile uint16_t          rampbreite = 0;  // anzahl Schritte der Ramp. Wird beim Start bestimmt und fuer das Ende verwendet

volatile uint32_t          rampendstep = 0; // Beginn der Endramp. Wird in Abschnittladen bestimmt

volatile uint16_t          tablezeile = 0;  // zeile in tabledata

// Create an IntervalTimer object 
IntervalTimer              delayTimer;

// Utilities

#  pragma mark TeensyStep Variablen
// TeensyStep

int schrittA = 0;
int richtungA = 1;
int schalterA = 2;

int schrittB = 3;
int richtungB = 4;
int schalterB = 5;

int schrittC = 6;
int richtungC = 7;
int schalterC = 8;

Stepper motor_A(schrittA, richtungA);   //STEP pin =  2, DIR pin = 3
Stepper motor_B(schrittB, richtungB);   //STEP pin =  9, DIR pin = 10
Stepper motor_C(schrittC, richtungC);  //STEP pin = 14, DIR pin = 15

int speedA = 1000;
int speedB = 1000;
int speedC = 500;

StepControl controller(10,5000);

int dir = 1;
int dist1 = 0;
int dist2 = 0;
constexpr int spr1 = 2*200;  // 3200 steps per revolution
constexpr int spr2 = 2*200;  // 3200 steps per revolution

static volatile uint8_t    controllerstatus=0x00;

volatile uint8_t    repeatcounter=0; // anzahl repeats
#define RUNNING   1
#define REPEATING   2
#define STOP      3
// end TeensyStep


// Functions

void OSZI_A_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_A,LOW);
}

void OSZI_A_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_A,HIGH);
}

void OSZI_A_TOGG(void)
{
   if (TEST)
      digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
}

void OSZI_B_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_B,LOW);
}

void OSZI_B_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_B,HIGH);
}



void startTimer2(void)
{
   timerstatus |= (1<<TIMER_ON);
}
void stopTimer2(void)
{
   timerstatus &= ~(1<<TIMER_ON);
}

void timerfunction() 
{ 
   if (timerstatus & (1<<TIMER_ON))
   {
      if (PWM) // Draht soll heiss sein. 
      {
      }
      else
      {
         pwmposition =0;
      }
      
      //  if (timer2Counter >= 14) 
      {
         
         if(CounterA)
         {
            CounterA-=1;
         }
         if(CounterB)
         {
            CounterB-=1;
         }
         if(CounterC)
         {
            CounterC-=1;
         }
         if(CounterD)
         {
            CounterD-=1;
         }
         
         if (PWM)
         {
            pwmposition ++;
         }
         else
         {
            pwmposition =0;
         }
         
         //      timer2Counter = 0; 
         //OSZI_B_TOGG ;
      } 
   } // if timerstatus
   //   digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   // TCNT2 = 10;                     // ergibt 2 kHz fuer Timertakt
}

void delaytimerfunction(void) // 1us ohne ramp
{ 
   // digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   
   if (timerstatus & (1<<TIMER_ON))
   {
      //OSZI_A_LO();
      {
         
         if(CounterA)
         {
            CounterA-=1;
         }
         if(CounterB)
         {
            CounterB-=1;
         }
         if(CounterC)
         {
            CounterC-=1;
         }
         if(CounterD)
         {
            CounterD-=1;
         }
         
         if (PWM)
         {
            pwmposition ++;
         }
         else
         {
            pwmposition =0;
         }
         
         //      timer2Counter = 0; 
         //OSZI_B_TOGG ;
      } 
      //OSZI_A_HI();
      
      {
         ramptimercounter += 1;
         
         //      Serial.printf("start rampstatus: %d",rampstatus);
         //      rampstatus = 0;
         
         if (ramptimercounter > ramptimerdelay) // Teiler, 200us
         {
            ramptimercounter = 0;
            if (rampstatus & (1<<RAMPSTARTBIT))
            {
               if (ramptimerintervall > timerintervall_FAST) // noch nicht auf max speed
               {
                  //  Serial.printf("start ramptimerintervall: %d\n",ramptimerintervall);
                  ramptimerintervall -= RAMPSCHRITT;
                  delayTimer.update(ramptimerintervall);
                  rampbreite++;
               }
               else
               {
                  
                  rampstatus &= ~(1<<RAMPSTARTBIT);
                  rampendstep = rampstepstart - max(StepCounterA, StepCounterB);
                  rampstatus |= (1<<RAMPENDBIT);
                  rampstatus |= (1<<RAMPEND0BIT);
                  // Serial.printf("start rampstepstart: %d rampendstep: %d ramptimerintervall: %d timerintervall: %d\n",rampstepstart,rampendstep, ramptimerintervall,timerintervall);
                  
               }
            }
            if (rampstatus & (1<<RAMPENDBIT))
            {
               
               if (max(StepCounterA, StepCounterB) < rampendstep)
               {
                  //Serial.printf("end StepCounterA: %d\n",StepCounterA);
                  // ramptimerintervall ist timerintervall_FAST
                  if (rampstatus & (1<<RAMPEND0BIT))
                  {
                     //Serial.printf("rampend0:  rampendstep: %d StepCounterA: %d StepCounterB: %d ramptimerintervall: %d timerintervall: %d timerintervall_SLOW: %d\n",rampendstep, StepCounterA,StepCounterB,ramptimerintervall,timerintervall,timerintervall_SLOW);
                     
                     rampstatus &= ~(1<<RAMPEND0BIT);
                  }
                  if (ramptimerintervall < timerintervall_SLOW)
                  {
                     //Serial.printf("end StepCounterA: %d ramptimerintervall: %d\n",StepCounterA,ramptimerintervall);
                     ramptimerintervall += RAMPSCHRITT;
                     
                     delayTimer.update(ramptimerintervall);
                     //rampbreite++;
                  }
                  
               }
               
            } // if RAMPENDBIT
         }
      }
      
   } // if timerstatus
   
   //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   // TCNT2 = 10;                     // ergibt 2 kHz fuer Timertakt
   
}

uint8_t  RepeatAbschnittLaden_TS(const uint8_t* AbschnittDaten) // 22us
{
   Serial.printf("RepeatAbschnittDaten_TS Start\n");
   stopTimer2();
   uint8_t returnwert=0;
   int lage = 0;
   
   lage = AbschnittDaten[25]; // Start: 1, innerhalb: 0, Ende: 2
   if (lage & 0x01)
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   StepCounterA = AbschnittDaten[0] | (AbschnittDaten[1]<<8) | (AbschnittDaten[2]<<16) | ((AbschnittDaten[3] & 0x7F)<<24);
    // Vorzeichen bestimmen
   if ((AbschnittDaten[3] & 0x80) > 0)
   {
      //Serial.printf("StepCounterA: Vorzeichen negativ\n");
      StepCounterA *= -1;
   }
   else
   {
      //Serial.printf("StepCounterA: Vorzeichen positiv\n");
   }
   Serial.printf("StepCounterA mit VZ: %d\n",StepCounterA);
   StepCounterB = AbschnittDaten[8] | (AbschnittDaten[9]<<8) | (AbschnittDaten[10]<<16) | ((AbschnittDaten[11] & 0x7F)<<24);
   //Serial.printf("StepCounterB: %d \n",StepCounterB);
   // Vorzeichen bestimmen
   if ((AbschnittDaten[11] & 0x80) > 0)
   {
      //Serial.printf("StepCounterB: Vorzeichen negativ\n");
      StepCounterB *= -1;
   }
   else
   {
      //Serial.printf("StepCounterB: Vorzeichen positiv\n");
   }
   Serial.printf("StepCounterB mit VZ: %d\n",StepCounterB);
   
   uint32_t dA = StepCounterA;
   uint32_t dB = StepCounterB;

   //Serial.printf("da: %d dB: %d\n",dA,dB);
   motor_A.setTargetRel(dA);
   motor_B.setTargetRel(dB);
   Serial.printf("repeatcounter: %d\n",repeatcounter);
   if (repeatcounter == 1) // Start
   {
      Serial.printf("repeatcounter 1: %d\n",repeatcounter);
      //   if ((digitalReadFast(END_A0_PIN)) &&  (digitalReadFast(END_A1_PIN)))
      {
         // Serial.printf("Alles offen\n");
         digitalWriteFast(MA_EN,LOW);
      }
      //   if ((digitalReadFast(END_B0_PIN)) &&  (digitalReadFast(END_B1_PIN)))
      {
         // Serial.printf("Alles offen\n");
         digitalWriteFast(MB_EN,LOW);
      }
      controllerstatus |= (1<<RUNNING);
      controller.moveAsync(motor_A, motor_B);

   }
   else if (repeatcounter == 0) // end
   {
      Serial.printf("repeatcounter 0: %d\n",repeatcounter);
      digitalWriteFast(MA_EN,HIGH);
      digitalWriteFast(MB_EN,HIGH);
      controllerstatus |= (1<<STOP);

   }
   Serial.printf("RepeatAbschnittDaten_TS End\n");
   startTimer2();
   return returnwert;
   
}
 



uint8_t  AbschnittLaden_TS(const uint8_t* AbschnittDaten) // 22us
{
   //OSZI_A_LO();
   uint8_t l = sizeof(AbschnittDaten);
    Serial.printf("\n*********                   AbschnittLaden_TS:  AbschnittDaten len: %d Motor: %d\n", l,AbschnittDaten[28]);
   stopTimer2();
   uint8_t returnwert=0;
   parallelstatus  |= (1<<THREAD_COUNT_BIT);
      Serial.printf("AbschnittDaten_TS\n");
   //  for(int i=0;i<32;i++) // 5 us ohne printf, 10ms mit printf
   { 
      //         Serial.printf("%d \t",AbschnittDaten[i]);
   }
   //OSZI_A_HI();
   //  rampstatus = 0;
   //   Serial.printf("\n            end Abschnittdaten\n");
   
#  pragma mark Reihenfolge der Daten
   /*         
    Rehenfolge der Daten   
    
    0   schrittexA
    1   schrittexB
    2   schrittexC
    3   schrittexD
    4   delayxA
    5   delayxB
    
    6   delayxC
    7   delayxD
    
    8   schritteyA
    9   schritteyB
    10   schritteyC
    11   schritteyD
    12   delayyA
    13   delayyB
    14   delayyC
    15   delayyD
    16   schrittezA
    17   schrittezB
    18   schrittezC
    19   schrittezD
    20   delayzA
    21   delayzB
    22   delayzC
    23   delayzD
    24   code
    25   position lage im Schnittpolygom
    26   indexh
    27   indexl
    */         
   int lage = 0;
   
   lage = AbschnittDaten[25]; // Start: 1, innerhalb: 0, Ende: 2
   if (lage & 0x01)
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   richtung=0;
   
   // Motor A
   //   
   
   StepCounterA = AbschnittDaten[0] | (AbschnittDaten[1]<<8) | (AbschnittDaten[2]<<16) | ((AbschnittDaten[3] & 0x7F)<<24);
    // Vorzeichen bestimmen
   if ((AbschnittDaten[3] & 0x80) > 0)
   {
      //Serial.printf("StepCounterA: Vorzeichen negativ\n");
      StepCounterA *= -1;
   }
   else
   {
      //Serial.printf("StepCounterA: Vorzeichen positiv\n");
   }
   Serial.printf("StepCounterA mit VZ: %d\n",StepCounterA);
   
   // Motor B
   //CounterB=0;
   //STEPPERPORT_1 &= ~(1<<MB_EN);   // Pololu ON
   
   StepCounterB = AbschnittDaten[8] | (AbschnittDaten[9]<<8) | (AbschnittDaten[10]<<16) | ((AbschnittDaten[11] & 0x7F)<<24);
   Serial.printf("StepCounterB: %d \n",StepCounterB);
   // Vorzeichen bestimmen
   if ((AbschnittDaten[11] & 0x80) > 0)
   {
      //Serial.printf("StepCounterB: Vorzeichen negativ\n");
      StepCounterB *= -1;
   }
   else
   {
      //Serial.printf("StepCounterB: Vorzeichen positiv\n");
   }
   Serial.printf("StepCounterB mit VZ: %d\n",StepCounterB);

 
   
   // Motor C
   //STEPPERPORT_2 &= ~(1<<MC_EN); // Pololu ON
  
   //CounterC=0;
   StepCounterC = AbschnittDaten[16] | (AbschnittDaten[17]<<8) | (AbschnittDaten[18]<<16) | ((AbschnittDaten[19] & 0x7F)<<24);
    
   //richtung=0;
   if (AbschnittDaten[19] & 0x80) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_C); // Rueckwarts
      //STEPPERPORT_2 &= ~(1<< MC_RI);
//      digitalWriteFast(MC_RI,LOW);
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_C);
      //STEPPERPORT_2 |= (1<< MC_RI);
//      digitalWriteFast(MC_RI,HIGH);
   }
   
  
   uint32_t dA = StepCounterA;
   uint32_t dB = StepCounterB;
   
   
   //Serial.printf("da: %d dB: %d\n",dA,dB);
   motor_A.setTargetRel(dA);
   motor_B.setTargetRel(dB);
   //   if ((digitalReadFast(END_A0_PIN)) &&  (digitalReadFast(END_A1_PIN)))
   {
      // Serial.printf("Alles offen\n");
      digitalWriteFast(MA_EN,LOW);
   }
   //   if ((digitalReadFast(END_B0_PIN)) &&  (digitalReadFast(END_B1_PIN)))
   {
      // Serial.printf("Alles offen\n");
      digitalWriteFast(MB_EN,LOW);
   }
   digitalWriteFast(MC_EN,LOW);
   
   controllerstatus |= (1<<RUNNING);
   controller.moveAsync(motor_A, motor_B);
   
 //  digitalWriteFast(MA_EN,HIGH);
 //  digitalWriteFast(MB_EN,HIGH);
 //  digitalWriteFast(MC_EN,HIGH);
   
   
   Serial.printf("*********   AbschnittLaden_TS END \n");
   return returnwert;

                 
                 // Figur
   for (uint8_t i = 0;i<16;i++)
   {
      Serial.printf("i: %d da: %d dB: %d\n",i,dA,dB);
      motor_A.setTargetRel(dA);
      motor_B.setTargetRel(dB);
      //   if ((digitalReadFast(END_A0_PIN)) &&  (digitalReadFast(END_A1_PIN)))
      {
         // Serial.printf("Alles offen\n");
         digitalWriteFast(MA_EN,LOW);
      }
      //   if ((digitalReadFast(END_B0_PIN)) &&  (digitalReadFast(END_B1_PIN)))
      {
         // Serial.printf("Alles offen\n");
         digitalWriteFast(MB_EN,LOW);
      }
      controller.move(motor_A, motor_B);
      
      digitalWriteFast(MA_EN,HIGH);
      digitalWriteFast(MB_EN,HIGH);
      
      motor_A.setTargetRel(-dA);
      motor_B.setTargetRel(dB);
      digitalWriteFast(MA_EN,LOW);
      digitalWriteFast(MB_EN,LOW);
      
      controller.move(motor_A, motor_B);
      
      digitalWriteFast(MA_EN,HIGH);
      digitalWriteFast(MB_EN,HIGH);
      digitalWriteFast(MC_EN,HIGH);
      
      motor_A.setTargetRel(-dA);
      motor_B.setTargetRel(-dB);
      digitalWriteFast(MA_EN,LOW);
      digitalWriteFast(MB_EN,LOW);
      
      controller.move(motor_A, motor_B);
      
      digitalWriteFast(MA_EN,HIGH);
      digitalWriteFast(MB_EN,HIGH);
      
      motor_A.setTargetRel(dA);
      motor_B.setTargetRel(-dB);
      digitalWriteFast(MA_EN,LOW);
      digitalWriteFast(MB_EN,LOW);
      
      controller.move(motor_A, motor_B);
      
      digitalWriteFast(MA_EN,HIGH);
      digitalWriteFast(MB_EN,HIGH);
      
      dA += 400;
      dB += 400;
   }
   
   
   /*
   
   // motorstatus: welcher Motor ist relevant
   motorstatus = AbschnittDaten[28];
   Serial.printf("*** Abschnittladen motorstatus: %d\n",motorstatus); 
   
   if (motorstatus > 3)
   {
      Serial.printf("*** Abschnittladen motorstatus korr\n"); 
      //motorstatus = 1;
   }
   
#pragma mark Ramp
   //   digitalWriteFast(MA_EN,LOW);
   
   if (rampstatus & (1<<RAMPOKBIT))
   {
      Serial.printf("*** Ramp\n");
      rampstepstart =  max(StepCounterA,StepCounterB); // maximalwert
      timerintervall_FAST = TIMERINTERVALL;
      timerintervall_SLOW = RampFaktor * TIMERINTERVALL; // Verlaengerung der delayzeit
      
      ramptimerintervall = timerintervall_SLOW;
      
      delayTimer.update(ramptimerintervall);
      
      if (rampstepstart > RampZeit) // 5mm
      {
         //rampendstep = max(StepCounterA,StepCounterB) -
         rampstatus |= (1<<RAMPSTARTBIT);
         Serial.printf("*** Ramp\n");
      }
      ramptimercounter = 0;
      rampbreite = 0;
   }
   
   
   
   
   Serial.printf("*                                     *** Abschnittladen motorstatus %d richtung: %02X StepCounterA: %d StepCounterB: %d StepCounterC: %d \n",motorstatus,richtung, StepCounterA, StepCounterB,  StepCounterC);
   */
   startTimer2();
   return returnwert;
   
   //OSZI_A_HI;
   
}

uint8_t  AbschnittLaden_4M(const uint8_t* AbschnittDaten) // 22us
{
   //OSZI_A_LO();
   uint8_t l = sizeof(AbschnittDaten);
   //  Serial.printf("\n*********                   AbschnittLaden_4M:  AbschnittDaten len: %d Motor: %d\n\n", l,AbschnittDaten[28]);
   stopTimer2();
   uint8_t returnwert=0;
   parallelstatus  |= (1<<THREAD_COUNT_BIT);
   //   Serial.printf("AbschnittDaten\n");
   //  for(int i=0;i<32;i++) // 5 us ohne printf, 10ms mit printf
   { 
      //         Serial.printf("%d \t",AbschnittDaten[i]);
   }
   //OSZI_A_HI();
   //  rampstatus = 0;
   //   Serial.printf("\n            end Abschnittdaten\n");
   
#  pragma mark Reihenfolge der Daten
   /*         
    Rehenfolge der Daten   
    
    0   schrittexA
    1   schrittexB
    2   schrittexC
    3   schrittexD
    4   delayxA
    5   delayxB
    
    6   delayxC
    7   delayxD
    
    8   schritteyA
    9   schritteyB
    10   schritteyC
    11   schritteyD
    12   delayyA
    13   delayyB
    14   delayyC
    15   delayyD
    16   schrittezA
    17   schrittezB
    18   schrittezC
    19   schrittezD
    20   delayzA
    21   delayzB
    22   delayzC
    23   delayzD
    24   code
    25   position lage im Schnittpolygom
    26   indexh
    27   indexl
    */         
   int lage = 0;
   
   lage = AbschnittDaten[25]; // Start: 1, innerhalb: 0, Ende: 2
   if (lage & 0x01)
   {
      returnwert=1;
   }
   if (lage & 0x02) // letztes Element
   {
      returnwert=2;
   }
   richtung=0;
   
   // Motor A
   //   
   
   StepCounterA = AbschnittDaten[0] | (AbschnittDaten[1]<<8) | (AbschnittDaten[2]<<16) | ((AbschnittDaten[3] & 0x7F)<<24);
   //uint32_t a  = AbschnittDaten[0] + (AbschnittDaten[1]<<8) + (AbschnittDaten[2]<<16) + ((AbschnittDaten[3] & 0x7F)<<24);
   //Serial.printf("l: %d\n",a);
   DelayA = AbschnittDaten[4] | ((AbschnittDaten[5] & 0x7F)<<8) ;
   Serial.printf("StepCounterA: %d DelayA: %d\n",StepCounterA,DelayA);
   
   //Serial.printf("AbschnittDaten 6: %d AbschnittDaten 7: %d\n",AbschnittDaten[6], AbschnittDaten[7]);
   korrekturintervallx = AbschnittDaten[6] | ((AbschnittDaten[7] & 0x7F)<<8);
   //   korrekturcounterx = 0;
   korrekturintervallcounterx = 0;
   vorzeichen = 0;
   
   uint8_t vorzeichenx = (AbschnittDaten[7] & 0x80 )>> 7;
   //   Serial.printf("korrekturintervallx: %d vorzeichenx: %d\n",korrekturintervallx,vorzeichenx);
   
   if (vorzeichenx)
   {
      //     Serial.printf("korrekturintervallx negativ\n");
      vorzeichen |= (1<<VORZEICHEN_X);
   }
   else
   {
      vorzeichen &= ~(1<<VORZEICHEN_X);
   }
   //  Serial.printf("korrekturintervallx m VZ: %d vorzeichenx: %d\n",korrekturintervallx,vorzeichenx);
   
   //   Serial.printf("StepCounterA: %d DelayA: %d\n",StepCounterA,DelayA);
   //   Serial.printf("dataL: %d dataH: %d  delayL: %d delayH: %d\n",dataL,dataH, delayL,delayH);
   
   
   //lcd_gotoxy(17,0);
   if (AbschnittDaten[3] & 0x80) // Bit 7 gesetzt, negative zahl
   {
      //     Serial.printf("Motor A rueckwaerts\n");
      richtung |= (1<<RICHTUNG_A0); // Rueckwarts
      //STEPPERPORT_1 &= ~(1<< MA_RI); // PIN fuer Treiber stellen
      digitalWriteFast(MA_RI, LOW);
      //lcd_putc('r');
   }
   else 
   {
      //     Serial.printf("Motor A vorwaerts\n");
      richtung &= ~(1<<RICHTUNG_A0);
      //richtung |= (1<<RICHTUNG_A1);
      
      digitalWriteFast(MA_RI,HIGH);
      //lcd_putc('v');   // Vorwaerts
   }
   
   CounterA = DelayA;
   // CounterA = 0;
  
   if ((digitalReadFast(END_A0_PIN)) &&  (digitalReadFast(END_A1_PIN)))
   {
     // Serial.printf("Alles offen\n");
      digitalWriteFast(MA_EN,LOW);
   }
   /* 
   else
   {
      if (digitalReadFast(END_A0_PIN)) // offen
      {
         Serial.printf("Abschnitt_Laden_4M  Anschlag A0 offen\n");
         digitalWriteFast(MA_EN,LOW);
         anschlagstatus &= ~(1<< END_A0);
      }
      else 
      {
         Serial.printf("Abschnitt_Laden_4M Anschlag A0 zu\n");
         if (richtung & (1<<RICHTUNG_A0)) // auf Anschlag zu
         {
            Serial.printf("Abschnitt_Laden_4M falsche Richtung\n");
         }
         else // vom Anschlag weg, OK
         {
            Serial.printf("Abschnitt_Laden_4M richtige Richtung\n");
            digitalWriteFast(MA_EN,LOW);
            anschlagstatus &= ~(1<< END_A0);
         }
         
      }
      if (digitalReadFast(END_A1_PIN))
      {
         Serial.printf("Abschnitt_Laden_4M Anschlag A1 offen\n");
         digitalWriteFast(MA_EN,LOW);
      }
      else
      {
         Serial.printf("Abschnitt_Laden_4M Anschlag A1 zu\n");
      }
   }
   */
   //   Serial.printf("CounterA: %d\n",CounterA);
   
   
   // Motor B
   //CounterB=0;
   //STEPPERPORT_1 &= ~(1<<MB_EN);   // Pololu ON
   
   StepCounterB = AbschnittDaten[8] | (AbschnittDaten[9]<<8) | (AbschnittDaten[10]<<16) | ((AbschnittDaten[11] & 0x7F)<<24);
   DelayB= AbschnittDaten[12] | ((AbschnittDaten[13]& 0x7F) <<8);
   Serial.printf("***   StepCounterB: %d DelayB: %d\n",StepCounterB,DelayB);
   
   korrekturintervally = AbschnittDaten[14] | ((AbschnittDaten[15] & 0x7F)<<8);
   //korrekturintervally = 0;
   korrekturintervallcountery = 0;
   //   korrekturcountery = 0;
   korrekturintervallcountery = 0;
   
   uint8_t vorzeicheny = (AbschnittDaten[15] & 0x80 )>> 7;
   Serial.printf("***   StepCounterB korrekturintervally: %d vorzeicheny: %d\n",korrekturintervally,vorzeicheny);
   if (vorzeicheny)
   {
      //     Serial.printf("korrekturintervally negativ\n");
      vorzeichen |= (1<<VORZEICHEN_Y);
   }
   else
   {
      vorzeichen &= ~(1<<VORZEICHEN_Y);
   }
   //   Serial.printf("korrekturintervally m VZ: %d vorzeicheny: %d\n",korrekturintervally,vorzeicheny);
   
   if (AbschnittDaten[11] & 0x80) // Bit 7 gesetzt, negative zahl
   {
      //     Serial.printf("Motor B rueckwaerts\n");
      richtung |= (1<<RICHTUNG_B0); // Rueckwarts
      //STEPPERPORT_1 &= ~(1<< MB_RI);
      digitalWriteFast(MB_RI,LOW);
      //lcd_putc('r');
   }
   else 
   {
      //     Serial.printf("Motor B vorwaerts\n");
      richtung &= ~(1<<RICHTUNG_B0);
      //STEPPERPORT_1 |= (1<< MB_RI);
      digitalWriteFast(MB_RI,HIGH);
      //lcd_putc('v');
   }
   CounterB = DelayB;
   // CounterB = 0;
   digitalWriteFast(MB_EN,LOW);
   
   
   // Motor C
   //STEPPERPORT_2 &= ~(1<<MC_EN); // Pololu ON
   digitalWriteFast(MC_EN,LOW);
   //CounterC=0;
   StepCounterC = AbschnittDaten[16] | (AbschnittDaten[17]<<8) | (AbschnittDaten[18]<<16) | ((AbschnittDaten[19] & 0x7F)<<24);
   DelayC= AbschnittDaten[20] | (AbschnittDaten[21]<<8) ;
   
   
     
   //richtung=0;
   if (AbschnittDaten[19] & 0x80) // Bit 7 gesetzt, negative zahl
   {
      richtung |= (1<<RICHTUNG_C); // Rueckwarts
      //STEPPERPORT_2 &= ~(1<< MC_RI);
      digitalWriteFast(MC_RI,LOW);
   }
   else 
   {
      richtung &= ~(1<<RICHTUNG_C);
      //STEPPERPORT_2 |= (1<< MC_RI);
      digitalWriteFast(MC_RI,HIGH);
   }
   
   CounterC = DelayC;
   
   
   /* 
    // Motor D
    //STEPPERPORT_2 &= ~(1<<MD_EN); // Pololu ON
    digitalWriteFast(MD_EN,LOW);
    //CounterD=0;
    dataL=0;
    dataH=0;
    
    delayL = 0;
    delayH = 0;
    
    dataL = AbschnittDaten[10];
    dataH = AbschnittDaten[11];
    
    if (dataH & (0x80)) // Bit 7 gesetzt, negative zahl
    {
    richtung |= (1<<RICHTUNG_D); // Rueckwarts
    //STEPPERPORT_2 &= ~(1<< MD_RI);
    digitalWriteFast(MD_RI,LOW);
    //lcd_putc('r');
    }
    else 
    {
    richtung &= ~(1<<RICHTUNG_D);
    //STEPPERPORT_2 |= (1<< MD_RI);
    digitalWriteFast(MD_RI,HIGH);
    }
    
    dataH &= (0x7F);
    StepCounterD= dataH;      // HByte
    StepCounterD <<= 8;      // shift 8
    StepCounterD += dataL;   // +LByte
    
    delayL=AbschnittDaten[14];
    delayH=AbschnittDaten[15];
    
    DelayD = delayH;
    DelayD <<= 8;
    DelayD += delayL;
    
    CounterD = DelayD;
    
    // pwm-rate
    PWM = AbschnittDaten[20];
    */
   // motorstatus: welcher Motor ist relevant
   motorstatus = AbschnittDaten[28];
   Serial.printf("*** Abschnittladen motorstatus: %d\n",motorstatus); 
   
   if (motorstatus > 3)
   {
      Serial.printf("*** Abschnittladen motorstatus korr\n"); 
      //motorstatus = 1;
   }
   
#pragma mark Ramp
   //   digitalWriteFast(MA_EN,LOW);
   
   if (rampstatus & (1<<RAMPOKBIT))
   {
      Serial.printf("*** Ramp\n");
      rampstepstart =  max(StepCounterA,StepCounterB); // maximalwert
      timerintervall_FAST = TIMERINTERVALL;
      timerintervall_SLOW = RampFaktor * TIMERINTERVALL; // Verlaengerung der delayzeit
      
      ramptimerintervall = timerintervall_SLOW;
      
      delayTimer.update(ramptimerintervall);
      
      if (rampstepstart > RampZeit) // 5mm
      {
         //rampendstep = max(StepCounterA,StepCounterB) -
         rampstatus |= (1<<RAMPSTARTBIT);
         Serial.printf("*** Ramp\n");
      }
      ramptimercounter = 0;
      rampbreite = 0;
   }
   
   
   startTimer2();
   
   Serial.printf("*                                     *** Abschnittladen motorstatus %d richtung: %02X StepCounterA: %d StepCounterB: %d StepCounterC: %d \n",motorstatus,richtung, StepCounterA, StepCounterB,  StepCounterC);
   
   return returnwert;
   
   //OSZI_A_HI;
   
}











gpio_MCP23S17     mcp0(10,0x20);//instance 0 (address A0,A1,A2 tied to 0)
//LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x20
//delay(1000); 
// Add setup code



void thread_func(int inc) 
{
   /*
    lcd.setCursor(0,0);
    lcd.print("A: ");
    
    lcd.setCursor(0,1);
    lcd.print("A:");
    lcd.setCursor(6,1);
    lcd.print("B:");
    
    lcd.setCursor(5,0);
    lcd.print("PWM:");
    */
   while (1)
   {
      if (parallelstatus & (1<<THREAD_COUNT_BIT))
      {
         parallelcounter += 1;
         
       //  lcd.setCursor(0,1);
         //String s = "D";
      //   lcd.setCursor(13,0);
         
         uint16_t rest = Abschnitte - AbschnittCounter;
     //    lcd.print(String(rest));
         
         
         //  s.append(rest);
         /*
          s.append("n ");
          
          s.append(StepCounterA);
          s.append("\n");
          
          lcd.print(s);
          */
         /*
          //lcd.print(String(parallelcounter));
          lcd.setCursor(9,0);
          lcd.print(String(PWM));
          lcd.setCursor(2,1);
          lcd.print(String(StepCounterA));
          lcd.setCursor(8,1);
          lcd.print(String(StepCounterB));
          */
         parallelstatus  &= ~(1<<THREAD_COUNT_BIT);
      }
   }
   /*
    if (sincelastthread >= 500)
    {
    sincelastthread = 0;
    parallelcounter += 2;
    //     lcd.setCursor(12,0);
    //    lcd.print(String(parallelcounter));
    }
    */
}

void A_ISR(void)
{
   digitalWriteFast(MA_EN,HIGH);
}


void setup()
{
   Serial.begin(9600);
   pinMode(LOOPLED, OUTPUT);
   
   
   pinMode(DC_PWM, OUTPUT);
   digitalWriteFast(DC_PWM, HIGH); // OFF
   
   pinMode(STROM, OUTPUT);
   digitalWriteFast(STROM, LOW); // LO, OFF
   
   
   // init Pins
   // Stepper A
   pinMode(MA_STEP, OUTPUT); // 
   pinMode(MA_RI, OUTPUT); // 
   pinMode(MA_EN, OUTPUT); // 
   
   digitalWriteFast(MA_STEP, HIGH); // HI
   digitalWriteFast(MA_RI, HIGH); // HI
   digitalWriteFast(MA_EN, HIGH); // HI
   
   attachInterrupt(digitalPinToInterrupt(END_A0_PIN), A_ISR, FALLING);
   attachInterrupt(digitalPinToInterrupt(END_A1_PIN), A_ISR, FALLING);
   
   
   // Stepper B
   pinMode(MB_STEP, OUTPUT); // HI
   pinMode(MB_RI, OUTPUT); // HI
   pinMode(MB_EN, OUTPUT); // HI
   
   digitalWriteFast(MB_STEP, HIGH); // HI
   digitalWriteFast(MB_RI, HIGH); // HI
   digitalWriteFast(MB_EN, HIGH); // HI
   
   
   
   // Stepper C
   pinMode(MC_STEP, OUTPUT); // HI
   pinMode(MC_RI, OUTPUT); // HI
   pinMode(MC_EN, OUTPUT); // HI
   
   digitalWriteFast(MC_STEP, HIGH); // HI
   digitalWriteFast(MC_RI, HIGH); // HI
   digitalWriteFast(MC_EN, HIGH); // HI
   
   // Stepper D
   pinMode(MD_STEP, OUTPUT); // HI
   pinMode(MD_RI, OUTPUT); // HI
   pinMode(MD_EN, OUTPUT); // HI
   
   digitalWriteFast(MD_STEP, HIGH); // HI
   digitalWriteFast(MD_RI, HIGH); // HI
   digitalWriteFast(MD_EN, HIGH); // HI
   
   pinMode(END_A0_PIN, INPUT); // 
   pinMode(END_A1_PIN, INPUT); // 
   
   pinMode(END_B0_PIN, INPUT); // 
   pinMode(END_B1_PIN, INPUT); // 
   
   pinMode(END_C0_PIN, INPUT); // 
   pinMode(END_C1_PIN, INPUT); // 
   
   
   pinMode(END_A0_PIN, INPUT_PULLUP); // HI
   pinMode(END_A1_PIN, INPUT_PULLUP); // HI
   pinMode(END_B0_PIN, INPUT_PULLUP); // 
   pinMode(END_B1_PIN, INPUT_PULLUP); // HI
   pinMode(END_C0_PIN, INPUT_PULLUP); // 
   pinMode(END_C1_PIN, INPUT_PULLUP); // 
   
   if (TEST)
   {
      pinMode(OSZI_PULS_A, OUTPUT);
      digitalWriteFast(OSZI_PULS_A, HIGH); 
   }
   
   delay(100);
//   lcd.init();
   delay(100);
   //lcd.backlight();
   
   //   rampstatus |=(1<<RAMPOKBIT);
   
   //lcd.setCursor(0,0);
   //lcd.print("hallo");
   delayTimer.begin(delaytimerfunction,timerintervall);
   delayTimer.priority(0);
   
   //   threads.addThread(thread_func, 1);
   
//   lcd.setCursor(0,0);
//   lcd.print("CNC");
   //   lcd.setCursor(0,1);
   //   lcd.print("PWM:");
   /*
    lcd.setCursor(0,1);
    lcd.print("A:");
    lcd.setCursor(6,1);
    lcd.print("B:");
    
    lcd.setCursor(5,0);
    lcd.print("PWM:");
    */
    
 //  anschlagstatus |= (1<<FIRSTRUN);
   if (digitalReadFast(END_A0_PIN) == 0) //  // Schlitten am Anschlag A0
   {
       anschlagstatus |= (1<<END_A0);
     Serial.printf("setup END_A0 set\n");
   }
   if (digitalRead(END_A1_PIN) == 0) //  // Schlitten am Anschlag A1
   {
      anschlagstatus |= (1<< END_A1); // 
      Serial.printf("setup END_A1 set\n");
   }
   
#  pragma mark TeensyStep setup
   // setup the motors 
   motor_A.setMaxSpeed(speedA);     // steps/s
   motor_A.setAcceleration(24000); // steps/s^2 
   motor_A.setStepPinPolarity(LOW);

   motor_B
     .setMaxSpeed(speedB)       // steps/s
     .setAcceleration(24000); // steps/s^2 


    
    motor_C
     //.setPullInSpeed(300)      // steps/s     currently deactivated...
     .setMaxSpeed(speedC)       // steps/s
     .setAcceleration(24000)   // steps/s^2     
     .setStepPinPolarity(LOW); // driver expects active low pulses


   dist1 = 10*spr1;
   dist2 = 10*spr2;
   

} // end setup

// Add loop code
void loop()
{
   //   Serial.println(steps);
   //   threads.delay(1000);
    
   if (sinceblink > 1000) 
   {  
     
      //scanI2C(100000);
      loopLED++;
      sinceblink = 0;
      
      //      lcd.setCursor(0,1);
      //      lcd.print(String(loopLED));
      
      if (digitalRead(LOOPLED) == 1)
      {
         
         //Serial.printf("LED ON\n");
         digitalWriteFast(LOOPLED, 0);
         /*
          //Serial.printf("blink\t %d\n",loopLED);
          lcd.setCursor(0,0);
          //lcd.print("hallo");
          lcd.print(String(timer2Counter));
          lcd.setCursor(10,0);
          lcd.print(String(usb_recv_counter));
          
          lcd.setCursor(16,0);
          lcd.print(String(abschnittnummer));
          lcd.setCursor(0,1);
          lcd.print(String(CounterA&0xFF));
          lcd.setCursor(4,1);
          lcd.print(String(CounterB&0xFF));
          */
          
      }
      else
      {
         digitalWriteFast(LOOPLED, 1);
      }
      parallelcounter += 2;
      //      lcd.setCursor(14,0);
      //      lcd.print(String(parallelcounter));
      
   }// sinceblink
   
   if (sincelaststep > 500)
   {
      uint32_t speed = controller.getCurrentSpeed();
      if (controller.isRunning())
      {
         //Serial.printf("motor running\n");
         /*
         if ((controllerstatus & (1<<STOP)) && (repeatcounter == 0))
             {
            digitalWriteFast(MA_EN,HIGH);
            digitalWriteFast(MB_EN,HIGH);
            digitalWriteFast(MC_EN,HIGH);
            controllerstatus &= ~(1<<RUNNING);
            controllerstatus &= ~(1<<STOP);

            }
          */
      }
      else
      {
         if (controllerstatus & (1<<RUNNING))
         {
         Serial.printf("motor finished\n");
         controller.stopAsync();
         motor_A.setTargetRel(0);
         motor_B.setTargetRel(0);
       
         digitalWriteFast(MA_EN,HIGH);
         digitalWriteFast(MB_EN,HIGH);
         digitalWriteFast(MC_EN,HIGH);
         controllerstatus &= ~(1<<RUNNING);
         
         }

      }
      //Serial.printf("motor getCurrentSpeed: %d\n",speed);
      //Serial.printf("sincelaststep\n");
      sincelaststep = 0;
      //     timerfunction();
      
      //   digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   }
   
   
   
   
#pragma mark start_usb
   
   r = usb_rawhid_recv((void*)buffer, 0); // 1.5us
   code = 0;
   if (r > 0) // 
   {
      noInterrupts();
      
      code = buffer[24];
      Serial.printf("\n----------------------------------->    rawhid_recv code: %02X\n",code);
      usb_recv_counter++;
      //     lcd.setCursor(10,1);
      //     lcd.print(String(usb_recv_counter));
      //     lcd.setCursor(14,1);
      //     lcd.print(String(code));
      uint8_t device = buffer[32];
      sendbuffer[24] =  buffer[32];
      
      sendbuffer[22] = 0xFF; // drillstatus
      sendbuffer[23] = 0xFF; // tabledatastatus
      switch (code)
      {   
         case 0xA4:
         {
            Serial.printf("A4 clear\n");
         }break;
            
#pragma mark A5  GO HOME         
         case 0xA5: //  go home
         {
            Serial.printf("A5\n");
            uint8_t i=0;
            
            for(i=0;i<48;i++) // 5 us ohne printf, 10ms mit printf
            { 
                 Serial.printf("%d \t",CNCDaten[0][i]);
            }
            Serial.printf("\n");
            /*
            for(i=0;i<48;i++) // 5 us ohne printf, 10ms mit printf
            { 
               Serial.printf("%d \t",CNCDaten[1][i]);
            }
            Serial.printf("\n");
             */
            abschnittnummer = 0;
            //Serial.printf("go home CNC_Daten: %d\n",);
            // ohne Ringbuffer-task
            AbschnittLaden_4M(buffer);
 //           endposition=abschnittnummer; // erster ist letzter Abschnitt
         }break;
#pragma mark B1    
         case 0xB1:
         {
            Serial.printf("B1 Joystick\n");
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            Serial.printf("B1 abschnittnummer: %d\n",abschnittnummer);
            
            AbschnittLaden_4M(buffer);
            if (abschnittnummer == 0)
            {
               
            }
            
            sendbuffer[8]=ladeposition & 0x00FF;
            
            sendbuffer[10]=(endposition & 0xFF00) >> 8;
            sendbuffer[11]=(endposition & 0x00FF);
            
            
            sendbuffer[0]=0xB2;
            //          usb_rawhid_send((void*)sendbuffer, 50);
            
         }break;
#pragma mark B3       
         case 0xB3: // sendTextdaten: nur 1 Abschnitt
         {
            Serial.printf("* B3 Device: %d* Data: \n",device);
            uint8_t i=0;
            for(i=0;i<48;i++) // 5 us ohne printf, 10ms mit printf
            { 
               Serial.printf("%d \t",buffer[i]);
            }
            Serial.printf("\n");
            sendbuffer[24] =  buffer[32];
            
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            Serial.printf("indexh: %d indexl: %d\n",indexh,indexl);
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            //Serial.printf("abschnittnummer: *%d*\n",abschnittnummer);
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            // Lage:
            
            uint8_t lage = buffer[25];
            
            Serial.printf("B3 abschnittnummer: %d\tbuffer25 lage: %d \t device: %d\n",abschnittnummer,lage,device);
            //             Serial.printf("count: %d\n",buffer[22]);
            if (abschnittnummer==0)  // Start
            {
               //noInterrupts();
               Serial.printf("B3 abschnittnummer 0\n");
               //Serial.printf("B3 abschnittnummer 0 \tbuffer25 lage: %d \t buffer32 device: %d\n",buffer[25],buffer[32]);
               //             Serial.printf("count: %d\n",buffer[22]);
               PWM= buffer[29];
               //              lcd.print(String(PWM));
               
               ladeposition=0;
               //globalaktuelleladeposition = 0;
               aktuelleladeposition = 0;
               endposition=0xFFFF;
               cncstatus = 0;
               sendstatus = 0;
               motorstatus = 0;
               korrekturcounterx = 0;
               korrekturcountery = 0;
               ringbufferstatus=0x00;
               //anschlagstatus=0;
               ringbufferstatus |= (1<<FIRSTBIT);
               ringbufferstatus |= (1<<STARTBIT);
               AbschnittCounter=0;
               //sendbuffer[8]= versionintl;
               //sendbuffer[8]= versioninth;
               sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
               sendbuffer[6]=abschnittnummer & 0x00FF;
               
               //lcd_gotoxy(0,0);
               sendbuffer[14] = (TIMERINTERVALL & 0xFF00)>>8;
               sendbuffer[15] = (TIMERINTERVALL & 0x00FF);
               sendbuffer[8] = ladeposition;
               sendbuffer[0]=0xD1;
               //               Serial.printf("------------------------------------->  first abschnitt, endposition: %d\n",endposition);
               
               /*
                if (code == 0xF0) // cncstatus fuer go_home setzen
                {
                
                sendbuffer[0]=0x45;
                
                cncstatus |= (1<<GO_HOME); // Bit fuer go_home setzen
                usb_rawhid_send((void*)sendbuffer, 50);
                }
                else if (code == 0xF1)
                {
                sendbuffer[0]=0x44;
                cncstatus &= ~(1<<GO_HOME); // Bit fuer go_home zuruecksetzen
                usb_rawhid_send((void*)sendbuffer, 50);
                }
                */
               // Abschnitt 0 melden
               //              usb_rawhid_send((void*)sendbuffer, 50);
               
               //               startTimer2();
               //               interrupts();
               // Serial.printf("------------------------------------->  first abschnitt end\n");
            }
            else // Abschnittnummer > 0
            {
               Serial.printf("XX\n");
               // Ablauf schon gestartert
               //          Serial.printf("  -----                  B3 Ablauf gestartet, abschnittnummer: %d\n",abschnittnummer);
               //lcd.setCursor(12,0);
               //lcd.print(String(abschnittnummer));
               
            }
            
            
            // lage im Ablauf: 
            // 1: erster Abschnitt
            // 2: letzter Abschnitt
            // 0: innerer Abschnitt
            
            if (buffer[25]& 0x02)// letzter Abschnitt
            {
               //              Serial.printf("------------------------  last abschnitt\n");
               ringbufferstatus |= (1<<LASTBIT); // letzter Abschnitt
               if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
               {
                  // endposition setzen
                  //                Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                  endposition=abschnittnummer; // erster ist letzter Abschnitt
                  
                  //               Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
               }
               
            }
            
            uint8_t pos=(abschnittnummer);
            
            pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
            Serial.printf("B3:  abschnittnummer: %d endposition: %d pos: %d\n",abschnittnummer,endposition,pos);
            //if (abschnittnummer>8)
            
            {
               //lcd_putint1(pos);
            }
            
            // Daten laden in ringbuffer an Position pos
            //          uint8_t i=0;
            /*
             for(i=0;i<10;i++)
             {
             Serial.printf("%d\t",i);
             }
             */
            //Serial.printf("\n");
            //OSZI_A_LO();
            //              Serial.printf("default: abschnittnummer: %d pos: %d\n",abschnittnummer,pos);
            //for(i=0;i<USB_DATENBREITE;i++) // 5 us ohne printf, 10ms mit printf
            for(i=0;i<64;i++) // 5 us ohne printf, 10ms mit printf
            { 
               //                 Serial.printf("%d \t",buffer[i]);
               CNCDaten[pos][i]=buffer[i];  
            }
            interrupts();
         }break;
#pragma mark B4 
         case 0xB4: // PCB neu: Pfeiltasten
         {
            Serial.printf("* B4 Device: %d* Data: \n",device);
            uint8_t i=0;
            for(i=0;i<48;i++) // 5 us ohne printf, 10ms mit printf
            { 
               Serial.printf("%d \t",buffer[i]);
            }
            Serial.printf("\n");
            sendbuffer[24] =  buffer[32];
            
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            Serial.printf("indexh: %d indexl: %d\n",indexh,indexl);
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            //Serial.printf("abschnittnummer: *%d*\n",abschnittnummer);
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            // Lage:
            
            uint8_t lage = buffer[25];
            
            Serial.printf("B4 abschnittnummer: %d\tbuffer25 lage: %d \t device: %d\n",abschnittnummer,lage,device);
            //             Serial.printf("count: %d\n",buffer[22]);
            if (abschnittnummer==0)  // Start
            {
               //noInterrupts();
               Serial.printf("B4 abschnittnummer 0\n");
               //Serial.printf("B3 abschnittnummer 0 \tbuffer25 lage: %d \t buffer32 device: %d\n",buffer[25],buffer[32]);
               //             Serial.printf("count: %d\n",buffer[22]);
               PWM= buffer[29];
               //              lcd.print(String(PWM));
               
               ladeposition=0;
               //globalaktuelleladeposition = 0;
               aktuelleladeposition = 0;
               endposition=0xFFFF;
               cncstatus = 0;
               sendstatus = 0;
               motorstatus = 0;
               korrekturcounterx = 0;
               korrekturcountery = 0;
               ringbufferstatus=0x00;
               //anschlagstatus=0;
               ringbufferstatus |= (1<<FIRSTBIT);
               ringbufferstatus |= (1<<STARTBIT);
               AbschnittCounter=0;
               //sendbuffer[8]= versionintl;
               //sendbuffer[8]= versioninth;
               sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
               sendbuffer[6]=abschnittnummer & 0x00FF;
               
               //lcd_gotoxy(0,0);
               sendbuffer[14] = (TIMERINTERVALL & 0xFF00)>>8;
               sendbuffer[15] = (TIMERINTERVALL & 0x00FF);
               sendbuffer[8] = ladeposition;
               sendbuffer[0]=0xD1;
               //               Serial.printf("------------------------------------->  first abschnitt, endposition: %d\n",endposition);
               
               // Abschnitt 0 melden
               //              usb_rawhid_send((void*)sendbuffer, 50);
               
               //               startTimer2();
               //               interrupts();
               // Serial.printf("------------------------------------->  first abschnitt end\n");
            }
            else // Abschnittnummer > 0
            {
               Serial.printf("XX\n");
               // Ablauf schon gestartert
               //          Serial.printf("  -----                  B3 Ablauf gestartet, abschnittnummer: %d\n",abschnittnummer);
               //lcd.setCursor(12,0);
               //lcd.print(String(abschnittnummer));
               
            }
            
            
            // lage im Ablauf: 
            // 1: erster Abschnitt
            // 2: letzter Abschnitt
            // 0: innerer Abschnitt
            
            if (buffer[25]& 0x02)// letzter Abschnitt
            {
               //              Serial.printf("------------------------  last abschnitt\n");
               ringbufferstatus |= (1<<LASTBIT); // letzter Abschnitt
               if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
               {
                  // endposition setzen
                  //                Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                  endposition=abschnittnummer; // erster ist letzter Abschnitt
                  
                  //               Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
               }
               
            }
            
            uint8_t pos=(abschnittnummer);
            
            pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
            Serial.printf("B4:  abschnittnummer: %d endposition: %d pos: %d\n",abschnittnummer,endposition,pos);
            //if (abschnittnummer>8)
            {
               //lcd_putint1(pos);
            }
            
            // Daten laden in ringbuffer an Position pos
            //          uint8_t i=0;
            /*
             for(i=0;i<10;i++)
             {
             Serial.printf("%d\t",i);
             }
             */
            //Serial.printf("\n");
            //OSZI_A_LO();
            //              Serial.printf("default: abschnittnummer: %d pos: %d\n",abschnittnummer,pos);
            //for(i=0;i<USB_DATENBREITE;i++) // 5 us ohne printf, 10ms mit printf
            for(i=0;i<64;i++) // 5 us ohne printf, 10ms mit printf
            { 
               //                 Serial.printf("%d \t",buffer[i]);
               CNCDaten[pos][i]=buffer[i];  
            }
            interrupts();
         }break; // B4

            
#pragma mark B5               
         case 0xB5: // PCB neu
         {
            Serial.printf("***   B5 Device: %d*\n",device);
            
            sendbuffer[0]=0xB6;
            uint8_t i=0;
            for(i=0;i<48;i++) // 5 us ohne printf, 10ms mit printf
            { 
               //                 Serial.printf("%d \t",buffer[i]);
               CNCDaten[0][i]=buffer[i];  
            }
            sendbuffer[24] =  buffer[32];
            
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            //   Serial.printf("indexh: %d indexl: %d\n",indexh,indexl);
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            //   Serial.printf("abschnittnummer: *%d*\n",abschnittnummer);
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            usb_rawhid_send((void*)sendbuffer, 100);
            
            if (abschnittnummer==0)  // Start
            {
               //noInterrupts();
               Serial.printf("B5 abschnittnummer 0 \tbuffer25 lage: %d \t buffer32 device: %d\n",buffer[25],buffer[32]);
               //             Serial.printf("count: %d\n",buffer[22]);
               PWM= buffer[29];
               //              lcd.print(String(PWM));
               
               ladeposition=0;
               //globalaktuelleladeposition = 0;
               aktuelleladeposition = 0;
               endposition=0xFFFF;
               cncstatus = 0;
               sendstatus = 0;
               motorstatus = 0;
               korrekturcounterx = 0;
               korrekturcountery = 0;
               ringbufferstatus=0x00;
               //anschlagstatus=0;
               ringbufferstatus |= (1<<FIRSTBIT);
               ringbufferstatus |= (1<<STARTBIT);
               AbschnittCounter=0;
               //sendbuffer[8]= versionintl;
               //sendbuffer[8]= versioninth;
               sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
               sendbuffer[6]=abschnittnummer & 0x00FF;
               
               //lcd_gotoxy(0,0);
               sendbuffer[14] = (TIMERINTERVALL & 0xFF00)>>8;
               sendbuffer[15] = (TIMERINTERVALL & 0x00FF);
               sendbuffer[8] = ladeposition;
               {
                  sendbuffer[0]=0xB6;
               }
               
            }
            if (buffer[25]& 0x02)// letzter Abschnitt
            {
               Serial.printf("------------------------ B5  last abschnitt\n");
               ringbufferstatus |= (1<<LASTBIT); // letzter Abschnitt
               if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
               {
                  // endposition setzen
                  //                     Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                  endposition=abschnittnummer; // erster ist letzter Abschnitt
                  
                  //                      Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
               }
               
            }
          }break;
            
            
 #pragma mark B7             
         case 0xB7: // report_move_Drill (wie BA)
         {
            Serial.printf("\nB7 \n");
            sendbuffer[0]=0xB9;
            sendbuffer[24] =  buffer[32];
           
            uint8_t i=0;
            for(i=0;i<48;i++) // 5 us ohne printf, 10ms mit printf
            { 
               Serial.printf("%d \t",buffer[i]);
               CNCDaten[0][i]=buffer[i];  
            }
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            Serial.printf("indexh: %d indexl: %d\n",indexh,indexl);
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            Serial.printf("BA abschnittnummer: *%d*\n",abschnittnummer);
            drillstatus = buffer[33];
            Serial.printf("B7 Drillaktion drillstatus new: %d\n",drillstatus);

            drillstatus = 0xA0; // keine rueckweg
            tabledatastatus = 0xFF;
            sendbuffer[22] = drillstatus;
            if (abschnittnummer == 0)
            {
               ringbufferstatus=0x00;
               //anschlagstatus=0;
               ringbufferstatus |= (1<<FIRSTBIT);
               ringbufferstatus |= (1<<STARTBIT);
               
            }
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            // von default:
            if (buffer[25]& 0x02)// letzter Abschnitt
            {
               //    Serial.printf("------------------------  last abschnitt\n");
               ringbufferstatus |= (1<<LASTBIT); // letzter Abschnitt
               if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
               {
                  // endposition setzen
                  //                     Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                  endposition=abschnittnummer; // erster ist letzter Abschnitt
                  
                  //                      Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
               }
               
            }
            
            //          uint8_t drill_lage = AbschnittLaden_4M(buffer);
            
            //          Serial.printf("BA drill_lage: *%d*\n",drill_lage);
            
            //usb_rawhid_send((void*)sendbuffer, 50);
            Serial.printf("         B7 END\n\n");
            
         }break;
            
#pragma mark                    BA             
         case 0xBA: // Drillaktion
         {
            Serial.printf("\n\nBA Drillaktion drillstatus old: %d\n",drillstatus);
            sendbuffer[0]=0xBD;
            
            drillstatus = buffer[33];
            Serial.printf("BA Drillaktion drillstatus new: %d\n",drillstatus);
            if (buffer[25] == 0xFF)
            {
               Serial.printf("BA Drillaktion fertig\n");
               break;
            }
            
            
            
            sendbuffer[22] = drillstatus;
         
            
            uint8_t i=0;
            for(i=0;i<48;i++) // 5 us ohne printf, 10ms mit printf
            { 
               Serial.printf("%d \t",buffer[i]);
               CNCDaten[0][i]=buffer[i];  
            }

            Serial.printf("\n");
            sendbuffer[24] =  buffer[32];

            
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            Serial.printf("indexh: %d indexl: %d\n",indexh,indexl);
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            Serial.printf("BA abschnittnummer: *%d*\n",abschnittnummer);
            
            if (abschnittnummer == 0)
            {
               ringbufferstatus=0x00;
               //anschlagstatus=0;
               ringbufferstatus |= (1<<FIRSTBIT);
               ringbufferstatus |= (1<<STARTBIT);

            }
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            // von default:
            if (buffer[25]& 0x02)// letzter Abschnitt
            {
               //    Serial.printf("------------------------  last abschnitt\n");
               ringbufferstatus |= (1<<LASTBIT); // letzter Abschnitt
               if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
               {
                  // endposition setzen
                  //                     Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                  endposition=abschnittnummer; // erster ist letzter Abschnitt
                  
                  //                      Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
               }
               
            }
            
  //          uint8_t drill_lage = AbschnittLaden_4M(buffer);
            
  //          Serial.printf("BA drill_lage: *%d*\n",drill_lage);
            
            //usb_rawhid_send((void*)sendbuffer, 50);
            Serial.printf("         BA END\n\n");
         }break;
            
#pragma mark   BC                  TeensyStep           
         case 0xBC:
         {
            repeatcounter = 0; // reset
            Serial.printf("* TeensyStep BC Device: %d* Data: \n",device);
            uint8_t i=0;
            for(i=0;i<48;i++) // 5 us ohne printf, 10ms mit printf
            { 
               Serial.printf("%d \t",buffer[i]);
            }
            Serial.printf("\n");
            sendbuffer[24] =  buffer[32];
            
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            Serial.printf("indexh: %d indexl: %d\n",indexh,indexl);
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            //Serial.printf("abschnittnummer: *%d*\n",abschnittnummer);
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            // Lage:
            
            uint8_t lage = buffer[25];
            
            Serial.printf("B3 abschnittnummer: %d\tbuffer25 lage: %d \t device: %d\n",abschnittnummer,lage,device);
            //             Serial.printf("count: %d\n",buffer[22]);
            if (abschnittnummer==0)  // Start
            {
               //noInterrupts();
               Serial.printf("BC abschnittnummer 0\n");
               //Serial.printf("B3 abschnittnummer 0 \tbuffer25 lage: %d \t buffer32 device: %d\n",buffer[25],buffer[32]);
               //             Serial.printf("count: %d\n",buffer[22]);
               PWM= buffer[29];
               //              lcd.print(String(PWM));
               
               ladeposition=0;
               //globalaktuelleladeposition = 0;
               aktuelleladeposition = 0;
               endposition=0xFFFF;
               cncstatus = 0;
               sendstatus = 0;
               motorstatus = 0;
               korrekturcounterx = 0;
               korrekturcountery = 0;
               ringbufferstatus=0x00;
               //anschlagstatus=0;
               ringbufferstatus |= (1<<FIRSTBIT);
               ringbufferstatus |= (1<<STARTBIT);
               AbschnittCounter=0;
               //sendbuffer[8]= versionintl;
               //sendbuffer[8]= versioninth;
               sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
               sendbuffer[6]=abschnittnummer & 0x00FF;
               
               //lcd_gotoxy(0,0);
               sendbuffer[14] = (TIMERINTERVALL & 0xFF00)>>8;
               sendbuffer[15] = (TIMERINTERVALL & 0x00FF);
               sendbuffer[8] = ladeposition;
               sendbuffer[0]=0xBD;
               //               Serial.printf("------------------------------------->  first abschnitt, endposition: %d\n",endposition);
               
               /*
                if (code == 0xF0) // cncstatus fuer go_home setzen
                {
                
                sendbuffer[0]=0x45;
                
                cncstatus |= (1<<GO_HOME); // Bit fuer go_home setzen
                usb_rawhid_send((void*)sendbuffer, 50);
                }
                else if (code == 0xF1)
                {
                sendbuffer[0]=0x44;
                cncstatus &= ~(1<<GO_HOME); // Bit fuer go_home zuruecksetzen
                usb_rawhid_send((void*)sendbuffer, 50);
                }
                */
               // Abschnitt 0 melden
               usb_rawhid_send((void*)sendbuffer, 50);
               
               //               startTimer2();
               //               interrupts();
               // Serial.printf("------------------------------------->  first abschnitt end\n");
            }
            else // Abschnittnummer > 0
            {
               Serial.printf("XX\n");
               // Ablauf schon gestartert
               //          Serial.printf("  -----                  B3 Ablauf gestartet, abschnittnummer: %d\n",abschnittnummer);
               //lcd.setCursor(12,0);
               //lcd.print(String(abschnittnummer));
               
            }
            
            
            // lage im Ablauf: 
            // 1: erster Abschnitt
            // 2: letzter Abschnitt
            // 0: innerer Abschnitt
            
            if (buffer[25]& 0x02)// letzter Abschnitt
            {
               //              Serial.printf("------------------------  last abschnitt\n");
               ringbufferstatus |= (1<<LASTBIT); // letzter Abschnitt
               if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
               {
                  // endposition setzen
                  //                Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                  endposition=abschnittnummer; // erster ist letzter Abschnitt
                  
                  //               Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
               }
               
            }
            
            uint8_t pos=(abschnittnummer);
            
            pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
            Serial.printf("B3:  abschnittnummer: %d endposition: %d pos: %d\n",abschnittnummer,endposition,pos);
            //if (abschnittnummer>8)
            
            {
               //lcd_putint1(pos);
            }
            
            // Daten laden in ringbuffer an Position pos
            //          uint8_t i=0;
            /*
             for(i=0;i<10;i++)
             {
             Serial.printf("%d\t",i);
             }
             */
            //Serial.printf("\n");
            //OSZI_A_LO();
            //              Serial.printf("default: abschnittnummer: %d pos: %d\n",abschnittnummer,pos);
            //for(i=0;i<USB_DATENBREITE;i++) // 5 us ohne printf, 10ms mit printf
            for(i=0;i<64;i++) // 5 us ohne printf, 10ms mit printf
            { 
               //                 Serial.printf("%d \t",buffer[i]);
               CNCDaten[pos][i]=buffer[i];  
            }
            interrupts();
         }break;
            
#pragma mark   DC            Pfeiltaste      TeensyStep           
         case 0xDC:
         {
            Serial.printf("* TeensyStep DC Device: %d* Data: \n",device);
            uint8_t i=0;
            for(i=0;i<48;i++) // 5 us ohne printf, 10ms mit printf
            { 
               Serial.printf("%d \t",buffer[i]);
            }
            Serial.printf("\n");
            sendbuffer[24] =  buffer[32];
            
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            Serial.printf("indexh: %d indexl: %d\n",indexh,indexl);
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            //Serial.printf("abschnittnummer: *%d*\n",abschnittnummer);
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            // Lage:
            
            uint8_t lage = buffer[25];
            
            uint8_t mausstatus = buffer[37];
            if (mausstatus & (1<<1)) // bit gesetzt
            {
               repeatcounter++;
            }
            else 
            {
               repeatcounter = 0; // mouseup
               Serial.printf("DC repeatcounter 0: %d\n",repeatcounter);
               digitalWriteFast(MA_EN,HIGH);
               digitalWriteFast(MB_EN,HIGH);
               controller.stopAsync();
               motor_A.setTargetRel(0);
               motor_B.setTargetRel(0);


            }
            Serial.printf("DC  **********  mausstatus(37) %d repeatcounter: %d\n",mausstatus,repeatcounter);
                          
                          
            //Serial.printf("DC abschnittnummer: %d\tbuffer(25) lage: %d \t device: %d mausstatus(37): %d\n",abschnittnummer,lage,device,mausstatus);
            //             Serial.printf("count: %d\n",buffer[22]);
            if (abschnittnummer==0)  // Start
            {
               //noInterrupts();
               //Serial.printf("DC abschnittnummer 0\n");
               //Serial.printf("DC abschnittnummer 0 \tbuffer25 lage: %d \t buffer32 device: %d\n",buffer[25],buffer[32]);
               //             Serial.printf("count: %d\n",buffer[22]);
               //PWM= buffer[29];
               //              lcd.print(String(PWM));
               
               ladeposition=0;
               //globalaktuelleladeposition = 0;
               aktuelleladeposition = 0;
               endposition=0xFFFF;
               cncstatus = 0;
               sendstatus = 0;
               motorstatus = 0;
               korrekturcounterx = 0;
               korrekturcountery = 0;
               ringbufferstatus=0x00;
               //anschlagstatus=0;
               ringbufferstatus |= (1<<FIRSTBIT);
               ringbufferstatus |= (1<<STARTBIT);
               AbschnittCounter=0;
               //sendbuffer[8]= versionintl;
               //sendbuffer[8]= versioninth;
               sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
               sendbuffer[6]=abschnittnummer & 0x00FF;
               
               //lcd_gotoxy(0,0);
               sendbuffer[14] = (TIMERINTERVALL & 0xFF00)>>8;
               sendbuffer[15] = (TIMERINTERVALL & 0x00FF);
               sendbuffer[8] = ladeposition;
               sendbuffer[0]=0xDC;
               //               Serial.printf("------------------------------------->  first abschnitt, endposition: %d\n",endposition);
               
               /*
                if (code == 0xF0) // cncstatus fuer go_home setzen
                {
                
                sendbuffer[0]=0x45;
                
                cncstatus |= (1<<GO_HOME); // Bit fuer go_home setzen
                usb_rawhid_send((void*)sendbuffer, 50);
                }
                else if (code == 0xF1)
                {
                sendbuffer[0]=0x44;
                cncstatus &= ~(1<<GO_HOME); // Bit fuer go_home zuruecksetzen
                usb_rawhid_send((void*)sendbuffer, 50);
                }
                */
               // Abschnitt 0 melden
               
               usb_rawhid_send((void*)sendbuffer, 50);
               
               //               startTimer2();
               //               interrupts();
               // Serial.printf("------------------------------------->  first abschnitt end\n");
            }
            else // Abschnittnummer > 0
            {
               Serial.printf("XX\n");
               // Ablauf schon gestartert
               //          Serial.printf("  -----                  B3 Ablauf gestartet, abschnittnummer: %d\n",abschnittnummer);
               //lcd.setCursor(12,0);
               //lcd.print(String(abschnittnummer));
               
            }
            
            
            // lage im Ablauf: 
            // 1: erster Abschnitt
            // 2: letzter Abschnitt
            // 0: innerer Abschnitt
            
            if (buffer[25]& 0x02)// letzter Abschnitt
            {
               //              Serial.printf("------------------------  last abschnitt\n");
               ringbufferstatus |= (1<<LASTBIT); // letzter Abschnitt
               if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
               {
                  // endposition setzen
                  //                Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                  endposition=abschnittnummer; // erster ist letzter Abschnitt
                  
                  //               Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
               }
               
            }
            
            uint8_t pos=(abschnittnummer);
            
            pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
            Serial.printf("DC:  abschnittnummer: %d endposition: %d pos: %d\n",abschnittnummer,endposition,pos);
            //if (abschnittnummer>8)
            
            {
               //lcd_putint1(pos);
            }
            
            // Daten laden in ringbuffer an Position pos
            //          uint8_t i=0;
            /*
             for(i=0;i<10;i++)
             {
             Serial.printf("%d\t",i);
             }
             */
            //Serial.printf("\n");
            //OSZI_A_LO();
            //              Serial.printf("default: abschnittnummer: %d pos: %d\n",abschnittnummer,pos);
            //for(i=0;i<USB_DATENBREITE;i++) // 5 us ohne printf, 10ms mit printf
            for(i=0;i<64;i++) // 5 us ohne printf, 10ms mit printf
            { 
               //                 Serial.printf("%d \t",buffer[i]);
               CNCDaten[pos][i]=buffer[i];  
            }
            interrupts();
         }break;

            
#pragma mark                    CA             
         case 0xCA: // goto Zeile Kopie BA
         {
            Serial.printf("\n\nCA dataTableaktion \n");
            sendbuffer[0]=0xCB;
            tabledatastatus = buffer[38];
            
            Serial.printf("CA tataTableaktion tabledatastatus: %d\n",tabledatastatus);
            tablezeile = (buffer[34]) << 8 + buffer[35];
            
            Serial.printf("CA tataTableaktion tablezeile: %d\n",tablezeile);
            if (tablezeile == 0xFFFF)
            {
               break;
            }
            {
               sendbuffer[23] = tabledatastatus;
               
            }
            
            uint8_t i=0;
            for(i=0;i<48;i++) // 5 us ohne printf, 10ms mit printf
            { 
               Serial.printf("%d \t",buffer[i]);
               CNCDaten[0][i]=buffer[i];  
            }
            
            Serial.printf("\n");
            sendbuffer[24] =  buffer[32];
            
            
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            Serial.printf("indexh: %d indexl: %d\n",indexh,indexl);
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            Serial.printf("BA abschnittnummer: *%d*\n",abschnittnummer);
            
            if (abschnittnummer == 0)
            {
               ringbufferstatus=0x00;
               //anschlagstatus=0;
               ringbufferstatus |= (1<<FIRSTBIT);
               ringbufferstatus |= (1<<STARTBIT);
               
            }
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            // von default:
            if (buffer[25]& 0x02)// letzter Abschnitt
            {
               //    Serial.printf("------------------------  last abschnitt\n");
               ringbufferstatus |= (1<<LASTBIT); // letzter Abschnitt
               if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
               {
                  // endposition setzen
                  //                     Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                  endposition=abschnittnummer; // erster ist letzter Abschnitt
                  
                  //                      Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
               }
               
            }
             //          Serial.printf("BA drill_lage: *%d*\n",drill_lage);
            
            //usb_rawhid_send((void*)sendbuffer, 50);
            Serial.printf("         CA END\n\n");
         }break;
            
         case 0xE0: // Man: Alles stoppen
         {
            Serial.printf("E0 Stop\n");
            ringbufferstatus = 0;
            motorstatus=0;
            //anschlagstatus = 0;
            cncstatus = 0;
            sendbuffer[0]=0xE1;
            
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            sendbuffer[8]=ladeposition & 0x00FF;
            // sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
            
            
            
            usb_rawhid_send((void*)sendbuffer, 100);
            
            sendbuffer[0]=0x00;
            sendbuffer[5]=0x00;
            sendbuffer[6]=0x00;
            sendbuffer[8]=0x00;
            
            ladeposition=0;
            sendbuffer[8]=ladeposition;
            endposition=0xFFFF;
            
            AbschnittCounter=0;
            PWM = sendbuffer[29];
            //CMD_PORT &= ~(1<<DC_PWM);
            
            
            
            digitalWriteFast(DC_PWM,HIGH);
            
            
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            
            CounterA=0;
            CounterB=0;
            CounterC=0;
            CounterD=0;
            
            /*
             STEPPERPORT_1 |= (1<<MA_EN); // Pololu OFF
             STEPPERPORT_1 |= (1<<MB_EN); // Pololu OFF
             STEPPERPORT_2 |= (1<<MC_EN); // Pololu OFF
             STEPPERPORT_2 |= (1<<MD_EN); // Pololu OFF
             */
            digitalWriteFast(MA_EN,HIGH);
            digitalWriteFast(MB_EN,HIGH);
            digitalWriteFast(MC_EN,HIGH);
            digitalWriteFast(MD_EN,HIGH);
            //lcd.setCursor(0,1);
            //lcd.print("HALT");
            
            // lcd_gotoxy(0,1);
            // lcd_puts("HALT\0");
            Serial.printf("E0 Stop END\n");
         }break;
            
            
         case 0xE2: // DC_PWM ON_OFF: Temperatur Schneiddraht setzen
         {
            
            PWM = buffer[20];
            Serial.printf("E2 setPWM: %d\n",PWM);
            if (PWM==0) // OFF
            {
               //CMD_PORT &= ~(1<<DC_PWM);
               digitalWriteFast(DC_PWM,LOW);
            }
            parallelstatus |= (1<<THREAD_COUNT_BIT);
            
            sendbuffer[0]=0xE3;
            //          usb_rawhid_send((void*)sendbuffer, 50);
            //sendbuffer[0]=0x00;
            //sendbuffer[5]=0x00;
            //sendbuffer[8]=0x00;
            
         }break;
            
            
         case 0xE4: // Stepperstrom ON_OFF
         {
            Serial.printf("E4 ON\n");
            if (buffer[8])
            {
               //CMD_PORT |= (1<<STROM); // ON
               digitalWriteFast(STROM,HIGH);
               PWM = buffer[29];
            }
            else
            {
               //CMD_PORT &= ~(1<<STROM); // OFF
               digitalWriteFast(STROM,LOW);
               PWM = 0;
            }
            
            if (PWM==0)
            {
               //CMD_PORT &= ~(1<<DC_PWM);
               digitalWriteFast(DC_PWM,LOW);
            }
            
            
            sendbuffer[0]=0xE5;
            //          usb_rawhid_send((void*)sendbuffer, 50);
            //sendbuffer[0]=0x00;
            //sendbuffer[5]=0x00;
            //sendbuffer[6]=0x00;
            
         }break;
            
            
            
            
            
            
            
#pragma mark F1 reset 
         case 0xF1: // reset
         {
            Serial.printf("F1 reset\n");
            uint8_t i=0, k=0;
            for (k=0;k<RINGBUFFERTIEFE;k++)
            {
               for(i=0;i<USB_DATENBREITE;i++)
               {
                  CNCDaten[k][i]=0;  
               }
            }
            drillstatus = 0;
            
            
            ringbufferstatus = 0;
            motorstatus=0;
            anschlagstatus = 0;
            
            cncstatus = 0;
            ladeposition=0;
            endposition=0xFFFF;
            
            AbschnittCounter=0;
            PWM = 0;
            //CMD_PORT &= ~(1<<DC_PWM);
            digitalWriteFast(DC_PWM,LOW);
            
            StepCounterA=0;
            StepCounterB=0;
            StepCounterC=0;
            StepCounterD=0;
            
            CounterA=0;
            CounterB=0;
            CounterC=0;
            CounterD=0;
            
            korrekturcounterx = 0;
            korrekturcountery = 0;
            
            digitalWriteFast(MA_EN,HIGH);
            digitalWriteFast(MB_EN,HIGH);
            
            digitalWriteFast(MA_STEP,HIGH);
            digitalWriteFast(MB_STEP,HIGH);
            
            //           lcd.setCursor(0,1);
            //           lcd.print("reset\n");
            //cli();
            //usb_init();
            /*
             while (!usb_configured()) // wait  ;
             
             // Wait an extra second for the PC's operating system to load drivers
             // and do whatever it does to actually be ready for input
             _delay_ms(1000);
             */
            //sei();
            sendbuffer[0]=0xF2;
            //         usb_rawhid_send((void*)sendbuffer, 50);
            sendbuffer[0]=0x00;
            
         }break;
            
#pragma mark default
         default:
         {
            Serial.printf("\n---  usb_recv default code: %02X\n",code);
            //          Serial.printf("\n---  usb_recv_counter %d\t default \nringbufferstatus: %02X position(buffer17): %02X\n",usb_recv_counter,ringbufferstatus, buffer[17]);
            // Abschnittnummer bestimmen
            
            sendbuffer[24] =  buffer[32];
            
            uint8_t indexh=buffer[26];
            uint8_t indexl=buffer[27];
            
            abschnittnummer= indexh<<8;
            abschnittnummer += indexl;
            
            // Lage:
            
            uint8_t lage = buffer[25];
            // lage im Ablauf: 
            // 1: erster Abschnitt
            // 2: letzter Abschnitt
            // 3: nur 1 Abschnitt
            // 0: innerer Abschnitt
            
            //OSZI_A_LO();
            //            Serial.printf("\n\ndefault abschnittnummer: %d  lage: %d code: %d\n",abschnittnummer,lage,code); // 50 us
            //            for(int i=0;i<48;i++)
            {
               //              Serial.printf("%d\t",buffer[i]);
            }
            //           Serial.printf("\n");
            
            //           Serial.printf("\n****************************************\n");
            //           Serial.printf("default Abschnitt lage: %d abschnittnummer: %d\n",lage,abschnittnummer);
            //           Serial.printf("****************************************\n");
            
            
            //Serial.printf("schritteX: \t%d \tschritteY: \t%d\n",schritteX,schritteY);
            //OSZI_A_HI();
            //         lcd.setCursor(5,1);
            //         lcd.print(String(abschnittnummer));
            
            
            //          sendbuffer[0]=0x36;
            //          sendbuffer[5]=abschnittnummer;
            //          sendbuffer[6]=buffer[16];
            
            //           usb_rawhid_send((void*)sendbuffer, 50); // nicht jedes Paket melden
            
            switch (device)
            {
               case DEVICE_MILL:
               {
                  //                                Serial.printf("default device 1 code: %d\n",code);
                  
#pragma mark default DEVICE_MILL abschnittnummer 0
                  
                  if (abschnittnummer==0)  // Start
                  {
                     //                   //noInterrupts();
                     //                   Serial.printf("abschnittnummer 0 \t25: %d \t 32: %d\n",buffer[25],buffer[32]);
                     sendbuffer[24] =  buffer[32];  
                     //             Serial.printf("count: %d\n",buffer[22]);
                      PWM= buffer[29];
                     
                     ladeposition=0;
                     //globalaktuelleladeposition = 0;
                     aktuelleladeposition = 0;
                     endposition=0xFFFF;
                     cncstatus = 0;
                     sendstatus = 0;
                     motorstatus = 0;
                     ringbufferstatus=0x00;
                     //anschlagstatus=0;
                     ringbufferstatus |= (1<<FIRSTBIT);
                     ringbufferstatus |= (1<<STARTBIT);
                     AbschnittCounter=0;
                     //sendbuffer[8]= versionintl;
                     //sendbuffer[8]= versioninth;
                     sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                     sendbuffer[6]=abschnittnummer & 0x00FF;
                     
                     //lcd_gotoxy(0,0);
                     sendbuffer[14] = (TIMERINTERVALL & 0xFF00)>>8;
                     sendbuffer[15] = (TIMERINTERVALL & 0x00FF);
                     sendbuffer[8] = ladeposition;
                     sendbuffer[0]=0xD1;
                     Serial.printf("------------------------------------->  first abschnitt, endposition: %d\n",endposition);
                     
                     /*
                      if (code == 0xF0) // cncstatus fuer go_home setzen
                      {
                      
                      sendbuffer[0]=0x45;
                      
                      cncstatus |= (1<<GO_HOME); // Bit fuer go_home setzen
                      usb_rawhid_send((void*)sendbuffer, 50);
                      }
                      else if (code == 0xF1)
                      {
                      sendbuffer[0]=0x44;
                      cncstatus &= ~(1<<GO_HOME); // Bit fuer go_home zuruecksetzen
                      usb_rawhid_send((void*)sendbuffer, 50);
                      }
                      */
                     
                     // Abschnitt 0 melden
                     usb_rawhid_send((void*)sendbuffer, 100);
                     
                     //               startTimer2();
                     //               interrupts();
                     
                  }
                  else // Abschnittnummer > 0
                  {
                     // Ablauf schon gestartert
                     //          Serial.printf("  -----                   Ablauf gestartet, abschnittnummer: %d\n",abschnittnummer);
                      
                  }
                  
                  //             if (buffer[9]& 0x02)// letzter Abschnitt
                  
                  //            Serial.printf("------------------------                buffer 25: %d\n",buffer[25]);
                  
                  // lage im Ablauf: 
                  // 1: erster Abschnitt
                  // 2: letzter Abschnitt
                  // 3: lnur 1 Abschnitt
                  // 0: innerer Abschnitt
                  
                  if (buffer[25]& 0x02)// letzter Abschnitt
                  {
                     //    Serial.printf("------------------------  last abschnitt\n");
                     ringbufferstatus |= (1<<LASTBIT); // letzter Abschnitt
                     if (ringbufferstatus & (1<<FIRSTBIT)) // nur ein Abschnitt
                     {
                        // endposition setzen
                        //                     Serial.printf("------------------------  erster ist letzter Abschnitt\n");
                        endposition=abschnittnummer; // erster ist letzter Abschnitt
                        
                        //                      Serial.printf("------------------------  nur ein abschnitt endposition: %d   * ringbufferstatus: %d\n",endposition, ringbufferstatus);
                     }
                     
                  }
                  
                  // empfangene Daten vom buffer in CNCDaten laden
                  //  if (abschnittnummer == 0)
                  {
                     
                     uint8_t pos=(abschnittnummer);
                     
                     pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
                    Serial.printf("    default: CNC-Daten. abschnittnummer: %d endposition: %d pos: %d\n",abschnittnummer,endposition,pos);
                     //if (abschnittnummer>8)
                     
                     {
                        //lcd_putint1(pos);
                     }
                     
                     // Daten laden in ringbuffer an Position pos
                     uint8_t i=0;
                     /*
                      for(i=0;i<10;i++)
                      {
                      Serial.printf("%d\t",i);
                      }
                      */
                     //Serial.printf("\n");
                     //OSZI_A_LO();
                     //              Serial.printf("default: abschnittnummer: %d pos: %d\n",abschnittnummer,pos);
                     //for(i=0;i<USB_DATENBREITE;i++) // 5 us ohne printf, 10ms mit printf
                     for(i=0;i<64;i++) // 5 us ohne printf, 10ms mit printf
                     { 
                        //                 Serial.printf("%d \t",buffer[i]);
                        
                        CNCDaten[pos][i]=buffer[i];  
                     }
                     //OSZI_A_HI();
                     //               Serial.printf("\n");
                     
                  }
                  
                  
                  // Erster Abschnitt, Beim Start naechsten Abschnitt ebenfalls laden, Anfrage an host
                  
                  
                  if ((abschnittnummer == 0)&&(endposition)) // endposition ist > 0, weitere daten
                  {
                     {
                        //                   Serial.printf("erster Abschnitt, mehr Abschnitte ladeposition: %d endposition: %d\n",ladeposition,endposition);
                        //lcd_putc('*');
                        
                        sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                        sendbuffer[6]=abschnittnummer & 0x00FF;
                        
                        
                        sendbuffer[8]=ladeposition;
                        
                        sendbuffer[0]=0xAF; // next
                        // AF nicht mehr gemeldet             
                        //             uint8_t senderfolg = usb_rawhid_send((void*)sendbuffer, 50);
                        //                     Serial.printf("mehr Abschnitte senderfolg; %d\n",senderfolg);
                        sei();
                        //  sendbuffer[0]=0x00;
                        //  sendbuffer[5]=0x00;
                        //  sendbuffer[6]=0x00;
                        
                        
                     }  
                  }
                  
                  ringbufferstatus &= ~(1<<FIRSTBIT);
                  //          ringbufferstatus |= (1<<STARTBIT);
                  // Ringbuffer ist voll oder  letzter Abschnitt schon erreicht
                  //           Serial.printf("end USB abschnittnummer: %d ringbufferstatus: %d\n",abschnittnummer,ringbufferstatus);
                  //            Serial.printf("\n****************************************\n");
                  //            Serial.printf("Run Abschnitt %d\n",abschnittnummer);
                  //            Serial.printf("****************************************\n");
                  
                  // if ((abschnittnummer ==1 )||((abschnittnummer ==0 )&&(ringbufferstatus & (1<<LASTBIT)))) 
                  if (((abschnittnummer ==0 )&&(ringbufferstatus & (1<<LASTBIT)))) 
                     
                  {
                     {
                        //                Serial.printf("abschnittnummer 1\n");
                        ringbufferstatus &= ~(1<<LASTBIT);
                        ringbufferstatus |= (1<<STARTBIT);
                        
                     }
                  }
                  if (abschnittnummer == 0)
                  {
                     //               startTimer2();
                     //interrupts();
                     
                  }
                  
                  // end case 1
               }break;
                  
#pragma mark DEVICE 2    Joystick            
               case DEVICE_JOY:
               {
                  Serial.printf("default device 2 code: %d\n",code);
#pragma mark abschnittnummer 0
                  
                  if (abschnittnummer==0)  // Start
                  {
                     //noInterrupts();
                     //                   Serial.printf("Device 2 abschnittnummer 0 \t buffer25: %d \t buffer32: %d\n",buffer[25],buffer[32]);
                     sendbuffer[24] =  buffer[32];  
                     //             Serial.printf("count: %d\n",buffer[22]);
                     PWM= buffer[29];
                     
                     ladeposition=0;
                     endposition=0xFFFF;
                     cncstatus = 0;
                     sendstatus = 0;
                     motorstatus = 0;
                     ringbufferstatus=0x00;
                     //anschlagstatus=0;
                     ringbufferstatus |= (1<<FIRSTBIT);
                     ringbufferstatus |= (1<<STARTBIT);
                     AbschnittCounter=0;
                     //sendbuffer[8]= versionintl;
                     //sendbuffer[8]= versioninth;
                     sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                     sendbuffer[6]=abschnittnummer & 0x00FF;
                     
                     //lcd_gotoxy(0,0);
                     sendbuffer[14] = (TIMERINTERVALL & 0xFF00)>>8;
                     sendbuffer[15] = (TIMERINTERVALL & 0x00FF);
                     sendbuffer[8] = ladeposition;
                     sendbuffer[0]=0xD1;
                     //               Serial.printf("------------------------------------->  first abschnitt, endposition: %d\n",endposition);
                     
                     /*
                      if (code == 0xF0) // cncstatus fuer go_home setzen
                      {
                      
                      sendbuffer[0]=0x45;
                      
                      cncstatus |= (1<<GO_HOME); // Bit fuer go_home setzen
                      usb_rawhid_send((void*)sendbuffer, 50);
                      }
                      else if (code == 0xF1)
                      {
                      sendbuffer[0]=0x44;
                      cncstatus &= ~(1<<GO_HOME); // Bit fuer go_home zuruecksetzen
                      usb_rawhid_send((void*)sendbuffer, 50);
                      }
                      */
                     // Abschnitt 0 melden
                     usb_rawhid_send((void*)sendbuffer, 100);
                     
                     //               startTimer2();
                     //               interrupts();
                     
                  }
                  else // Abschnittnummer > 0
                  {
                     // Ablauf schon gestartert
                     //          Serial.printf("  -----                   Ablauf gestartet, abschnittnummer: %d\n",abschnittnummer);
                     //lcd.setCursor(12,0);
                     //lcd.print(String(abschnittnummer));
                     
                  }
                  
                  // abschnittnummer beliebig
                  {
                     
                     uint8_t pos=(abschnittnummer);
                     
                     pos &= 0x03; // 2 bit // Beschraenkung des index auf Buffertiefe 
                     //              Serial.printf("default: load  CNC-Daten. abschnittnummer: %d endposition: %d pos: %d\n",abschnittnummer,endposition,pos);
                     //if (abschnittnummer>8)
                     
                     {
                        //lcd_putint1(pos);
                     }
                     
                     // Daten laden in ringbuffer an Position pos
                     uint8_t i=0;
                     /*
                      for(i=0;i<10;i++)
                      {
                      Serial.printf("%d\t",i);
                      }
                      */
                     //Serial.printf("\n");
                     //OSZI_A_LO();
                     //              Serial.printf("default: abschnittnummer: %d pos: %d\n",abschnittnummer,pos);
                     //for(i=0;i<USB_DATENBREITE;i++) // 5 us ohne printf, 10ms mit printf
                     for(i=0;i<64;i++) // 5 us ohne printf, 10ms mit printf
                     { 
                        //                 Serial.printf("%d \t",buffer[i]);
                        
                        CNCDaten[pos][i]=buffer[i];  
                     }
                     //OSZI_A_HI();
                     //               Serial.printf("\n");
                     
                  }
                  // end case 2
               }break;
            }// switch device
            
            
         }break; // default
            
            
            
      } // switch code
      interrupts();
 //     code=0;
   }// r > 0
   /**   End USB-routinen   ***********************/
   
#pragma mark CNC-routinen   
   /*>
    #define STARTBIT   2       // Buffer ist geladen
    #define RINGBUFFERBIT 3    // Ringbuffer wird verwendet
    #define LASTBIT   4         // Letzter Abschnitt  ist geladen
    #define ENDBIT   5          // Letzter Abschnitt  ist abgearbeitet
    #define STOPBIT   6        // Ablauf stoppen
    #define FIRSTBIT   7
    
    */
   /**   Start CNC-routinen   ***********************/
   if (ringbufferstatus & (1<<STARTBIT)) // Buffer ist in Ringbuffer geladen, Schnittdaten von Abschnitt 0 laden
   {
      //noInterrupts();
      
      Serial.printf("\n                 Abschnitt 0 laden ringbufferstatus: %d\n",ringbufferstatus);
      ringbufferstatus &= ~(1<<STARTBIT);  // Startbit entfernen      
      ladeposition=0;  // laufender Zaehler fuer Ringbuffer, gefiltert mit Ringbuffertiefe
      AbschnittCounter=0;
      
      // Abschnitt 0 laden
      uint8_t l = sizeof(CNCDaten[ladeposition]);
      uint8_t lage = 0;
#pragma mark Ersten Abschnitt laden
      Serial.printf("+++ Ersten Abschnitt laden len: %d ringbufferstatus: %d ladeposition: %d\n",l,ringbufferstatus,ladeposition);
      
      uint8_t i=0;
      
       for(i=0;i<48;i++)
       {
       Serial.printf("%d\t",CNCDaten[ladeposition][i]);
       }
      Serial.printf("\n");
      switch (code)
      {
         case 0xDC:
         {
            if (repeatcounter == 1) // erster Auftritt
            {
               Serial.printf("loop RepeatAbschnittLaden_TS start\n");
               lage=RepeatAbschnittLaden_TS(CNCDaten[ladeposition]); // erster Wert im Ringbuffer
               Serial.printf("loop RepeatAbschnittLaden_TS end\n");

            }

         }break;
         case 0xBC:
         case 0xB4:
         {
            Serial.printf("loop AbschnittLaden_TS\n");
            lage=AbschnittLaden_TS(CNCDaten[ladeposition]); // erster Wert im Ringbuffer
            Serial.printf("loop AbschnittLaden_TS end\n");

         }break;
            
         default:
         {
            Serial.printf("loop AbschnittLaden_TM\n");
            lage=AbschnittLaden_4M(CNCDaten[ladeposition]); // erster Wert im Ringbuffer
            Serial.printf("loop AbschnittLaden_TM end\n");
         }break;

            
      }// switch
       
      //    Serial.printf("+++ Erster Abschnitt lage: %d\n",lage);
 //     lage = 2;
      ladeposition++;
      if (lage==2) // nur ein Abschnitt
      {
         Serial.printf("lage == 2 Abschnitt 0 laden nur 1 Abschnitt anschlagstatus: %d\n",anschlagstatus);
         ringbufferstatus |=(1<<ENDBIT); // unbenutzt
         ringbufferstatus |=(1<<LASTBIT);
      }
      AbschnittCounter+=1;
      Serial.printf("+++ Ersten Abschnitt laden END\n");
      interrupts();
      //      startTimer2();
      //      Serial.printf("motorstatus: %d\n",motorstatus);
   }
   
#pragma mark Anschlag
   
   

   // Anschlagsituation abfragen
   
   // ********************
   // * Anschlag Motor A *
   // ********************
   //AnschlagVonMotor(0); 
   /*
    #define END_A0_PIN         14
    #define END_A1_PIN         15
    
    // X-Achse
    #define END_A0          0       //  Bit fuer Endanschlag A0 
    #define END_A1          1       //  Bit fuer Endanschlag A0 
    
    // Y-Achse
    #define END_B0          2       //           Endanschlag B0 
    #define END_B1          3       //           Endanschlag B0 
    
    // Spindel
    #define END_C0          4      //  Bit fuer Endanschlag C0 
    #define END_C1          5       //  Bit fuer Endanschlag C0 
    
    #define RICHTUNG_A0   0 // entspricht Motor A (motor 0)
    #define RICHTUNG_B0   1
    #define RICHTUNG_C   2
    #define RICHTUNG_D   3
    
    */
   // uint8_t richtungA = richtung & (1<<RICHTUNG_A0);
   
   // vorwaerts
    //Serial.printf("Vorwaerts: A0_PIN_status: %d\n",A0_PIN_status);
   if ((digitalReadFast(END_A0_PIN) == 1))// Eingang ist HI, Schlitten nicht am Anschlag A0
   {
      if (anschlagstatus &(1<< END_A0)) // Schlitten war, aber ist nicht mehr am Anschlag
      {
         Serial.printf("Vorwaerts: Eingang ist HI, Schlitten nicht mehr am Anschlag A0 anschlagstatus: %d richtungA: %d\n",anschlagstatus, (richtung & (1<<RICHTUNG_A0)));
         anschlagstatus &= ~(1<< END_A0); // Bit fuer Anschlag A0 zuruecksetzen
      }
   }
   else if (digitalReadFast(END_A0_PIN) == 0)// Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A0
   {  
      // Serial.printf("Vorwaerts: Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A0 anschlagstatus: %d richtungA: %d\n",anschlagstatus,(richtung & (1<<RICHTUNG_A0)));
      if (richtung & (1<<RICHTUNG_A0))
      {
         Serial.printf("\nVorwaerts: falsche richtung, auf Anschlag A0 zu, warten\n");
         anschlagstatus |= (1<< END_A0);
         //digitalWriteFast(MA_EN,HIGH); // in Interrupt A0_ISR
         sendbuffer[11] = 0;
         sendbuffer[12] = anschlagstatus;
         sendbuffer[13] = richtung;
         sendbuffer[14] = END_A0;
         sendbuffer[14] += 4;
         sendbuffer[0]=0xC5;
         usb_rawhid_send((void*)sendbuffer, 100);
         richtung &= ~(1<<RICHTUNG_A0); // Bit 0
         StepCounterA = 0;
      }
      else if (StepCounterA)
      {
         //Serial.printf("\nVorwaerts: richtige richtung anschlagstatus: *%d*\n",anschlagstatus);
         if (anschlagstatus & (1<< END_A0))
         {
            digitalWriteFast(MA_EN,LOW);
            Serial.printf("\nVorwaerts: richtige richtung, vom Anschlag weg anschlagstatus: *%d*\n",anschlagstatus);
            anschlagstatus &= ~(1<< END_A0); // Bit fuer Anschlag A0 zuruecksetzen
            sendbuffer[12] = anschlagstatus;
            sendbuffer[13] = richtung;
            sendbuffer[14] = END_A0;
            sendbuffer[14] += 8;
            sendbuffer[0]=0xC5;
            usb_rawhid_send((void*)sendbuffer, 100);
         }
      }
   } // if (digitalReadFast(END_A0_PIN))
   
   // rueckwarts
  if (digitalRead(END_A1_PIN)) // Eingang ist HI, Schlitten nicht am Anschlag A1
   {
      if (anschlagstatus &(1<< END_A1)) // Schlitten war, aber ist nicht mehr am Anschlag
      {
         Serial.printf("Rueckwaerts: Eingang ist HI, Schlitten nicht mehr am Anschlag A1 anschlagstatus: %d\n",anschlagstatus);
         anschlagstatus &= ~(1<< END_A1); // Bit fuer Anschlag A0 zuruecksetzen
      }
   }
   else if (digitalRead(END_A1_PIN) == 0)// Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A1
   {   
         sendbuffer[0]=0xC5;
         sendbuffer[11] = 0; // motor
          //Serial.printf("Rueckwaerts: Schlitten bewegte sich auf Anschlag A1 zu und ist am Anschlag A1 anschlagstatus: %d richtungA: %d\n",anschlagstatus,richtung & (1<<RICHTUNG_A0));
         //      AnschlagVonMotor(0,0); // Bewegung anhalten
         if ((richtung & (1<<RICHTUNG_A0)) )
         {
           if ((anschlagstatus & (1<< END_A1)) && (StepCounterA))
            {
               digitalWriteFast(MA_EN,LOW);
               Serial.printf("Rueckwaerts: richtige richtung, vom Anschlag weg anschlagstatus: *%d*\n",anschlagstatus);
               anschlagstatus &= ~(1<< END_A1); // Bit fuer Anschlag A0 zuruecksetzen
               sendbuffer[12] = anschlagstatus;
               sendbuffer[13] = richtung;
               sendbuffer[14] = END_A1;
               sendbuffer[14] += 8;
               sendbuffer[0]=0xC5;
               //usb_rawhid_send((void*)sendbuffer, 50);
            }
         }
         else 
         {
            Serial.printf("Rueckwaerts: falsche richtung, auf Anschlag zu \n");
            anschlagstatus |= (1<< END_A1);
         //   digitalWriteFast(MA_EN,HIGH);
            sendbuffer[12] = anschlagstatus;
            sendbuffer[13] = richtung;
            sendbuffer[14] = END_A1;
            sendbuffer[14] += 4;
            sendbuffer[0]=0xC5;
            richtung |= (1<<RICHTUNG_A0);
            StepCounterA = 0;
            usb_rawhid_send((void*)sendbuffer, 100);
         }
         //
    //  }
   }
   
   // **************************************
   // * Anschlag Motor B *
   // **************************************
   // vorwaerts
   //Serial.printf("Vorwaerts: A0_PIN_status: %d\n",A0_PIN_status);
   if ((digitalReadFast(END_B0_PIN) == 1))// Eingang ist HI, Schlitten nicht am Anschlag A0
   {
      if (anschlagstatus &(1<< END_B0)) // Schlitten war, aber ist nicht mehr am Anschlag
      {
         Serial.printf("Vorwaerts: Eingang ist HI, Schlitten nicht mehr am Anschlag B0 anschlagstatus: %d richtungB: %d\n",anschlagstatus, (richtung & (1<<RICHTUNG_B0)));
         anschlagstatus &= ~(1<< END_B0); // Bit fuer Anschlag A0 zuruecksetzen
      }
   }
   else if (digitalReadFast(END_B0_PIN) == 0)// Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A0
   {  
      // Serial.printf("Vorwaerts: Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A0 anschlagstatus: %d richtungA: %d\n",anschlagstatus,(richtung & (1<<RICHTUNG_A0)));
      if (richtung & (1<<RICHTUNG_B0))
      {
         Serial.printf("\nVorwaerts: falsche richtung, auf Anschlag B0 zu, warten\n");
         anschlagstatus |= (1<< END_B0);
         digitalWriteFast(MB_EN,HIGH);
         sendbuffer[11] = 0;
         sendbuffer[12] = anschlagstatus;
         sendbuffer[13] = richtung;
         sendbuffer[14] = END_B0;
         sendbuffer[14] += 4;
         sendbuffer[0]=0xC5;
         usb_rawhid_send((void*)sendbuffer, 100);
         richtung &= ~(1<<RICHTUNG_B0); // Bit 0
         StepCounterB = 0;
      }
      else if (StepCounterB)
      {
         //Serial.printf("\nVorwaerts: richtige richtung anschlagstatus: *%d*\n",anschlagstatus);
         if (anschlagstatus & (1<< END_B0))
         {
            digitalWriteFast(MB_EN,LOW);
            Serial.printf("\nVorwaerts: richtige richtung, vom Anschlag weg anschlagstatus: *%d*\n",anschlagstatus);
            anschlagstatus &= ~(1<< END_B0); // Bit fuer Anschlag B0 zuruecksetzen
            sendbuffer[12] = anschlagstatus;
            sendbuffer[13] = richtung;
            sendbuffer[14] = END_B0;
            sendbuffer[14] += 8;
            sendbuffer[0]=0xC5;
            usb_rawhid_send((void*)sendbuffer, 100);
         }
      }
   } // if (digitalReadFast(END_A0_PIN))
   
   // rueckwarts
   if (digitalRead(END_B1_PIN)) // Eingang ist HI, Schlitten nicht am Anschlag A1
   {
      if (anschlagstatus &(1<< END_B1)) // Schlitten war, aber ist nicht mehr am Anschlag
      {
         Serial.printf("Rueckwaerts: Eingang ist HI, Schlitten nicht mehr am Anschlag B1 anschlagstatus: %d\n",anschlagstatus);
         anschlagstatus &= ~(1<< END_B1); // Bit fuer Anschlag B1 zuruecksetzen
      }
   }
   else if (digitalRead(END_B1_PIN) == 0)// Schlitten bewegte sich auf Anschlag zu und ist am Anschlag A1
   {   
      sendbuffer[0]=0xC5;
      sendbuffer[11] = 1; // motor
      //Serial.printf("Rueckwaerts: Schlitten bewegte sich auf Anschlag A1 zu und ist am Anschlag A1 anschlagstatus: %d richtungA: %d\n",anschlagstatus,richtung & (1<<RICHTUNG_A0));
      //      AnschlagVonMotor(0,0); // Bewegung anhalten
      if ((richtung & (1<<RICHTUNG_B0)) )
      {
         if ((anschlagstatus & (1<< END_B1)) && (StepCounterB))
         {
            digitalWriteFast(MB_EN,LOW);
            Serial.printf("Rueckwaerts: richtige richtung, vom Anschlag weg anschlagstatus: *%d*\n",anschlagstatus);
            anschlagstatus &= ~(1<< END_B1); // Bit fuer Anschlag B0 zuruecksetzen
            sendbuffer[12] = anschlagstatus;
            sendbuffer[13] = richtung;
            sendbuffer[14] = END_B1;
            sendbuffer[14] += 8;
            sendbuffer[0]=0xC5;
            //usb_rawhid_send((void*)sendbuffer, 50);
         }
      }
      else 
      {
         Serial.printf("Rueckwaerts: falsche richtung, auf Anschlag zu \n");
         anschlagstatus |= (1<< END_B1);
         digitalWriteFast(MB_EN,HIGH);
         sendbuffer[12] = anschlagstatus;
         sendbuffer[13] = richtung;
         sendbuffer[14] = END_B1;
         sendbuffer[14] += 4;
         sendbuffer[0]=0xC5;
         richtung |= (1<<RICHTUNG_B0);
         StepCounterB = 0;
         usb_rawhid_send((void*)sendbuffer, 100);
      }
      //
      //  }
   }

   
   // ********************
   // * Anschlag Motor C *
   // ********************
   //AnschlagVonMotor(2);
   /*
    // Anschlag C0
    if (digitalRead(END_C0_PIN)) // Eingang ist HI, Schlitten nicht am Anschlag C0
    {
    if (anschlagstatus &(1<< END_C0))
    {
    anschlagstatus &= ~(1<< END_C0); // Bit fuer Anschlag C0 zuruecksetzen
    }         
    }
    else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag C0
    {
    AnschlagVonMotor(2);
    }
    */
   // ***************
   // * Anschlag Motor D *
   // ***************
   //AnschlagVonMotor(3);
   /*
    // Anschlag D0
    if (digitalRead(END_D0_PIN)) // Schlitten nicht am Anschlag D0
    {
    if (anschlagstatus &(1<< END_D0))
    {
    anschlagstatus &= ~(1<< END_D0); // Bit fuer Anschlag D0 zuruecksetzen
    }
    }
    else // Schlitten bewegte sich auf Anschlag zu und ist am Anschlag D0
    {
    AnschlagVonMotor(3);
    }
    */
   
#pragma mark Motor A   
   /*
   // Begin Motor A
   // **************************************
   // * Motor A *
   // **************************************
   //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   
   // Es hat noch Steps, CounterA ist abgezaehlt (DelayA bestimmt Impulsabstand fuer Steps)
   if ((StepCounterA > 0)  && (CounterA == 0) &&(!(anschlagstatus & (1<< END_A0))))
   {
      //noInterrupts();
      
      // Impuls starten
      digitalWriteFast(MA_STEP,LOW);
      CounterA = DelayA;                     // CounterA zuruecksetzen fuer neuen Impuls
      korrekturintervallcounterx++;
      if (korrekturintervallcounterx > korrekturintervallx) // 
      {
         
         korrekturcounterx++;
         //        Serial.printf("MA korr korrekturcounterx: %d StepCounterA: %d\n",korrekturcounterx,StepCounterA);
         korrekturintervallcounterx = 0;
         if (vorzeichen & (1<<VORZEICHEN_X)) 
         {
            CounterA -= 1; // korrektur negativ, CounterA verkleinern
          }
         else
         {
            //           StepCounterA++;
            CounterA += 1;// korrektur positiv, CounterA vergroessern
         }
      }
      
      
      //     Serial.printf("Motor A StepCounterA: %d\n",StepCounterA);
      //     Serial.printf("Motor A CounterA: %d StepCounterA: %d \n",CounterA, StepCounterA);
      if (StepCounterA > 0)
      {
         //Serial.printf("*");
         StepCounterA--; // ein Step weniger
      }
      
      // Wenn StepCounterA jetzt abgelaufen und relevant: next Datenpaket abrufen
      if ((StepCounterA == 0 ) && (motorstatus & (1<< COUNT_A)))    // StepCounterA abgelaufen, Motor A ist relevant fuer Stepcount
      {
         sendstatus |= (1<<COUNT_A);
         if (StepCounterB > 0) // Motor B ist noch nicht fertig
         {
            motorstatus |=  (1<< COUNT_B); // Relevanz zuteilen Motor B soll noch abzaehlen
            motorstatus &= ~(1<< COUNT_A); 
         }
         else
         {
            sendstatus |= (1<<COUNT_B); // Motor B auch markieren
         }
         //        Serial.printf("\nMotor A StepCounterA abgelaufen abschnittnummer: %d korrekturcounterx: %d korrekturcountery: %d StepCounterB: %d\n",abschnittnummer,korrekturcounterx, korrekturcountery,StepCounterB);
         Serial.printf("\nMotor A StepCounterA abgelaufen abschnittnummer: %d endposition: %d ringbufferstatus: %d StepCounterB: %d sendstatus: %d\n", abschnittnummer, endposition, ringbufferstatus, StepCounterB, sendstatus);
         //        Serial.printf("Rampbreite: %d rampendstep: %d",rampbreite, rampendstep);
         
         // Begin Ringbuffer-Stuff
         
         if ((abschnittnummer==endposition)) // Ablauf fertig
         {  
            Serial.printf("-----------------------> Motor A endpos korrekturcounterx: %d korrekturcountery: %d\n",korrekturcounterx, korrekturcountery);
            //noInterrupts();
            ringbufferstatus = 0;
            
            //           if (StepCounterB == 0) // erst beide abzaehlen
            {
               sendstatus |= (1<<COUNT_LAST);
            }
            
            sendbuffer[19] = motorstatus;
            sendbuffer[20] = cncstatus;
            cncstatus=0;
            //         motorstatus=0;
            //         sendbuffer[0]=0xAD;
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            sendbuffer[8]=ladeposition & 0x00FF;
            ladeposition=0;
            interrupts();
         }
         else 
         {
            
            uint8_t aktuellelage=0;
            aktuelleladeposition=(ladeposition & 0x00FF);
            aktuelleladeposition &= 0x03;
            
            // aktuellen Abschnitt laden
            aktuellelage = CNCDaten[aktuelleladeposition][25];
            //globalaktuelleladeposition = ladeposition;
            aktuelleladeposition = ladeposition;
            
            Serial.printf("\tMotor A next aktuelleladeposition: %d\n",aktuelleladeposition);
            // > verschoben in 'sendstatus-Bearbeitung
            //     aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
            //sendstatus = 0;
            //          }
            
            if (aktuellelage==2) // war letzter Abschnitt
            {
               Serial.printf("\tMotor A last Abschnitt\n");
               //noInterrupts();
               endposition=abschnittnummer; // letzter Abschnitt
               
                
            }
            else
            {
               //                    Serial.printf("\tMotor A next abschnittnummer: %d\n",abschnittnummer);
               
                
            }
            //           ladeposition++;
            //             AbschnittCounter++;
            //          Serial.printf("\tMotor A Abschnittcounter: %d\n",AbschnittCounter);
         }
      }
      
      interrupts();
   }
   else
   {
      //OSZI_A_LO();
      if (digitalReadFast(MA_STEP) == 0) //100 ns Step ist noch im Gang
      {
         digitalWriteFast(MA_STEP,HIGH);
      }  
      if ((StepCounterA ==0) &&  (digitalReadFast(MA_EN) == 0))                   // Keine Steps mehr fuer Motor A
      {
//         Serial.printf("Motor A > OFF\n");// Motor A OFF
         //             Serial.printf("Motor A OFF korrekturcounterx: %d korrekturcountery: %d\n",korrekturcounterx, korrekturcountery);                  
         digitalWriteFast(MA_EN,HIGH);
         
         if (StepCounterB ==0)
         {
            digitalWriteFast(MB_EN,HIGH);
         }
         
      }
      
      //OSZI_A_HI();
      interrupts();
   }
   */
#pragma mark Motor B
   
   /*
   // **************************************
   // * Motor B *
   // **************************************
   
   if ((StepCounterB > 0) && (CounterB == 0)&&(!(anschlagstatus & (1<< END_B0))))
   {
      //noInterrupts();
      
      //STEPPERPORT_1 &= ~(1<<MB_STEP);               // Impuls an Motor B LO ON
      digitalWriteFast(MB_STEP,LOW);
      CounterB= DelayB;
      korrekturintervallcountery++;
      if (korrekturintervallcountery > korrekturintervally) // 
      {
         korrekturintervallcountery = 0;
 //        Serial.printf("MB korr korrekturintervally: %d  korrekturcountery: %d\n",korrekturintervally,korrekturcountery);
         korrekturcountery++;
         //        Serial.printf("MB korr korrekturcounterxy: %d StepCounterB: %d\n",korrekturcountery,StepCounterB);         korrekturintervallcountery = 0;
         if (vorzeichen & (1<<VORZEICHEN_Y)) 
         {
            if (StepCounterB > 0)
            {
               //          StepCounterB--;
            }
            
            CounterB -= 1; // korrektur negativ, CounterA verkleinern
         }
         else
         {
            //fB korr korrekturcounterxy:          StepCounterB++;
            CounterB += 1;// korrektur positiv, CounterA vergroessern
         }
      }
      
      if (StepCounterB > 0)
      {
         StepCounterB--;
      }
      
      if ((StepCounterB == 0)  && (motorstatus & (1<< COUNT_B))) // StepCounterB abgelaufen, Motor B ist relevant fuer Stepcount 
      {
         sendstatus |= (1<<COUNT_B);
         if (StepCounterA > 0)
         {
            motorstatus |=  (1<< COUNT_A);
            motorstatus &= ~(1<< COUNT_B);
         }
         else
         {
            sendstatus |= (1<<COUNT_A); // Motor A abgelaufen, auch markieren
         }
         // Serial.printf("\nMotor B StepCounterB abgelaufen abschnittnummer: %d",abschnittnummer);
         //       Serial.printf("\nMotor B StepCounterB abgelaufen abschnittnummer: %d korrekturcounterx: %d korrekturcountery: %d  StepCounterA: %d\n",abschnittnummer,korrekturcounterx, korrekturcountery,StepCounterA);
         
         //         Serial.printf("\nMotor B StepCounterB abgelaufen abschnittnummer: %d endposition: %d ringbufferstatus: %d StepCounterA: %d sendstatus: %d\n", abschnittnummer, endposition, ringbufferstatus,StepCounterA,sendstatus);
         
         // Begin Ringbuffer-Stuff
         
         if ((abschnittnummer==endposition) )
         {  
            //cli();
            //            Serial.printf("----------------------> Motor B endpos korrekturcounterx: %d korrekturcountery: %d\n",korrekturcounterx, korrekturcountery);
            //noInterrupts();
            ringbufferstatus = 0;
            
            //           if (StepCounterA == 0)
            {
               sendstatus |= (1<<COUNT_LAST); // beenden abzeigen
            }
            
            sendbuffer[19] = motorstatus;
            sendbuffer[20] = cncstatus;
            cncstatus=0;
            //     motorstatus=0;
            
            //           sendbuffer[0]=0xAD;
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
    //        sendbuffer[7]=(ladeposition & 0xFF00) >> 8;;
            sendbuffer[8]=ladeposition & 0x00FF;
            //           usb_rawhid_send((void*)sendbuffer, 50);
            ladeposition=0;
            
         }
         else 
         { 
            uint8_t aktuellelage=0;
            aktuelleladeposition=(ladeposition & 0x00FF);
            aktuelleladeposition &= 0x03;
            
            // aktuellen Abschnitt laden
            aktuellelage = CNCDaten[aktuelleladeposition][25];
            //globalaktuelleladeposition = ladeposition;
            aktuelleladeposition = ladeposition;
            
            //          Serial.printf("\tMotor B  globalaktuelleladeposition: %d\n",globalaktuelleladeposition);
            // > verschoben in 'sendstatus-Bearbeitung
            
            //aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
            //sendstatus = 0;
            
            
            if (aktuellelage==2) // war letzter Abschnitt
            {
               //noInterrupts();
               endposition=abschnittnummer; // letzter Abschnitt
               Serial.printf("\tMotor B last Abschnitt\n");
               // sendstatus |= (1<<COUNT_LAST);
               
               
               // Neu: letzten Abschnitt melden
                 
            }  
            else 
            {
               
               //                 Serial.printf("\tMotor B next abschnittnummer: %d\n",abschnittnummer);
               
            }
            
         }
         
      }
      
      
      interrupts();
   }
   else// if (CounterB)
   {
      
      //STEPPERPORT_1 |= (1<<MB_STEP);
      if (digitalReadFast(MB_STEP) == 0) //100 ns Step ist noch im Gang
      {
         digitalWriteFast(MB_STEP,HIGH);
         if (StepCounterB ==0)                     // Keine Steps mehr fuer Motor B
         {
            Serial.printf("Motor B OFF korrekturcounterx: %d korrekturcountery: %d StepCounterA: %d\n",korrekturcounterx, korrekturcountery,StepCounterA);
           
            //STEPPERPORT_1 |= (1<<MB_EN);               // Motor B OFF
            digitalWriteFast(MB_EN,HIGH);
            
            if (StepCounterA ==0)
            {
               digitalWriteFast(MA_EN,HIGH);
            }
         }
      }
      interrupts();
   }
   
   
   // End Motor B
*/   
   // Begin Motor C
#pragma mark Motor C
   
  /* 
   // **************************************
   // * Motor C *
   // **************************************
   
   // Es hat noch Steps, CounterC ist abgezaehlt (DelayA bestimmt Impulsabstand fuer Steps)
   if (StepCounterC &&(CounterC == 0) &&(!(anschlagstatus & (1<< END_C0))))//||(cncstatus & (1<< END_D0)))))//   
   {
      //noInterrupts();
      // Impuls starten
      //STEPPERPORT_2 &= ~(1<<MC_STEP);   // Impuls an Motor C LO -> ON
      digitalWriteFast(MC_STEP,LOW);
      CounterC=DelayC;                     // CounterC zuruecksetzen fuer neuen Impuls
      
      if (StepCounterC > 0)
      {
      StepCounterC--;
      }
      
      // Wenn StepCounterC abgelaufen und relevant: next Datenpaket abrufen
      if (StepCounterC ==0 && (motorstatus & (1<< COUNT_C)))    // Motor A ist relevant fuer Stepcount 
      {
         sendstatus |= (1<<COUNT_C);
         Serial.printf("\n***             Motor C StepCounterC abgelaufen abschnittnummer: %d endposition: %d ringbufferstatus: %d  sendstatus: %d\n", abschnittnummer, endposition, ringbufferstatus,  sendstatus);
         
          
         
         if (abschnittnummer==endposition)
         {  
            //cli();
            Serial.printf("\nabschnittnummer = endposition  Motor C endpos\n");
            sendstatus |= (1<<COUNT_LAST);
            sendbuffer[0]=0xBB;
            sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
            sendbuffer[6]=abschnittnummer & 0x00FF;
            
            sendbuffer[8]=ladeposition;
            
            sendbuffer[19] = motorstatus;
            sendbuffer[20] = cncstatus;
            
            // drillstatus incr.
            drillstatus++;
            Serial.printf("\nMotor C abschnittnummer==endposition drillstatus incr drillstatus nach: %d\n",drillstatus);
            
            sendbuffer[22] = drillstatus;
            
            
            cncstatus=0;
            motorstatus=0;
            
            usb_rawhid_send((void*)sendbuffer, 100);
            
            ringbufferstatus = 0;
            
            ladeposition=0;
            interrupts();
         }
         else 
         { 
            uint8_t aktuellelage=0; // Lage innerhalb der Abschnittserie: Start: 1, Innerhalb: 0, Ende: 2
            {
               aktuelleladeposition=(ladeposition & 0x00FF);
               aktuelleladeposition &= 0x03;
               
               // aktuellen Abschnitt laden
               
               if (ladeposition>8)
               {
                  //lcd_putint1(ladeposition);
               }
               aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
               Serial.printf("\nMotor C aktuellelage: %d\n",aktuellelage);
               if (aktuellelage==2) // war letzter Abschnitt
               {
                  endposition=abschnittnummer; // letzter Abschnitt
                  // Neu: letzten Abschnitt melden
                  sendbuffer[0]=0xD0;
                  sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                  sendbuffer[6]=abschnittnummer & 0x00FF;
                  
                  sendbuffer[8]=ladeposition & 0x00FF;
                  //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  usb_rawhid_send((void*)sendbuffer, 100);
                  
                  
               }  
               else
               {
                  // neuen Abschnitt abrufen
                  sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                  sendbuffer[6]=abschnittnummer & 0x00FF;
                  
                  // drillstatus incr.
                  drillstatus++;
                  Serial.printf("\nMotor C drillstatus incr drillstatus nach: %d\n",drillstatus);
                  sendbuffer[22] = drillstatus;
                  //drillstatus = 0xFF;
                  sendbuffer[8]=ladeposition & 0x00FF;
                  //      sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  sendbuffer[0]=0xBC; // Schnittdatenarray leer
                  usb_rawhid_send((void*)sendbuffer, 100);  
                  
               }
               
               ladeposition++;
               
            }
            
            
            AbschnittCounter++;
            
         }
         
      }
      
      interrupts();
   }
   else
   {
      //STEPPERPORT_2 |= (1<<MC_STEP);               // Impuls an Motor C HI -> OFF
      digitalWriteFast(MC_STEP,HIGH);
      if (StepCounterC ==0)                     // Keine Steps mehr fuer Motor C
      {
         //Serial.printf("Motor C  StepCounterC 0");
         //STEPPERPORT_2 |= (1<<MC_EN);                     // Motor C OFF
         digitalWriteFast(MC_EN,HIGH);
      }
   }
   */
#pragma mark Motor D
   /*
   // **************************************
   // * Motor D *
   // **************************************
   
   if (StepCounterD && (CounterD == 0)&&(!(anschlagstatus & (1<< END_D0))))
   {
      noInterrupts();
      
      //STEPPERPORT_2 &= ~(1<<MD_STEP);               // Impuls an Motor D LO: ON
      digitalWriteFast(MD_STEP,LOW);
      CounterD= DelayD;
      StepCounterD--;
      
      if (StepCounterD ==0 && (motorstatus & (1<< COUNT_D))) // Motor D ist relevant fuer Stepcount 
      {
         //            STEPPERPORT_2 |= (1<<MD_EN);               // Motor D OFF
         
         //StepCounterC=0;
         // Begin Ringbuffer-Stuff
         if (abschnittnummer==endposition)
         {  
            //cli();
            Serial.printf("\nMotor D endpos\n");
            ringbufferstatus = 0;
            cncstatus=0;
            motorstatus=0;
            sendbuffer[0]=0xAD;
            //sendbuffer[1]=abschnittnummer;
            usb_rawhid_send((void*)sendbuffer, 100);
            ladeposition=0;
            
         }
         else 
         { 
            uint8_t aktuellelage=0;
            {
               aktuelleladeposition=(ladeposition & 0x00FF);
               aktuelleladeposition &= 0x03;
               
               // aktuellen Abschnitt laden
               
               aktuellelage = AbschnittLaden_4M(CNCDaten[aktuelleladeposition]);
               if (aktuellelage==2) // war letzter Abschnitt
               {
                  endposition=abschnittnummer; // letzter Abschnitt
                  // Neu: letzten Abschnitt melden
                  sendbuffer[0]=0xD0;
                  sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                  sendbuffer[6]=abschnittnummer & 0x00FF;
                  
                  sendbuffer[8]=ladeposition & 0x00FF;
                  //    sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  usb_rawhid_send((void*)sendbuffer, 100);
                  
                  
               }  
               else
               {
                  // neuen Abschnitt abruffen
                  sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
                  sendbuffer[6]=abschnittnummer & 0x00FF;
                  
                  sendbuffer[8]=ladeposition & 0x00FF;
                  //   sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
                  sendbuffer[0]=0xA3;
                  usb_rawhid_send((void*)sendbuffer, 100);
                  
               }
               
               ladeposition++;
               
            }
            AbschnittCounter++;
            
         }
      }
      
      
      interrupts();
   }
   else// if (CounterB)
   {
      //STEPPERPORT_2 |= (1<<MD_STEP);
      digitalWriteFast(MD_STEP,HIGH);
      if (StepCounterD ==0)                     // Keine Steps mehr fuer Motor D
      {
         //STEPPERPORT_2 |= (1<<MD_EN);               // Motor D OFF
         digitalWriteFast(MD_EN,HIGH);
      }
      
   }
   */
#pragma mark sendstatus
   //if (sendstatus >= 3)
   if (sendstatus > 0)
   {
      
      //Serial.printf("\n++++++++++++++++++++++++++++++\nsendstatus: %d abschnittnummer: %d endposition: %d globalaktuelleladeposition: %d: StepCounterA: %d StepCounterB: %d\n", sendstatus,abschnittnummer,endposition,globalaktuelleladeposition,StepCounterA, StepCounterB);
      //     Serial.printf("\n++++++++++++++++++++++++++++++\nsendstatus: %d abschnittnummer: %d endposition: %d aktuelleladeposition: %d: StepCounterA: %d StepCounterB: %d\n", sendstatus,abschnittnummer,endposition,aktuelleladeposition,StepCounterA, StepCounterB);
      
      //      if ((sendstatus == 3) ) 
      if ((sendstatus  <= 4) ) 
      {
         
         
         Serial.printf("\nsendstatus.task abschnittnummer: %d endposition: %d \n",abschnittnummer, endposition);
         Serial.printf("\nsendstatus.task aktuelleladeposition: %d ladeposition: %d \n",aktuelleladeposition, ladeposition);
         
         if (abschnittnummer == endposition)
         {
            //            Serial.printf("\n****************************************\n");
            //            Serial.printf("\n sendstatus <=4  wert: %d  abschnittnummer = endposition\n",sendstatus);
            //            Serial.printf("\n****************************************\n");
         }
         
         motorstatus=0;
         sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;
         sendbuffer[6]=abschnittnummer & 0x00FF;
         
         //sendbuffer[8]=globalaktuelleladeposition & 0x00FF;
   //      sendbuffer[7]=(aktuelleladeposition & 0xFF00) >> 8;
         sendbuffer[8]=aktuelleladeposition & 0x00FF;
         
         
         {
            // ***************************************
            sendbuffer[0]=0xA1;
            // ***************************************
            
            uint8_t senderfolg = usb_rawhid_send((void*)sendbuffer, 100);
            Serial.printf("\nsendstatus senderfolg: %d",senderfolg);
            /*         
             if (abschnittnummer == endposition)
             {
             //         Serial.printf("\tsendstatus LAST setzen\n\n");
             //         sendstatus |= (1<<COUNT_LAST);
             }
             else
             */
            {
               //             Serial.printf("\tsendstatus next Abschnitt\n\n");
               //uint8_t aktuellelage = AbschnittLaden_4M(CNCDaten[(globalaktuelleladeposition & 0x03)]);
               uint8_t aktuellelage = AbschnittLaden_4M(CNCDaten[(aktuelleladeposition & 0x03)]);
               
               //           Serial.printf("\n****************************************\n");
               //           Serial.printf("Load Abschnitt %d\n",globalaktuelleladeposition);
               //           Serial.printf("****************************************\n");
               
               //            Serial.printf("\n\tsendstatus next Abschnitt geladen aktuellelage: \n",aktuellelage);
               ladeposition++;
               
               AbschnittCounter=0;
            }
            /*
             if (abschnittnummer == endposition)
             {
             sendbuffer[0]=0xD0;
             sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
             sendbuffer[6]=abschnittnummer & 0x00FF;
             
             sendbuffer[8]=globalaktuelleladeposition & 0x00FF;
             //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
             usb_rawhid_send((void*)sendbuffer, 50);
             
             }
             */
         }
         
         //Serial.printf("\nsendstatus: %d abschnittnummer: %d globalaktuelleladeposition: %d\n", sendstatus,abschnittnummer,globalaktuelleladeposition);  
         //         Serial.printf("\n end <=3 sendstatus: %d abschnittnummer: %d aktuelleladeposition: %d\n", sendstatus,abschnittnummer,aktuelleladeposition);  
         sendstatus = 0;
      }
      
      
      
      if ((sendstatus & (1<<COUNT_LAST)) && ((StepCounterA == 0) && (StepCounterB == 0) && (StepCounterB == 0))) // 131
      {
         //        Serial.printf("\t+++   sendstatus last   +++\n");
         Serial.printf("\t COUNT LAST StepCounterA: %d StepCounterB: %d tabledatastatus: %02X\n", StepCounterA, StepCounterB,tabledatastatus);
         if (abschnittnummer == endposition)
         {
            Serial.printf("\n**************************************** \n");
            Serial.printf(" sendstatus 131 COUNT_LAST  abschnittnummer = endposition");
            Serial.printf("\n****************************************\n");
         }
         
         //       sendbuffer[0]=0xD0;
         //      motorstatus=0;
         
         // ***************************************
         sendbuffer[0]=0xAD;
         // ***************************************
         
         sendbuffer[5]=(abschnittnummer & 0xFF00) >> 8;;
         sendbuffer[6]=abschnittnummer & 0x00FF;
         
         //sendbuffer[8]=globalaktuelleladeposition & 0x00FF;
         sendbuffer[8] = aktuelleladeposition & 0x00FF;
         
         sendbuffer[22] = drillstatus;
         sendbuffer[23] = tabledatastatus;
         
         sendbuffer[33] = tabledatastatus;
      
         //sendbuffer[7]=(ladeposition & 0xFF00) >> 8;
         uint8_t senderfolg = usb_rawhid_send((void*)sendbuffer, 100);
         Serial.printf("COUNT_LAST sendstatus senderfolg: %d\n",senderfolg);
         sendstatus = 0;
      }
      // ladeposition++;
      AbschnittCounter++;
      sendstatus = 0;
      
   }
   
   if (sendstatus > 0)
   {
      //  Serial.printf("      sendstatus: %d, anschlagstatus: %02X",sendstatus, anschlagstatus);
      //Serial.printf("\nsendstatus: %d abschnittnummer: %d globalaktuelleladeposition: %d\n", sendstatus,abschnittnummer,globalaktuelleladeposition);
      //Serial.printf("\nsendstatus: %d abschnittnummer: %d aktuelleladeposition: %d\n", sendstatus,abschnittnummer,aktuelleladeposition);
      //     sendstatus = 0;
   }
   
   interrupts();
   // End Motor D
   
   
   
   
   
   
   
   /**   End CNC-routinen   ***********************/
}
