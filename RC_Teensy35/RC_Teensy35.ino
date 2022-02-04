#include <RingBuf.h>
//#include <SdFatConfig.h>

#include <BufferedPrint.h>
//#include <FreeStack.h>
//#include <MinimumSerial.h>
//#include <SD.h>

//#include <Wire.h>
//#include <WireIMXRT.h>
//#include <WireKinetis.h>

///



// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu
#include "Arduino.h"
//#include "TeensyStep.h"
#include <ADC.h>
#include "gpio_MCP23S17.h"
#include <SPI.h>
#include "lcd.h"
#include "settings.h"
//#include <Wire.h>
//#include <i2c_t3.h>
//#include <LiquidCrystal_I2C.h> // auch in Makefile angeben!!!
//#include <TeensyThreads.h>

/*
#include <SdFat.h>
//#include <sdios.h>

//#include "TeensyTimerTool.h"
//using namespace TeensyTimerTool;
const int8_t DISABLE_CS_PIN = -1;
#define SD_FAT_TYPE 3

// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(50)

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif  ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
SdFs sd;
cid_t m_cid;
csd_t m_csd;
uint32_t m_eraseSize;
uint32_t m_ocr;
static ArduinoOutStream cout(Serial);
//------------------------------------------------------------------------------

bool cidDmp() {
  cout << F("\nManufacturer ID: ");
  cout << uppercase << showbase << hex << int(m_cid.mid) << dec << endl;
  cout << F("OEM ID: ") << m_cid.oid[0] << m_cid.oid[1] << endl;
  cout << F("Product: ");
  for (uint8_t i = 0; i < 5; i++) {
    cout << m_cid.pnm[i];
  }
  cout << F("\nVersion: ");
  cout << int(m_cid.prv_n) << '.' << int(m_cid.prv_m) << endl;
  cout << F("Serial number: ") << hex << m_cid.psn << dec << endl;
  cout << F("Manufacturing date: ");
  cout << int(m_cid.mdt_month) << '/';
  cout << (2000 + 16*m_cid.mdt_year_high + m_cid.mdt_year_low) << endl;
  cout << endl;
  return true;
}
//------------------------------------------------------------------------------
void clearSerialInput() {
  uint32_t m = micros();
  do {
    if (Serial.read() >= 0) {
      m = micros();
    }
  } while (micros() - m < 10000);
}
//------------------------------------------------------------------------------
bool csdDmp() {
  bool eraseSingleBlock;
  if (m_csd.v1.csd_ver == 0) {
    eraseSingleBlock = m_csd.v1.erase_blk_en;
    m_eraseSize = (m_csd.v1.sector_size_high << 1) | m_csd.v1.sector_size_low;
  } else if (m_csd.v2.csd_ver == 1) {
    eraseSingleBlock = m_csd.v2.erase_blk_en;
    m_eraseSize = (m_csd.v2.sector_size_high << 1) | m_csd.v2.sector_size_low;
  } else {
    cout << F("m_csd version error\n");
    return false;
  }
  m_eraseSize++;
  cout << F("cardSize: ") << 0.000512 * sdCardCapacity(&m_csd);
  cout << F(" MB (MB = 1,000,000 bytes)\n");

  cout << F("flashEraseSize: ") << int(m_eraseSize) << F(" blocks\n");
  cout << F("eraseSingleBlock: ");
  if (eraseSingleBlock) {
    cout << F("true\n");
  } else {
    cout << F("false\n");
  }
  return true;
}
//------------------------------------------------------------------------------
void errorPrint() {
  if (sd.sdErrorCode()) {
    cout << F("SD errorCode: ") << hex << showbase;
    printSdErrorSymbol(&Serial, sd.sdErrorCode());
    cout << F(" = ") << int(sd.sdErrorCode()) << endl;
    cout << F("SD errorData = ") << int(sd.sdErrorData()) << endl;
  }
}
//------------------------------------------------------------------------------
bool mbrDmp() {
  MbrSector_t mbr;
  bool valid = true;
  if (!sd.card()->readSector(0, (uint8_t*)&mbr)) {
    cout << F("\nread MBR failed.\n");
    errorPrint();
    return false;
  }
  cout << F("\nSD Partition Table\n");
  cout << F("part,boot,bgnCHS[3],type,endCHS[3],start,length\n");
  for (uint8_t ip = 1; ip < 5; ip++) {
    MbrPart_t *pt = &mbr.part[ip - 1];
    if ((pt->boot != 0 && pt->boot != 0X80) ||
        getLe32(pt->relativeSectors) > sdCardCapacity(&m_csd)) {
      valid = false;
    }
    cout << int(ip) << ',' << uppercase << showbase << hex;
    cout << int(pt->boot) << ',';
    for (int i = 0; i < 3; i++ ) {
      cout << int(pt->beginCHS[i]) << ',';
    }
    cout << int(pt->type) << ',';
    for (int i = 0; i < 3; i++ ) {
      cout << int(pt->endCHS[i]) << ',';
    }
    cout << dec << getLe32(pt->relativeSectors) << ',';
    cout << getLe32(pt->totalSectors) << endl;
  }
  if (!valid) {
    cout << F("\nMBR not valid, assuming Super Floppy format.\n");
  }
  return true;
}
//------------------------------------------------------------------------------
void dmpVol() {
  cout << F("\nScanning FAT, please wait.\n");
  uint32_t freeClusterCount = sd.freeClusterCount();
  if (sd.fatType() <= 32) {
    cout << F("\nVolume is FAT") << int(sd.fatType()) << endl;
  } else {
    cout << F("\nVolume is exFAT\n");
  }
  cout << F("sectorsPerCluster: ") << sd.sectorsPerCluster() << endl;
  cout << F("clusterCount:      ") << sd.clusterCount() << endl;
  cout << F("freeClusterCount:  ") << freeClusterCount << endl;
  cout << F("fatStartSector:    ") << sd.fatStartSector() << endl;
  cout << F("dataStartSector:   ") << sd.dataStartSector() << endl;
  if (sd.dataStartSector() % m_eraseSize) {
    cout << F("Data area is not aligned on flash erase boundary!\n");
    cout << F("Download and use formatter from www.sdcard.org!\n");
  }
}
//------------------------------------------------------------------------------
void printCardType() {

  cout << F("\nCard type: ");

  switch (sd.card()->type()) {
    case SD_CARD_TYPE_SD1:
      cout << F("SD1\n");
      break;

    case SD_CARD_TYPE_SD2:
      cout << F("SD2\n");
      break;

    case SD_CARD_TYPE_SDHC:
      if (sdCardCapacity(&m_csd) < 70000000) {
        cout << F("SDHC\n");
      } else {
        cout << F("SDXC\n");
      }
      break;

    default:
      cout << F("Unknown\n");
  }
}
//------------------------------------------------------------------------------
void printConfig(SdSpiConfig config) {
  if (DISABLE_CS_PIN < 0) {
    cout << F(
           "\nAssuming the SD is the only SPI device.\n"
           "Edit DISABLE_CS_PIN to disable an SPI device.\n");
  } else {
    cout << F("\nDisabling SPI device on pin ");
    cout << int(DISABLE_CS_PIN) << endl;
    pinMode(DISABLE_CS_PIN, OUTPUT);
    digitalWrite(DISABLE_CS_PIN, HIGH);
  }
  cout << F("\nAssuming the SD chip select pin is: ") << int(config.csPin);
  cout << F("\nEdit SD_CS_PIN to change the SD chip select pin.\n");
}
//------------------------------------------------------------------------------
void printConfig(SdioConfig config) {
  (void)config;
  cout << F("Assuming an SDIO interface.\n");
}
*/

// Include application, user and local libraries
// !!! Help http://bit.ly/2CL22Qp


// Define structures and classes


// Define variables and constants
uint8_t loopLED;
#define USB_DATENBREITE 64

#define TEST 1

#define NUM_SERVOS 8

int8_t r;

// USB
volatile uint8_t inbuffer[USB_DATENBREITE]={};
volatile uint8_t outbuffer[USB_DATENBREITE]={};
volatile uint16_t          usb_recv_counter=0;
volatile uint16_t          cnc_recv_counter=0;
// end USB

// RC 
volatile uint8_t           timerstatus=0;
volatile uint8_t           code=0;
volatile uint8_t           servostatus=0;

#define PAUSE 1
elapsedMillis sinceblink;
elapsedMillis sincelcd;
elapsedMicros sinceusb;
uint16_t cncdelaycounter = 0;

elapsedMillis sinceload; // Zeitdauer der Anzeige des Potentialwertes

elapsedMicros sincelaststep;

elapsedMillis sincelastthread;

elapsedMicros sincelastimpuls;


// Prototypes

static volatile uint8_t buffer[USB_DATENBREITE]={};   // Daten von usb
static volatile uint8_t sendbuffer[USB_DATENBREITE]={};// Daten nach usb

static volatile uint8_t drillbuffer[USB_DATENBREITE]={};// Daten fuer Drill, beim Start geladen


// Ringbuffer
uint8_t                    CNC_Data[USB_DATENBREITE];



volatile uint16_t          tablezeile = 0;  // zeile in tabledata

// Create an IntervalTimer object 
#  pragma mark intervaltimer

IntervalTimer servoTimer;
IntervalTimer impulsTimer;

IntervalTimer microTimer; 
uint16_t microcounter = 0;

#define IMPULSPIN  2

IntervalTimer              delayTimer;

volatile uint8_t           servoindex = 0;
// Utilities

volatile uint16_t impulstimearray[NUM_SERVOS] = {};

#  pragma mark TeensyStep Variablen
// TeensyStep


uint16_t taste = 0xFF;
uint8_t tastencounter = 0;
uint8_t TastaturCount=0;
uint8_t Taste=0;
uint8_t analogtastaturstatus = 0;
#define TASTE_OFF  0
#define TASTE_ON  1

uint16_t TastenStatus=0;
uint16_t Tastenprellen=0x1F;


ADC *adc = new ADC(); // adc object

#  pragma mark Ganssle setup
// Ganssle von Netzteil_20
typedef struct
{
   uint8_t pin;
   uint16_t tasten_history;
   uint8_t pressed;
   long lastDebounceTime;
}tastenstatus;

//long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 20;    // the debounce time; increase if the output flickers
volatile uint8_t tipptastenstatus = 0;
tastenstatus tastenstatusarray[8] = {}; 

uint8_t tastenbitstatus = 0; // bits fuer tasten

volatile uint8_t tastencode = 0; // status der Tasten vom SPI-SR

// http://www.ganssle.com/debouncing-pt2.htm
#define MAX_CHECKS 8
volatile uint8_t last_debounced_state = 0;
volatile uint8_t debounced_state = 0;
volatile uint8_t state[MAX_CHECKS] = {0};

volatile uint8_t debounceindex = 0;
void debounce_switch(uint8_t port)
{
   uint8_t i,j;
   state[debounceindex] = port;
   ++debounceindex;
   j = 0xFF;
   for (i=0;i<MAX_CHECKS;i++)
   {
      j=j & state[i];
   }
   debounced_state = j;
   
   if (debounceindex >= MAX_CHECKS)
   {
      debounceindex = 0;
   }
   
}




// Elliot https://hackaday.com/2015/12/10/embed-with-elliot-debounce-your-noisy-buttons-part-ii/
uint8_t readTaste(uint8_t taste)
{
   
   return (digitalReadFast(taste) == 0);
}

void update_button(uint8_t taste, uint16_t *button_history)
{
   *button_history = *button_history << 1;
   *button_history |= readTaste(taste); 
}

uint8_t is_button_pressed(uint8_t button_history)
{
   return (button_history == 0b01111111);
}

uint8_t is_button_released(uint8_t button_history)
{
   return (button_history == 0b10000000);
}

uint8_t is_button_down(uint8_t *button_history)
{
        return (*button_history == 0b11111111);
}
uint8_t is_button_up(uint8_t *button_history)
{
        return (*button_history == 0b00000000);
}
uint8_t test_for_press_only(uint8_t pin)
{   
   static uint16_t button_history = 0;
   uint8_t pressed = 0;    
   
   button_history = button_history << 1;
   button_history |= readTaste(pin);
   if ((button_history & 0b11000111) == 0b00000111)
   { 
      pressed = 1;
      button_history = 0b11111111;
   }
   return pressed;
}

// end Elliot


uint8_t checktasten(void)
{
   uint8_t count = 0; // Anzahl aktivierter Tasten
   uint8_t i=0;
   uint8_t tastencode = 0;
   while (i<8)
   {
      uint8_t pressed = 0;
      if (tastenstatusarray[i].pin < 0xFF)
      {
         count++;
         tastenstatusarray[i].tasten_history = (tastenstatusarray[i].tasten_history << 1);
         tastenstatusarray[i].tasten_history |= readTaste(tastenstatusarray[i].pin); // pin-nummer von element i
         if ((tastenstatusarray[i].tasten_history & 0b11000111) == 0b00000111)
         {
            pressed = 1;
            tipptastenstatus |= (1<<i);
            tastenbitstatus |= (1<<i);
            tastenstatusarray[i].tasten_history = 0b11111111;
            tastenstatusarray[i].pressed = pressed;
         }
         
      }// i < 0xFF
      
      i++;
   }
   // tastenstatusarray
   //return tastencode;
   return tipptastenstatus ;
}



// end rev

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


void OSZI_C_LO(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_C,LOW);
}

void OSZI_C_HI(void)
{
   if (TEST)
      digitalWriteFast(OSZI_PULS_C,HIGH);
}

uint8_t Menu_Ebene=0;

uint8_t Tastenwahl(uint16_t Tastaturwert)
{
   if (Tastaturwert < TASTE1)
      return 1;
   if (Tastaturwert < TASTE2)
      return 2;
   if (Tastaturwert < TASTE3)
      return 3;
   if (Tastaturwert < TASTE4)
      return 4;
   if (Tastaturwert < TASTE5)
      return 5;
   if (Tastaturwert < TASTE6)
      return 6;
   if (Tastaturwert < TASTE7)
      return 7;
   if (Tastaturwert < TASTE8)
      return 8;
   if (Tastaturwert < TASTE9)
      return 9;
   if (Tastaturwert < TASTEL)
      return 10;
   if (Tastaturwert < TASTE0)
      return 0;
   if (Tastaturwert < TASTER)
      return 12;
   
   return 0;
}
 // tastenwahl

void servoimpulsfunction(void) // 
{ 

}

void servopaketfunktion(void) // start Abschnitt
{ 
   servostatus &= ~(1<<PAUSE);
   OSZI_B_LO();
}

void microtimerfunktion(void)
{
   OSZI_A_TOGG();
   if (servostatus & (1<<PAUSE))
   {
      OSZI_C_LO();
   }
   else 
   {
      microcounter++;
      
      
      if (microcounter > impulstimearray[servoindex]) // Impuls ende
      {
         digitalWriteFast(IMPULSPIN,LOW);
      }
      
      if (microcounter > 2200)
      {
         digitalWriteFast(IMPULSPIN,HIGH);
         microcounter = 0;
         
         if(servoindex < NUM_SERVOS)
         {
            if (servoindex == 0)
            {
               OSZI_B_LO();
            }
            servoindex++;
         }
         else 
         {
            
            servoindex = 0;
            OSZI_B_HI();
            servostatus |= (1<<PAUSE); // pause beginnt
            
            digitalWriteFast(IMPULSPIN,LOW);
            OSZI_C_HI();
         }
         
      }
   }
   // digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   
}

void servotimerfunction(void) // 1us ohne ramp
{ 
//   digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   
   
  
   
   if (timerstatus & (1<<TIMER_ON))
   {
      //OSZI_A_LO();
       
      //OSZI_A_HI();
      
      
      
   } // if timerstatus
   
   //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
   // TCNT2 = 10;                     // ergibt 2 kHz fuer Timertakt
   
}






gpio_MCP23S17     mcp0(10,0x20);//instance 0 (address A0,A1,A2 tied to 0)
//LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x20
//delay(1000); 
// Add setup code

#  pragma mark Tastenfunktion

void tastenfunktion(uint16_t Tastenwert)
{
   if (Tastenwert>23) // ca Minimalwert der Matrix
   {
      //         wdt_reset();
      /*
       0: Wochenplaninit
       1: IOW 8* 2 Bytes auf Bus laden
       2: Menu der aktuellen Ebene nach oben
       3: IOW 2 Bytes vom Bus in Reg laden
       4: Auf aktueller Ebene nach rechts (Heizung: Vortag lesen und anzeigen)                           
       5: Ebene tiefer
       6: Auf aktueller Ebene nach links (Heizung: Folgetag lesen und anzeigen)                           
       7: 
       8: Menu der aktuellen Ebene nach unten
       9: DCF77 lesen
       
       12: Ebene höher
       */
      TastaturCount++;
      if (TastaturCount>=40)   //   Prellen
      {
         
      }
      
   }
   else 
   {
     }
}
uint16_t readTastatur(void)
{
   uint16_t adctastenwert = adc->adc0->analogRead(ANALOGTASTATUR);
   if (adctastenwert > 10)
   {
      //Serial.printf("readTastatur adctastenwert: %d\n",adctastenwert);
      return adctastenwert;
   }
   return 0;
}


/*
void C1_ISR(void)
{
   digitalWriteFast(MC_EN,HIGH);
   anschlagstatus |= (1<<END_C1);
   uint32_t posC = motor_C.getPosition();
   Serial.printf("* C1_ISR posC: %d\n",posC) ;

}
*/




void setup()
{
   Serial.begin(9600);
  // Wait for USB Serial
  while (!Serial) {
    yield();
  }

 // cout << F("SdFat version: ") << SD_FAT_VERSION_STR << endl;
//  printConfig(SD_CONFIG);



   pinMode(LOOPLED, OUTPUT);
   
   
   for (int i=0;i<NUM_SERVOS;i++)
   {
    int wert = 500 + i * 50;
       impulstimearray[i] = wert; // mittelwert
   //Serial.printf("i: %d wert:\t %d\n",i,wert);
   }
   
   // init Pins
   
   pinMode(pot0_PIN, INPUT);
   pinMode(pot1_PIN, INPUT);
   pinMode(pot2_PIN, INPUT);
   pinMode(pot3_PIN, INPUT);
   
   pinMode(IMPULSPIN, OUTPUT);
   digitalWriteFast(IMPULSPIN,0);
   
   servoTimer.begin(servopaketfunktion, 80000);
   microTimer.begin(microtimerfunktion,2);
   
  //   pinMode(END_C1_PIN, INPUT_PULLUP); // 
   
//   attachInterrupt(digitalPinToInterrupt(END_A0_PIN), A0_ISR, FALLING);
   {
      pinMode(OSZI_PULS_A, OUTPUT);
      digitalWriteFast(OSZI_PULS_A, HIGH); 

      pinMode(OSZI_PULS_B, OUTPUT);
      digitalWriteFast(OSZI_PULS_B, HIGH); 

      pinMode(OSZI_PULS_C, OUTPUT);
      digitalWriteFast(OSZI_PULS_C, HIGH); 
   }
   
   delay(100);
   
    
   

} // end setup

bool firstTry = true;

// Add loop code
void loop()
{
   /*
 clearSerialInput();
 // F stores strings in flash to save RAM
  cout << F("\ntype any character to start\n");
  while (!Serial.available()) {
    yield();
  }

 uint32_t t = millis();
  if (!sd.cardBegin(SD_CONFIG)) {
    cout << F(
           "\nSD initialization failed.\n"
           "Do not reformat the card!\n"
           "Is the card correctly inserted?\n"
           "Is there a wiring/soldering problem?\n");
    if (isSpi(SD_CONFIG)) {
      cout << F(
           "Is SD_CS_PIN set to the correct value?\n"
           "Does another SPI device need to be disabled?\n"
           );
    }
    errorPrint();
    return;
  }
  t = millis() - t;
  cout << F("init time: ") << t << " ms" << endl;

  if (!sd.card()->readCID(&m_cid) ||
      !sd.card()->readCSD(&m_csd) ||
      !sd.card()->readOCR(&m_ocr)) {
    cout << F("readInfo failed\n");
    errorPrint();
    return;
  }
  printCardType();
  cidDmp();
  csdDmp();
  cout << F("\nOCR: ") << uppercase << showbase;
  cout << hex << m_ocr << dec << endl;
  if (!mbrDmp()) {
    return;
  }
  if (!sd.volumeBegin()) {
    cout << F("\nvolumeBegin failed. Is the card formatted?\n");
    errorPrint();
    return;
  }
  dmpVol();
*/


  

   //   Serial.println(steps);
   //   threads.delay(1000);
   

   if (sinceblink > 500) 
   {   
      //digitalWrite(OSZI_PULS_A, !digitalRead(OSZI_PULS_A));
      

      sinceblink = 0;
      
       if (digitalRead(LOOPLED) == 1)
      {
         digitalWriteFast(LOOPLED, 0);
          
      }
      else
      {
         digitalWriteFast(LOOPLED, 1);
      }
      
   }// sinceblink
 
#  pragma mark motor finished
   if (sincelaststep > 50) // 50 us
   {
      
     // digitalWriteFast(OSZI_PULS_A, LOW); 
      
      
      tastencounter++;
      if (tastencounter > 10)
      {
         tastencounter = 0;
         taste = readTastatur();
         tastenfunktion(taste);
      }

      sincelaststep = 0;
      //     timerfunction();
      
   } // sincelaststep > 100
   

   
#pragma mark start_usb
   
//   r = usb_rawhid_recv((void*)buffer, 0); // 1.5us
if (sinceusb > 100)   
{
   sinceusb = 0;
   r = RawHID.recv(buffer, 0); 
   code = 0;
   if (r > 0) // 
   {
   //   noInterrupts();
      
      code = buffer[24];
      
      
      Serial.printf("\n***************************************  --->    rawhid_recv start code HEX: %02X\n",code);
      //Serial.printf("code: %d\n",code);
      usb_recv_counter++;
      //     lcd.setCursor(10,1);
      //     lcd.print(String(usb_recv_counter));
      //     lcd.setCursor(14,1);
      //     lcd.print(String(code));
      uint8_t device = buffer[32];
      sendbuffer[24] =  buffer[32];
      
      switch (code)
      {   
         case 0xA4:
         {
            Serial.printf("A4 clear\n");
         }break;
            
#pragma mark A5  GO HOME         
         case 0xA5: //  go home
         {
            
         }break;
  
#pragma mark default
         default:
         {
            
         }break; // default
            
            
            
      } // switch code
      interrupts();
 //     code=0;
   }// r > 0
   /**   End USB-routinen   ***********************/
 
} // since usb
   
#pragma mark sendstatus
   //if (sendstatus >= 3)
   //Serial.printf("\n++++++++++++++++++++++++++++++\nsendstatus: %d",sendstatus);

   
   interrupts();
   
   
   
   /**   End CNC-routinen   ***********************/
}
