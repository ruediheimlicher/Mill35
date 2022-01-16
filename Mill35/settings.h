//
//  settings.h
//  Stepper32
//
//  Created by Ruedi Heimlicher on 07.05.2020.
//  Copyright © 2020 Ruedi Heimlicher. All rights reserved.
//

#ifndef settings_h
#define settings_h

#define LOOPLED 13
#define TIMER0_STARTWERT   0x40

// Stepper A


//Pins 3.5

#define MA_STEP           0
#define MA_RI             1
#define MA_EN             2

// Stepper B
#define MB_STEP           3
#define MB_RI             4
#define MB_EN             5


// Stepper C
#define MC_STEP            6           // 
#define MC_RI              7
#define MC_EN              8

#define BLINKLED           9     

// 10,11,12,13: SPI

#define END_A0_PIN         14
#define END_A1_PIN         15
#define END_B0_PIN         16
#define END_B1_PIN         17

// 18, 19: I2C

#define END_C0_PIN         20           // Anschlagstatus:  PIN fuer Endanschlag bei C0
#define END_C1_PIN         21  

// neu 3.5:
#define CS1_PIN            24 // SPI 2
#define LED_PWM_PIN        36
#define DRILL_PWM_PIN       35




// Tastatur
#define ANALOGTASTATUR     22





/*
#define UP_PIN             24
#define DOWN_PIN           25
#define LEFT_PIN           26
#define RIGHT_PIN          27

#define MILL_UP_PIN        28
#define MILL_DOWN_PIN      29
*/
#define HALT_PIN           32

#define DRILL_BIT  28
#define PWM_BIT  29



#define TASTE0            0   // HALT-Bit Motor A
#define TASTE1            1




#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define LOAD_NEXT          5  // Bit fuer Laden des naechsten Abschnitts in der loop
#define LOAD_LAST          6  // Bit fuer Laden des letzten Abschnitts in der loop

// X-Achse
#define END_A0          0       //  Bit fuer Endanschlag A0 
#define END_A1          1       //  Bit fuer Endanschlag A1 

// Y-Achse
#define END_B0          2       //           Endanschlag B0 
#define END_B1          3       //           Endanschlag B1 

// Spindel
#define END_C0          4      //  Bit fuer Endanschlag C0 
#define END_C1          5       //  Bit fuer Endanschlag C1 

#define HOME_A0          7       //  Home A
#define HOME_B0          8


#define FIRSTRUN          7

#define RICHTUNG_A0   0 // Motor A vorwaerts
#define RICHTUNG_A1   1 // Motor A rueckwaerts
#define RICHTUNG_B0   2
#define RICHTUNG_B1   3

#define RICHTUNG_C   2
#define RICHTUNG_D   3


#define COUNT_A            0 // 4      // Motorstatus:   Schritte von Motor A zaehlen
#define COUNT_B            1 // 5      // Motorstatus:   Schritte von Motor B zaehlen

#define COUNT_C            2 // 4      // Motorstatus:   Schritte von Motor C zaehlen
#define COUNT_D            3 // 5      // Motorstatus:   Schritte von Motor D zaehlen

#define COUNT_END          6
#define COUNT_LAST         7


#define TIMER_ON           1 // Bit fuer timerfunktion start


#define GO_HOME            3     // Bit fuer befehl beginn home auf cncstatus
#define DC_DIVIDER         1      // teilt die pwm-Frequenz in ISR




// Ringbuffer
#define RINGBUFFERTIEFE    4
#define READYBIT           0        // buffer kann Daten aufnehmen
#define FULLBIT            1        // Buffer ist voll
#define STARTBIT           2        // Buffer ist geladen
#define INNERBIT           3        // Ringbuffer wird verwendet
#define LASTBIT            4        // Letzter Abschnitt  ist geladen
#define ENDBIT             5        // Letzter Abschnitt  ist abgearbeitet
#define STOPBIT            6        // Ablauf stoppen
#define FIRSTBIT           7


#define OSZI_PULS_A        25
#define OSZI_PULS_B        26


#define THREAD_COUNT_BIT   0

#define TIMERINTERVALL 128

// Ramp
#define RAMP_OK      1 // Ramp einschalten
#define RAMPFAKTOR   2 // Verlaengerung der delayzeit
#define RAMPZEIT     800 // Mindestdauer fuer Ramp

#define RAMPSTARTBIT 1
#define RAMPENDBIT 2
#define RAMPEND0BIT 3 // Beginn der Endrampe
#define RAMPOKBIT    7
#define RAMPSCHRITT  10

// new
#define STARTIMPULSDAUER   100
#define ENDIMPULSDAUER     20
#define TASTENENDIMPULSDAUER     20

#define RAMPSCHRITT        4
# define RAMPDELAY 80 // delay fuerr Reduktion Impulsdauer
// revision
#define TASTE1     67
#define TASTE2     109
#define TASTE3     163
#define TASTE4     253
#define TASTE5     360
#define TASTE6     484
#define TASTE7     628
#define TASTE8     742
#define TASTE9     827
#define TASTEL     899
#define TASTE0     946
#define TASTER     993


// revision end

#define DEVICE_MILL  1
#define DEVICE_JOY  2

#define VORZEICHEN_X   0
#define VORZEICHEN_Y   1

#endif /* settings_h */
