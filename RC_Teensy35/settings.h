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





#define TIMER_ON    1 // Bit fuer timerfunktion start

#define pot0_PIN   14
#define pot1_PIN   A1
#define pot2_PIN   A2
#define pot3_PIN   A3




// neu 3.5:
#define CS1_PIN            24 // SPI 2




// Tastatur
#define ANALOGTASTATUR     22





#define OSZI_PULS_A        25
#define OSZI_PULS_B        26
#define OSZI_PULS_C        27
#define OSZI_PULS_D        28


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
