/*


  emonBase - radio using JeeLib RFM69 "Native" format

  
   ------------------------------------------
  Part of the openenergymonitor.org project

  Authors: Glyn Hudson, Trystan Lea & Robert Wall
  Builds upon JCW JeeLabs RF69 Driver and Arduino

  Licence: GNU GPL V3


Change Log:
*/
const char *firmware_version = {"2.0.0\n\r"};
/*

V2.0.0  10/07/21 Derived from emonLibCM examples and original emonPi sketch, that being derived from 
https://github.com/openenergymonitor/emonpi/blob/master/Atmega328/emonPi_RFM69CW_RF12Demo_DiscreteSampling
and emonLibCM example sketches, with config input based on emonTx V3 sketches.

*********************************************************************************************
*                                                                                           *
*                                  IMPORTANT NOTE                                           *
*                                                                                           *
* When compiling for the RFM2Pi:                                                            * 
*  In IDE, set Board to "Arduino Pro or Pro Mini" & Processor to "ATmega328P (3.3V, 8MHz)"  *
* When compiling for the RFM69Pi:                                                           *
*  In IDE, set Board to "Arduino Uno"                                                       *
*                                                                                           *
* The output file used must NOT be the "with_bootloader" version, else the processor will   *
*  be locked.                                                                               *
*********************************************************************************************
*/


// #define SAMPPIN 19 

#include <emonEProm.h>
// OEM EPROM library

// RFM interface
#include "spi.h"                                                       // Requires "RFM69 Native" JeeLib Driver
#include "rf69.h"
// typedef SpiDev<10> SpiDev10;
RF69<SpiDev10> rf;

bool rfDataAvailable = false;

byte nativeMsg[66];                                                    // 'Native' format message buffer

#define MAXMSG 66                                                      // Max length of o/g message
char outmsg[MAXMSG];                                                   // outgoing message (to emonGLCD etc)
byte outmsgLength;                                                     // length of message: non-zero length triggers transmission

struct {                                                               // Ancilliary information
  byte srcNode = 0;
  byte msgLength = 0;
  signed char rssi = -127;
  bool crc = false;
} rfInfo;

enum rfband {RFM_433MHZ = 1, RFM_868MHZ, RFM_915MHZ };                 // frequency band.
bool rfChanged = false;                                                // marker to re-initialise radio
#define RFTX 0x01                                                      // Control of RFM - transmit enabled


//----------------------------emonBase Settings------------------------------------------------------------------------------------------------
bool debug                      = true;
bool verbose                    = false;
const unsigned long BAUD_RATE   = 38400;


void single_LED_flash(void);
void double_LED_flash(void);
void getCalibration(void);
void send_emonpi_serial();
void emonPi_startup(void);
static void showString (PGM_P s);


//---------------------------- Settings - Stored in EEPROM and shared with config.ino ------------------------------------------------
struct {
  byte RF_freq = RFM_433MHZ;                            // Frequency of radio module can be RFM_433MHZ, RFM_868MHZ or RFM_915MHZ. 
  byte rfPower = 25;                                    // Power when transmitting
  byte networkGroup = 210;                              // wireless network group, must be the same as emonBase / emonPi and emonGLCD. OEM default is 210
  byte  nodeID = 5;                                     // node ID for this emonBase.
  byte  rfOn = 0x1;                                     // Turn transmitter on -receiver is always on
} EEProm;

uint16_t eepromSig = 0x0017;                            // oemEProm signature - see oemEEProm Library documentation for details.


//--------------------------- hard-wired connections ----------------------------------------------------------------------------------------

const byte LEDpin               = 9;                                   // emonPi LED - on when HIGH

// Use D5 for ISR timimg checks - only if Pi is not connected!

//-------------------------------------------------------------------------------------------------------------------------------------------


/**************************************************************************************************************************************************
*
* SETUP        Set up & start the radio
*
***************************************************************************************************************************************************/
void setup() 
{  

  emonPi_startup();

  delay(2000);

  rf.init(EEProm.nodeID, EEProm.networkGroup, 
               EEProm.RF_freq == RFM_915MHZ ? 915                      // Fall through to 433 MHz Band @ 434 MHz
            : (EEProm.RF_freq == RFM_868MHZ ? 868 : 434)); 
  digitalWrite(LEDpin, LOW);
}

/**************************************************************************************************************************************************
*
* LOOP         Poll the radio for incoming data, and the serial input for calibration & outgoing r.f. data 
*
***************************************************************************************************************************************************/

void loop()             
{
  
//-------------------------------------------------------------------------------------------------------------------------------------------
// RF Data handler - inbound ****************************************************************************************************************
//-------------------------------------------------------------------------------------------------------------------------------------------

  int len = rf.receive(&nativeMsg, sizeof(nativeMsg));                 // Poll the RFM buffer and extract the data
  if (len > 1)
  {
    rfInfo.crc = true;
    rfInfo.msgLength = len;
    rfInfo.srcNode = nativeMsg[1];
    rfInfo.rssi = -rf.rssi/2;

    // send serial data
    Serial.print(F("OK"));                                             // Bad packets (crc failure) are discarded by RFM69CW
    print_frame(rfInfo.msgLength);		                                 // Print received data
    double_LED_flash();
  }
  
//-------------------------------------------------------------------------------------------------------------------------------------------
// RF Data handler - outbound ***************************************************************************************************************
//-------------------------------------------------------------------------------------------------------------------------------------------


	if ((EEProm.rfOn & RFTX) && outmsgLength) {                          // if command 'outmsg' is waiting to be sent then let's send it
    showString(PSTR(" -> "));
    Serial.print((word) outmsgLength);
    showString(PSTR(" b\n"));
    rf.send(0, (void *)outmsg, outmsgLength);                          //  void RF69<SPI>::send (uint8_t header, const void* ptr
    outmsgLength = 0;
    single_LED_flash();
	}

  
//-------------------------------------------------------------------------------------------------------------------------------------------
// Calibration Data handler *****************************************************************************************************************
//-------------------------------------------------------------------------------------------------------------------------------------------

  if (Serial.available())                                              // Serial input from RPi for configuration/calibration
  {
    getCalibration();                                                  // If serial input is received from RPi
    double_LED_flash();
    if (rfChanged)
    {
      rf.init(EEProm.nodeID, EEProm.networkGroup,                      // Reset the RFM69CW if NodeID, Group or frequency has changed.
            EEProm.RF_freq == RFM_915MHZ ? 915 : (EEProm.RF_freq == RFM_868MHZ ? 868 : 434)); 
      rfChanged = false;
    }
  }

}

/**************************************************************************************************************************************************
*
* SEND RECEIVED RF DATA TO RPi (/dev/ttyAMA0)
*
***************************************************************************************************************************************************/

void print_frame(int len) 
{
  Serial.print(F(" "));
  Serial.print(rfInfo.srcNode);        // Extract and print node ID
  Serial.print(F(" "));
  for (byte i = 2; i < len; ++i) 
  {
    Serial.print((word)nativeMsg[i]);
    Serial.print(F(" "));
  }
  Serial.print(F("("));
  Serial.print(rfInfo.rssi);
  Serial.print(F(")"));
  Serial.println();
}


/**************************************************************************************************************************************************
*
* LED flash
*
***************************************************************************************************************************************************/

void single_LED_flash(void)
{
  digitalWrite(LEDpin, HIGH);  delay(30); digitalWrite(LEDpin, LOW);
}

void double_LED_flash(void)
{
  digitalWrite(LEDpin, HIGH);  delay(20); digitalWrite(LEDpin, LOW); delay(60); 
  digitalWrite(LEDpin, HIGH);  delay(20); digitalWrite(LEDpin, LOW);
}


/**************************************************************************************************************************************************
*
* Startup      Set I/O pins, print initial message, read configuration from EEPROM
*
***************************************************************************************************************************************************/

void emonPi_startup()
{
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin,HIGH);

  Serial.begin(BAUD_RATE);
  Serial.print(F("|emonBase V")); Serial.write(firmware_version);
  Serial.println(F("|OpenEnergyMonitor.org"));
  load_config();                                                      // Load RF config from EEPROM (if any exists)

#ifdef SAMPPIN
  pinMode(SAMPPIN, OUTPUT);
  digitalWrite(SAMPPIN, LOW);
#endif
  
}

