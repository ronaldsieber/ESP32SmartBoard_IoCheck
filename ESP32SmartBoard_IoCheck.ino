/****************************************************************************

  Copyright (c) 2021 Ronald Sieber

  Project:      ESP32SmartBoard
  Description:  Basic IO Routines to support Commisioning of ESP32SmartBoard

  -------------------------------------------------------------------------

    Arduino IDE Settings:

    Board:              "ESP32 Dev Module"
    Upload Speed:       "115200"
    CPU Frequency:      "240MHz (WiFi/BT)"
    Flash Frequency:    "80Mhz"
    Flash Mode:         "QIO"
    Flash Size:         "4MB (32Mb)"
    Partition Scheme:   "No OTA (2MB APP/2MB SPIFFS)"
    PSRAM:              "Disabled"

  -------------------------------------------------------------------------

  Revision History:

  2021/01/22 -rs:   V1.00 Initial version

****************************************************************************/


#include <DHT.h>                                                        // Requires Library "DHT sensor library" by Adafruit
#include <MHZ19.h>                                                      // Requires Library "MH-Z19" by Jonathan Dempsey
#include <SoftwareSerial.h>                                             // Requires Library "EspSoftwareSerial" by Dirk Kaar, Peter Lerup





/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
//  Application Configuration
//---------------------------------------------------------------------------

const int       APP_VERSION                         = 1;                // 1.xx
const int       APP_REVISION                        = 10;               // x.00
const char      APP_BUILD_TIMESTAMP[]               = __DATE__ " " __TIME__;

const int       CFG_ENABLE_KEY                      = 1;
const int       CFG_ENABLE_LED                      = 1;
const int       CFG_ENABLE_STATUS_LED               = 1;
const int       CFG_ENABLE_DHT_SENSOR               = 1;
const int       CFG_ENABLE_MHZ_SENSOR               = 1;



//---------------------------------------------------------------------------
//  Hardware/Pin Configuration
//---------------------------------------------------------------------------

const int       PIN_KEY_BLE_CFG                     = 36;               // PIN_KEY_BLE_CFG      (GPIO36 -> Pin02)
const int       PIN_KEY0                            = 35;               // KEY0                 (GPIO35 -> Pin05)
const int       PIN_KEY1                            = 34;               // KEY1                 (GPIO34 -> Pin04)

const int       PIN_LED0                            = 13;               // LED0 (green)         (GPIO13 -> Pin13)
const int       PIN_LED1                            = 12;               // LED1 (green)         (GPIO12 -> Pin12)
const int       PIN_LED2                            = 14;               // LED2 (green)         (GPIO14 -> Pin11)
const int       PIN_LED3                            =  4;               // LED3 (yellow)        (GPIO04 -> Pin20)
const int       PIN_LED4                            =  5;               // LED4 (yellow)        (GPIO05 -> Pin23)
const int       PIN_LED5                            = 18;               // LED5 (yellow)        (GPIO18 -> Pin24)
const int       PIN_LED6                            = 19;               // LED6 (red)           (GPIO19 -> Pin25)
const int       PIN_LED7                            = 21;               // LED7 (red)           (GPIO21 -> Pin26)
const int       PIN_LED8                            = 22;               // LED8 (red)           (GPIO22 -> Pin29)
const int       PIN_STATUS_LED                      =  2;               // On-board LED (blue)  (GPIO02 -> Pin19)

const uint32_t  LED_BAR_SET_PERIOD                  = 500;              // Period for Set next LED Bar value in [ms]
const uint32_t  STATUS_LED_TOGGLE_PERIOD_SLOW       = 1000;             // Slow Toggle Period for Status LED in [ms]
const uint32_t  STATUS_LED_TOGGLE_PERIOD_FAST       = 125;              // Fast Toggle Period for Status LED in [ms]

const int       DHT_TYPE                            = DHT22;            // DHT11, DHT21 (AM2301), DHT22 (AM2302,AM2321)
const int       DHT_PIN_SENSOR                      = 23;               // PIN used for DHT22 (AM2302/AM2321)
const uint32_t  DHT_SENSOR_SAMPLE_PERIOD            = 5000;             // Sample Period for DHT-Sensor in [ms]

const int       MHZ19_PIN_SERIAL_RX                 = 32;               // ESP32 Rx pin which the MH-Z19 Tx pin is attached to
const int       MHZ19_PIN_SERIAL_TX                 = 33;               // ESP32 Tx pin which the MH-Z19 Rx pin is attached to
const int       MHZ19_BAUDRATE_SERIAL               = 9600;             // Serial baudrate for communication with MH-Z19 Device
const uint32_t  MHZ19_SENSOR_SAMPLE_PERIOD          = 5000;             // Sample Period for MH-Z19-Sensor in [ms]



//---------------------------------------------------------------------------
//  Local Variables
//---------------------------------------------------------------------------

static  String          strChipID_g;

static  int             iLastStateKeyBleCfg_g       = -1;
static  int             iLastStateKey0_g            = -1;
static  int             iLastStateKey1_g            = -1;

static  uint32_t        ui32LedBarValue_g           = 0;
static  bool            fInvertLedBar_g             = false;
static  uint32_t        ui32LastTickLedBarSet_g     = 0;

static  volatile byte   bStatusLedState_g           = LOW;
static  bool            fLockStatusLedOn_g          = false;
static  uint32_t        ui32StatusLedTogglePeriod_g = STATUS_LED_TOGGLE_PERIOD_SLOW;
static  uint32_t        ui32LastTickStatLedToggle_g = 0;

static  DHT             DhtSensor_g(DHT_PIN_SENSOR, DHT_TYPE);
static  uint32_t        ui32LastTickDhtRead_g       = 0;

static  MHZ19           Mhz19Sensor_g;
static  SoftwareSerial  Mhz19SoftSerial_g(MHZ19_PIN_SERIAL_RX, MHZ19_PIN_SERIAL_TX);
static  uint32_t        ui32LastTickMhz19Read_g     = 0;





//=========================================================================//
//                                                                         //
//          S K E T C H   P U B L I C   F U N C T I O N S                  //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Setup
//---------------------------------------------------------------------------

void setup()
{

char  szTextBuff[64];
int   iIdx;


    // Serial console
    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println();
    Serial.println("======== APPLICATION START ========");
    Serial.println();


    // Application Version Information
    snprintf(szTextBuff, sizeof(szTextBuff), "App Version:      %u.%02u", APP_VERSION, APP_REVISION);
    Serial.println(szTextBuff);
    snprintf(szTextBuff, sizeof(szTextBuff), "Build Timestamp:  %s", APP_BUILD_TIMESTAMP);
    Serial.println(szTextBuff);
    Serial.println();
    delay(100);


    // Device Identification
    strChipID_g = GetChipID();
    Serial.print("Unique ChipID: ");
    Serial.println(strChipID_g);
    Serial.println();
    delay(100);


    // Setup Input Pins for Keys
    if ( CFG_ENABLE_KEY )
    {
        Serial.println("Setup Input Pins for Keys...");
        pinMode(PIN_KEY_BLE_CFG, INPUT);
        pinMode(PIN_KEY0, INPUT);
        pinMode(PIN_KEY1, INPUT);
    }
    iLastStateKeyBleCfg_g = -1;
    iLastStateKey0_g      = -1;
    iLastStateKey1_g      = -1;


    // Setup Output Pins for LEDs
    if ( CFG_ENABLE_LED )
    {
        Serial.println("Setup Output Pins for LEDs...");
        pinMode(PIN_LED0, OUTPUT);
        pinMode(PIN_LED1, OUTPUT);
        pinMode(PIN_LED2, OUTPUT);
        pinMode(PIN_LED3, OUTPUT);
        pinMode(PIN_LED4, OUTPUT);
        pinMode(PIN_LED5, OUTPUT);
        pinMode(PIN_LED6, OUTPUT);
        pinMode(PIN_LED7, OUTPUT);
        pinMode(PIN_LED8, OUTPUT);
        fInvertLedBar_g = false;
        ui32LastTickLedBarSet_g = 0;
    }


    // Status LED Setup
    if ( CFG_ENABLE_STATUS_LED )
    {
        pinMode(PIN_STATUS_LED, OUTPUT);
        bStatusLedState_g = LOW;
        fLockStatusLedOn_g = false;
        ui32StatusLedTogglePeriod_g = STATUS_LED_TOGGLE_PERIOD_SLOW;
        ui32LastTickStatLedToggle_g = 0;
    }


    // Setup DHT22 Sensor (Temerature/Humidity)
    if ( CFG_ENABLE_DHT_SENSOR )
    {
        Serial.println("Setup DHT22 Sensor...");
        DhtSensor_g.begin();
        AppProcessDhtSensor(0);                                         // get initial values
    }


    // Setup MH-Z19 Sensor (CO2)
    if ( CFG_ENABLE_MHZ_SENSOR )
    {
        Serial.println("Setup MH-Z19 Sensor...");
        Mhz19SoftSerial_g.begin(MHZ19_BAUDRATE_SERIAL);                 // Initialize Software Serial Device to communicate with MH-Z19 sensor
        Mhz19Sensor_g.begin(Mhz19SoftSerial_g);                         // Initialize MH-Z19 Sensor (using Software Serial Device)
        Mhz19Sensor_g.autoCalibration();                                // Turn Auto Calibration ON (OFF = autoCalibration(false))
        AppProcessMhz19Sensor(0);                                       // get initial values
    }

    return;

}



//---------------------------------------------------------------------------
//  Application Main Loop
//---------------------------------------------------------------------------

void loop()
{

    // Process Input Pins for Keys
    if ( CFG_ENABLE_KEY )
    {
        AppProcessKey();
    }


    // Process Output Pins for LEDs
    if ( CFG_ENABLE_LED )
    {
        AppProcessLedBar(LED_BAR_SET_PERIOD);
    }


    // Process Status LED
    if ( CFG_ENABLE_STATUS_LED )
    {
        AppProcessStatusLed(ui32StatusLedTogglePeriod_g);
    }


    // Process DHT Sensor (Temperature/Humidity)
    if ( CFG_ENABLE_DHT_SENSOR )
    {
        AppProcessDhtSensor(DHT_SENSOR_SAMPLE_PERIOD);
    }

    // Process MH-Z19 Sensor (CO2Value/SensorTemperature)
    if ( CFG_ENABLE_MHZ_SENSOR )
    {
        AppProcessMhz19Sensor(MHZ19_SENSOR_SAMPLE_PERIOD);
    }

    return;

}





//=========================================================================//
//                                                                         //
//          S K E T C H   P R I V A T E   F U N C T I O N S                //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Process Keys
//---------------------------------------------------------------------------

void  AppProcessKey()
{

int  iStateKeyBleCfg;
int  iStateKey0;
int  iStateKey1;


    // process KEY_BLE_CFG
    iStateKeyBleCfg = (digitalRead(PIN_KEY_BLE_CFG) == LOW) ? 1 : 0;
    if (iLastStateKeyBleCfg_g != iStateKeyBleCfg)
    {
        if (iStateKeyBleCfg == 1)
        {
            Serial.println("State changed: KEY_BLE_CFG=1");
            fLockStatusLedOn_g = true;
        }
        else
        {
            Serial.println("State changed: KEY_BLE_CFG=0");
            fLockStatusLedOn_g = false;
        }

        iLastStateKeyBleCfg_g = iStateKeyBleCfg;
    }


    // process KEY0
    iStateKey0 = (digitalRead(PIN_KEY0) == LOW) ? 1 : 0;
    if (iLastStateKey0_g != iStateKey0)
    {
        if (iStateKey0 == 1)
        {
            Serial.println("State changed: KEY0=1");
            ui32StatusLedTogglePeriod_g = STATUS_LED_TOGGLE_PERIOD_SLOW;
            fInvertLedBar_g = false;
        }
        else
        {
            Serial.println("State changed: KEY0=0");
        }

        iLastStateKey0_g = iStateKey0;
    }


    // process KEY1
    iStateKey1 = (digitalRead(PIN_KEY1) == LOW) ? 1 : 0;
    if (iLastStateKey1_g != iStateKey1)
    {
        if (iStateKey1 == 1)
        {
            Serial.println("State changed: KEY1=1");
            ui32StatusLedTogglePeriod_g = STATUS_LED_TOGGLE_PERIOD_FAST;
            fInvertLedBar_g = true;
        }
        else
        {
            Serial.println("State changed: KEY1=0");
        }

        iLastStateKey1_g = iStateKey1;
    }

    return;

}



//---------------------------------------------------------------------------
//  Process LEDs
//---------------------------------------------------------------------------

void  AppProcessLedBar (uint32_t ui32LedBarSetInterval_p)
{

uint32_t  ui32CurrTick;


    ui32CurrTick = millis();
    if ((ui32CurrTick - ui32LastTickLedBarSet_g) >= ui32LedBarSetInterval_p)
    {
        ui32LedBarValue_g++;
        AppPresentLedBar ((ui32LedBarValue_g % 10), fInvertLedBar_g);

        ui32LastTickLedBarSet_g = ui32CurrTick;
    }

    return;

}



//---------------------------------------------------------------------------
//  Present LED Bar (0 <= iBarValue_p <= 9)
//---------------------------------------------------------------------------

void  AppPresentLedBar (int iBarValue_p, bool fInvertBar_p)
{

uint32_t  ui32BarBitMap;


    if (iBarValue_p < 0)
    {
        ui32BarBitMap = 0x0000;             // set LED0..LED8 = OFF
    }
    else if (iBarValue_p > 9)
    {
        ui32BarBitMap = 0x01FF;             // set LED0..LED8 = ON
    }
    else
    {
        ui32BarBitMap = 0x0000;
        while (iBarValue_p > 0)
        {
            ui32BarBitMap <<= 1;
            ui32BarBitMap  |= 1;
            iBarValue_p--;
        }
    }

    if ( fInvertBar_p )
    {
        ui32BarBitMap ^= 0x01FF;
    }

    digitalWrite(PIN_LED0, (ui32BarBitMap & 0b000000001) ? HIGH : LOW);
    digitalWrite(PIN_LED1, (ui32BarBitMap & 0b000000010) ? HIGH : LOW);
    digitalWrite(PIN_LED2, (ui32BarBitMap & 0b000000100) ? HIGH : LOW);
    digitalWrite(PIN_LED3, (ui32BarBitMap & 0b000001000) ? HIGH : LOW);
    digitalWrite(PIN_LED4, (ui32BarBitMap & 0b000010000) ? HIGH : LOW);
    digitalWrite(PIN_LED5, (ui32BarBitMap & 0b000100000) ? HIGH : LOW);
    digitalWrite(PIN_LED6, (ui32BarBitMap & 0b001000000) ? HIGH : LOW);
    digitalWrite(PIN_LED7, (ui32BarBitMap & 0b010000000) ? HIGH : LOW);
    digitalWrite(PIN_LED8, (ui32BarBitMap & 0b100000000) ? HIGH : LOW);

    return;

}



//---------------------------------------------------------------------------
//  Process Status LED
//---------------------------------------------------------------------------

void  AppProcessStatusLed (uint32_t ui32StatusLedToggleInterval_p)
{

uint32_t  ui32CurrTick;


    if ( fLockStatusLedOn_g )
    {
        digitalWrite(PIN_STATUS_LED, HIGH);
    }
    else
    {
        ui32CurrTick = millis();
        if ((ui32CurrTick - ui32LastTickStatLedToggle_g) >= ui32StatusLedToggleInterval_p)
        {
            bStatusLedState_g = !bStatusLedState_g;
            digitalWrite(PIN_STATUS_LED, bStatusLedState_g);

            ui32LastTickStatLedToggle_g = ui32CurrTick;
        }
    }

    return;

}



//---------------------------------------------------------------------------
//  Process DHT Sensor (Temperature/Humidity)
//---------------------------------------------------------------------------

void  AppProcessDhtSensor (uint32_t ui32DhtReadInterval_p)
{

uint32_t  ui32CurrTick;
float     flTemperature;
float     flHumidity;
String    strTemperature;
String    strHumidity;


    ui32CurrTick = millis();
    if ((ui32CurrTick - ui32LastTickDhtRead_g) >= ui32DhtReadInterval_p)
    {
        flTemperature = DhtSensor_g.readTemperature(false);             // false = Temp in Celsius degrees, true = Temp in Fahrenheit degrees
        flHumidity    = DhtSensor_g.readHumidity();

        if ( isnan(flTemperature) || isnan(flHumidity) )
        {
            Serial.println("ERROR: Failed to read from DHT sensor!");
            return;
        }

        strTemperature = String(flTemperature, 1);                      // convert float to String with one decimal place
        Serial.print("Temperature: ");
        Serial.println(strTemperature);

        strHumidity = String(flHumidity, 1);                            // convert float to String with one decimal place
        Serial.print("Humidity: ");
        Serial.println(strHumidity);

        ui32LastTickDhtRead_g = ui32CurrTick;
    }

    return;

}



//---------------------------------------------------------------------------
//  Process MH-Z19 Sensor (CO2Value/SensorTemperature)
//---------------------------------------------------------------------------

void  AppProcessMhz19Sensor (uint32_t ui32Mhz19ReadInterval_p)
{

uint32_t  ui32CurrTick;
int       iMhz19CO2Value;
int       iMhz19CO2SensTemp;
String    strCO2Value;
String    strCO2SensTemp;


    ui32CurrTick = millis();
    if ((ui32CurrTick - ui32LastTickMhz19Read_g) >= ui32Mhz19ReadInterval_p)
    {
        iMhz19CO2Value    = Mhz19Sensor_g.getCO2();                     // MH-Z19: Request CO2 (as ppm)
        iMhz19CO2SensTemp = Mhz19Sensor_g.getTemperature();             // MH-Z19: Request Sensor Temperature (as Celsius)

        strCO2Value = String(iMhz19CO2Value);                           // convert int to String
        Serial.print("CO2(ppm): ");
        Serial.println(strCO2Value);

        strCO2SensTemp = String(iMhz19CO2SensTemp);                     // convert int to String
        Serial.print("SensTemp: ");
        Serial.println(strCO2SensTemp);

        ui32LastTickMhz19Read_g = ui32CurrTick;
    }

    return;

}





//=========================================================================//
//                                                                         //
//          P R I V A T E   G E N E R I C   F U N C T I O N S              //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
//  Get Unique Client Name
//---------------------------------------------------------------------------

String  GetUniqueClientName (const char* pszClientPrefix_p)
{

String  strChipID;
String  strClientName;


    // Create a unique client name, based on ChipID (the ChipID is essentially its 6byte MAC address)
    strChipID = GetChipID();
    strClientName  = pszClientPrefix_p;
    strClientName += strChipID;

    return (strClientName);

}



//---------------------------------------------------------------------------
//  Get ChipID as String
//---------------------------------------------------------------------------

String  GetChipID()
{

String  strChipID;


    strChipID = GetEsp32MacId (false);

    return (strChipID);

}



//---------------------------------------------------------------------------
//  Get ChipMAC as String
//---------------------------------------------------------------------------

String  GetChipMAC()
{

String  strChipMAC;


    strChipMAC = GetEsp32MacId (true);

    return (strChipMAC);

}



//---------------------------------------------------------------------------
//  Get GetEsp32MacId as String
//---------------------------------------------------------------------------

String  GetEsp32MacId (bool fUseMacFormat_p)
{

uint64_t  ui64MacID;
String    strMacID;
byte      bDigit;
char      acDigit[2];
int       iIdx;


    ui64MacID = ESP.getEfuseMac();
    strMacID = "";
    for (iIdx=0; iIdx<6; iIdx++)
    {
        bDigit = (byte) (ui64MacID >> (iIdx * 8));
        sprintf(acDigit, "%02X", bDigit);
        strMacID += String(acDigit);

        if (fUseMacFormat_p && (iIdx<5))
        {
            strMacID += ":";
        }
    }

    strMacID.toUpperCase();

    return (strMacID);

}




// EOF
