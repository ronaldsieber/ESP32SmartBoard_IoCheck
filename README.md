# ESP32SmartBoard_IoCheck

This Arduino sketch is the elementary basic software for the hardware project [ESP32SmartBoard_PCB](https://github.com/ronaldsieber/ESP32SmartBoard_IoCheck) and uses the entire periphery of the board.

On the one hand, the sketch is a good starting point for new software projects based on this board. On the other hand, all components can be checked for their correct function after the board has been assembeld.

## Sketch Functionalities
The sketch implements the following functionalities:

**PowerOn / Reset:**
- LED0 ... LED8 operates as a gradually growing LED Bar
- The blue on-board LED of the ESP32DevKit flashes slowly (0.5 Hz)

**Button KEY0:**
- Same as PowerOn / Reset

**Button KEY1:**
- LED0 ... LED8 operates as a gradually growing inverse LED Bar
- The blue on-board LED of the ESP32DevKit flashes quickly (2 Hz)

**Button BLE_CFG:**
- The blue on-board LED of the ESP32DevKit lights up continuously as long as the button is pressed

The values ​​of the **DHT22** (temperature and humidity) and the **MH-Z19** (CO2 level and sensor temperature) are displayed in the serial terminal window (115200Bd) with a cycle interval of 5 seconds.

## Configuration Section

At the beginning of the sketch there is the following configuration section:

    const int CFG_ENABLE_KEY = 1;
    const int CFG_ENABLE_LED = 1;
    const int CFG_ENABLE_STATUS_LED = 1;
    const int CFG_ENABLE_DHT_SENSOR = 1;
    const int CFG_ENABLE_MHZ_SENSOR = 1;

This allows for enable *(= 1)* or disable *(= 0)* the runtime execution of the associated code sections. This avoids the occurrence of runtime failures on boards on which not all components are available (especially if the DHT22 or the MH-Z19 are not present). 
