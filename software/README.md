# BMS Software implementation NO_RTOS Version
Software implemented based on hardware desing by version QUTMS-BMS-V01 PDF file.

Attention: Floating value operations enabled at C/C++ Build -> Tool Settings

Flag _printf_float Disabled. RAM overflow by 2048 bytes.

RTOS version keeps it disabled.

## Layour setup


### BQ76930 - Voltage Sensing, Balancing
I^2C Communication at Standard Mode with 100KHz. No CRC.

    I2C1 GPIO Configuration

    PB6     ------> I2C1_SCL
    PB7     <-----> I2C1_SDA
    PB11    <------ CELL ALERT (EXTI11 NVIC interupt on Rising edge)
    The CRC polynomial is x8 + x2 + x + 1, and the initialvalue is 0.

ADC sampler for voltage reading tend to be inperfect. Based on each break ID
a separate table has to be created to accumulate offset from multimeter
reading.


### TMP05 - Digital Temperature Sensor
4 Temperature Line with 4-5 Daisy Chained Sensors.
    
    GPIO Configuration
    PA15    <------ INPUT line TEMP1 
    PA3     <------ INPUT line TEMP2
    PB3     <------ INPUT line TEMP3
    PB4     <------ INPUT line TEMP4
    PB5     <------ INPUT line TEMP5

    Timer 3 as a sensing reader at frequency 1MHz?? - 1nS??
    TIM3 - (Internal!! Clock - change to Oscilator at 8MHz)
        Prescaler       - 8-1
        Counter Period  - 0xffff-1 = 65534

2 Output line for Pulse Signal and Alert sequence.

    GPIO Conf and Pulse description.
    PA4     ------> Output initial Pulse line
    PA5     ------> Output Alarm if BMS at fault state nALARM

    Ready Pulse has following form:
        SET   PA4  - HIGH;
        WAIT       - Withing time 20nS > delay > 25us; delay_us(2)
        RESET PA4  - LOW;
        Enable Timer3 interupt and start recording

### ID Definition
16 positional switch over 4 pins. Used to define initial ID of the BMS.
Later applied for CAN bus and fault identification within Greek Gods names.

    GPIO Configuration
        PB0    <------ INPUT ID1
        PB1    <------ INPUT ID2
        PB2    <------ INPUT ID3
        PB10   <------ INPUT ID4

??? The way for ID placing is meant to be decided.

### CAN Bus setup
CAN protocol used over CAN4 line to communicate between AMS, BMS and Masters'
board. Protocol and package outlines:

    GPIO Configuration
        PA11    <------ CAN RX
        PB12    ------> CAN TX
    CAN Configuration.
        Prescaler for Time Quantum      - 8
        Time Quanta Bit 1               - 9 Times
        Time Quanta Bit 2               - 8 Times
        ReSync Jump Width               - 1 Time

        Receive Fifo Locked Mode        Enable

    CAN Software & Filter.
        	TxHeader.ExtId = board physical ID;
            TxHeader.IDE = CAN_ID_EXT;
            TxHeader.RTR = CAN_RTR_DATA;
            TxHeader.DLC = 2;
            TxHeader.TransmitGlobalTime = DISABLE;

            TxData[8] - uint8_t by 8.

### WatchDog Timer - WWDG
Window WatchDoG Timer. Used to reset software of the board if Timer does not get 
reseted at set of time. There are 2 of them to configurate. This one goes with
Windowd approach.
    
    WWDG Configuration
        Counter Clock Prescaler     - 1
        Window value                - 64
        Free-running Down Counter   - 64

        Early WakeUp interupt       Disabled
The refresh function to reset the timer has been placed after temperature
reading completed. If, by some accident,some tempsegment will be skipped, 
board will be restarted.

### Oscilator and Clock Configuration
Board consists of 2 clock sources to use and requires 2 mods to operate. 
Internal was tested in range of Frequencies of 1-48 MHz. 
External Oscilator uses 8MHz silicon oscilator.

Before boards moves from shipping mode, the internal frequency has to be set to
the lowest frequency possible. 1Mhz has been tested on Internal clock.
Theoretically, with external one, it can be no less than 100Khz as I2C uses
same one to tranmit messages. Current Consuptions has to be within Micro Amps.

After board has been moved into Normal mode, the Frequency sets to the 8MHz.
With external oscilator, it requires validation. A separate function has to be
implemented to switch between those mods.

TimeBase Source set to SysTick. RTOS reliably relies on one of the timers.
If required, set to TIM2.

3 Timers used for different purposes. Tim1 as internal microsecond clock,
Tim2 as a SysTick if necessary, Tim3 as Temperature capturer.

    Timer 1 as a source of us micro seconds. Internal!! Clock - Fix it.
    Timer 1 as a us at ***
    TIM1 - (Internal!! Clock - change to Oscilator at 8MHz)
        Prescaler       - 8-1
        Counter Period  - 0xffff-1 = 65534



### DEBUG: USART output
Asynchronous USB communication with converter over TX, RX and common GND.
No need for extra configuration to avoid errors in transmission.
Server as a debugger and has to be eliminated at production stage.

    No Flow control for RS232
    Baud Rate   - 38400 Bits/s
    Word Length - 8 Bits (Including Parity)
    Parity Bit  - None
    Stop Bit    - 1
### DEBUG: 2 LED outputs
2 LEDs serve as board heartbeat. Normally has to be disabled at Shipping mod.

??? Check the mod, I keep forget which one is which. Set to Open-Drain.
Default Off state - HIGH.

    GPIO Configuration
    PB14    ------> Output to LED0 Out and 3.3V over 330R for current limit.
    PB15    ------> Output to LED1 Out and 3.3V over 330R for current limit.

    Software:
    HAL id                      - huart1
    Message char buffer size    - 256

## Seting unused pins as analog - ENABLED.
!!! Determine how much power in actually optimizes.

## Following psudocode outlines the procedure.

    Power the MCU;
    Initialize clock at lowest frequency (currently 1MHz);
    while(ShipMod) remain here;
    Set Clock to 8MHz;
    Initialize HAL peripherals;    
    Define Local private variables;
    Enable Timer 1 and 3 interupt.
    ...
    Read status of BQ controller.
    ?? Test TMPs (send a pulse signal, if all lines trigger high - they safe)
    Determine ID of the board, store as CAN ExtID.

    Infinite loop {
        Set LED0 HIGH
        for every available cell {
            Read & store single voltage
            Set LED1 HIGH
            Capture all TMPS and accumulate 
            Set LED1 LOW
            Transmit Voltage reading.
        }
        Set LED0 LOW
        Tranmit and Average all TMPs
        Tranmit and Calculate Total Voltage
        ...
    }

## This is a psudocode of the load balancer used on MAX14920
    voltageThreashold = 0.27V
    balansingTimer = 1250ms     // No idea where that number came from
    cutOffMinVoltage = 2.9
    cutOffMaxVoltage = 3.4

    If Balancing Disable due to Discharge - Set registers to zero
    Else {
        
        for every available cell {
            if averageCellVoltage > Max {
                update max
                MarkForBalancing
            }
            if averageCellVoltage < Min {
                update min
                MarkForDisabelingBalancing
                Record as potentially unhealthy cell
            }

            voltageDifference = max - min
            if(voltageDiffernece >= voltageThreashold && time < balancingTimer) {
                Enable Balancing on Max
                Make sure Balancing disabled on Min.
                Report balancing cell to either turn off voltage reading or
                adjust based on Internal Resistance
            }
            else {
                Send Disabled balancing command.
                Report unhealthy cell
            }
        }
    }

## Psudocode for alarm trigger due to overcharge or undercharge
    cutOffMinVoltage = 2.9
    cutOffMaxVoltage = 3.4
    alarmCounter[numberOfCells]
    for every available cell voltage {
        if(CellVoltages[i] < cutOffMinVoltage ) {				
			alarmCounter[i]++;
		}
		else if(CellVoltages[i] > cutOffMaxVoltage ) {
			alarmCounter[i]++;
		}
		else {
			alarmCounter[i]=0;
		}
    }

    if alarmCounter.any() > 3
        Send Alarm signal
        Create Alarm breaker for 2.50s
    
    count down alarm breaker. 
    Repeat if shutdown not initialised

    
    
