# mbus v1.0

## *Project is still under development, if you have any feedback or notice any errors, please feel free to inform me.*

## Table of contents
* [General info](#general-info)
* [Hardware setup](#hardware-setup)
* [Pheriperial configuration](#pheriperial-configuration)
	- [USART](#usart)
	- [GPIO](#gpio)
	- [Timer](#timer)
* [mbus configuration](#mbus-configuration)
	- [Initialization](#initialization)
	- [Sending data](#sending-data)
* [Usage](#usage)
* [Adding new functions](#adding-new-functions)


## General info
mbus is a versatile serial communication library designed for embedded systems, specifically tailored for STM32 microcontrollers with Cortex-M architecture. Drawing inspiration from Modbus, it introduces a novel feature allowing any device to become a master through arbitration. This unique capability enables flexible and decentralized communication, making it ideal for applications requiring dynamic control over serial communication.

The protocol employs an additional line, in conjunction with the RS485 line, for arbitration. When this line is high, a device seeking master status pulls it low, waits for a specified duration, and then releases it. It subsequently checks the line's state: if it remains high, indicating a successful arbitration, the device becomes the master and pulls the line low again. This prevents other devices connected to the line from becoming masters.

Devices with lower IDs are prioritized, granting them precedence in transmission.

## Hardware setup
Connect the RS485 transceiver to the UART port of the microcontroller along with two transistors - one for pulling the additional line to ground, and the other for reading the state of the line. You can refer to the provided schematic for proper connections. 
> [Important]
> Ensure that the line is pulled up to VBUS with a resistor but is not protected by a fuse.

## Pheriperial configuration
### USART
Frame Configuration:
 - Baud Rate: Recommended maximum speed is 9600 baud
 - Data Bits: 8 bits
 - Parity: None
 - Stop Bits: 1

> [!TIP]
> Don't forget to enable USART interrupts in the NVIC
### GPIO
**Pin INT0_R** is used to read the state of the line, with an interrupt triggered by the **rising edge**. Don't forget to enable interrupts in the NVIC.

**Pin INT0_W** is used to pull the line low and is configured as **GPIO_OUTPUT**.

### Timer

Configure timer to generete period interrupts every 1ms.
For example - TIM17 (16MHz):

- Prescaler 0
- Counter Mode Up
- Counter period 15999 ((f_timer / 1000) - 1)
- RCR 0
- Internal Clock Division 0
- Auto-reload preload Disable

> [!TIP]
> Don't forget to enable interrupts in the NVIC.

> [!WARNING]
> mbus uses SysTick, which is configured to generate interrupts every 1 millisecond.
> If your project has different settings, you need to adjust the function `void mbus_delay_500us(void)` located in the `mbus.c` file.


## mbus configuration
1.  Include file `mbus.h` in `main.c`.
 ```c
#include "mbus.h"
```
2.  Open file `mbus_config.h` and configure transmission parameters.
    In this file you can change timeout values, synchronization time, time between calls and more.
    There is one parameter called `#define MBUS_ONE_BYTE_TX_TIME_MS`, enter here the duration of one UART byte rounded up to milliseconds.
   
4.  Open file `mbus_device_config.h` and configure your device parameters.
    In `#define MBUS_TIMER` you can replace `&htim17` if you use another timer.
    The same with `#define MBUS_UART &huart2`.
    In this file also you can change number of read-write / read-only registers.
   
5.  Add interrupt handling in the `main.c` file:
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim == MBUS_TIMER) mbus_period_int();
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == //CHECK HERE WHICH PIN//) mbus_line0_interrupt();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart == &MBUS_UART) mbus_TX_interrupt();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart == &MBUS_UART) mbus_RX_interrupt();
}
```
> [Tip]
> If you're testing devices, you can use HAL_GPIO_EXTI_Falling_Callback and connect two devices without input transistors, but output transistors are necessery. Code is prepared for that.

6.  Open file `mbus.c` and enter your code in this functions if it's needed:
```c
// Check line 0 status
uint8_t mbus_check_line0(void);
// Hold line 0
void mbus_line0_hold(void);
// Release line 0
void mbus_line0_release(void);
// Enable RS485 TX
void mbus_RS485_TX_enable(void);
// Disable RS485 TX
void mbus_RS485_TX_disable(void);
// Start MBus timer
void mbus_timer_start(void);
// Stop MBus timer
void mbus_timer_stop(void);
// Check MBus timer
uint8_t mbus_timer_check(void);
// Set RX interrupt
void mbus_set_RX_interrupt(void);
// Enable UART reception
void mbus_enable_UART_reception(void);
// Transmit data with interrupt
void mbus_transmit_data_with_interrupt(void);
// Transmit data with interrupt for master call
void mbus_transmit_data_with_interrupt_master_call(void);
// Transmit response data
void mbus_TX_RSP(void);
```

## Usage

### Initialization

First thing you need to initialize mbus with following function:
```c
mbus_init(uint8_t id, uint8_t group, uint8_t wait_to_sync);
```
Where:
- **id** - id of this device
- **group** - id of this device group
- **wait_to_sync** - parameter whether the function should wait for synchrozination (1) or program should continue execution without waiting

If you need, you can use this function to check status of synchronization:
```c
mbus_check_status(void)
```
It returns 1 if device is synchronized.
Timer will automatically turn on and turn off after synchronization. It will also automatically activate when any device becomes the master, and will automatically turn off after the transmission is completed.

**From now on, the device will respond to calls addressed to its own address, group address, and address 0.**

If you don't need group address, you can set it to 0.

Store the data accessible to mbus in the `mbus` structure. These include the following arrays:

- `mbus.Coil_R[]`: 8-bit array, readable only by other devices.
- `mbus.Coil_RW[]`: 8-bit array, readable and writable by other devices.
- `mbus.Reg_R[]`: 16-bit array, readable only by other devices.
- `mbus.Reg_RW[]`: 16-bit array, readable and writable by other devices.


### Sending data

To initiate communication with another device, you need to prepare data for transmission.
All requests are stored in `mbus_TX[MBUS_MASTER_CALLS_COUNT].data[]`. The parameter `MBUS_MASTER_CALLS_COUNT` determines the maximum number of requests the master can transmit in one session.
The order of sending requests/commands is from 0 to "MBUS_MASTER_CALLS_COUNT - 1".

For each request, an expected response is also established:

`mbus_TX[x].rsp = MBUS_RSP_NONE` - no response, only broadcast

`mbus_TX[x].rsp = MBUS_RSP_ACK` - response from the slave by pulling the line to ground (the Master briefly releases the line after sending)

`mbus_TX[x].rsp = MBUS_RSP_DATA` - data from the slave are expected

Once the data is prepared, set the status to `mbus_TX[x].status = MBUS_CALL_BUF_READY;`.


**The following functions are provided for this purpose:**
```c
mbus_master_prepare_Read_Coil_R(uint8_t tab, uint8_t id, uint8_t address, uint8_t count);
mbus_master_prepare_Read_Coil_RW(uint8_t tab, uint8_t id, uint8_t address, uint8_t count);
mbus_master_prepare_Read_Reg_R(uint8_t tab, uint8_t id, uint8_t address, uint8_t count);
mbus_master_prepare_Read_Reg_RW(uint8_t tab, uint8_t id, uint8_t address, uint8_t count);
mbus_master_prepare_Write_Coil_RW(uint8_t tab, uint8_t id, uint8_t address, uint8_t *buf, uint8_t len);
mbus_master_prepare_Write_Reg_RW(uint8_t tab, uint8_t id, uint8_t address, uint16_t *buf, uint8_t len);
mbus_master_prepare_Command_0(uint8_t tab, uint8_t id);
```

Where:

`tab` - specifies the request number (mbus_TX[tab].data[])

`id` - the ID of the device to which the data is addressed

`address` - the address of the first coil/register

`count` - the number of data to read/write

`*buf` - buffer containing the data to be transmitted

`len` - the number of data from the buffer to transmit


**How does the data frame look like?**

| Byte | Master TX | Slave RSP 			|Byte 	| Master TX | Slave RSP 			| Byte 	| Master TX |
| :----: | :--------: | :--------:  		| :----: | :--------: | :--------:  			| :----: | :--------: |
| | **Read_Coil_R** 	|  			| | **Read_Reg_R** |  				| | **Write_Reg_RW** |
| byte 0 | Device ID 	| Slave ID 		| byte 0 | Device ID 		| Slave ID 		| byte 0 | Device ID |
| byte 1 | Slave ID 	| Master ID 		| byte 1 | Slave ID 		| Master ID 		| byte 1 | Slave ID |
| byte 2 | Frame lenght = 8| Frame lenght = 10 	| byte 2 | Frame lenght = 8 	| Frame lenght = 11	| byte 2 | Frame lenght = 11|
| byte 3 | Function code | mbus.Coil_R[2] 	| byte 3 | Function code 	| mbus.Reg_R[10] H 	| byte 3 | Function Code |
| byte 4 | Address = 2 	| mbus.Coil_R[3] 	| byte 4 | Address = 10 	| mbus.Reg_R[10] L 	| byte 4 | Address |
| byte 5 | Count = 5 	| mbus.Coil_R[4] 	| byte 5 | Count = 3 		| mbus.Reg_R[11] H 	| byte 5 | VAL1 H |
| byte 6 | CRC H 	| mbus.Coil_R[5] 	| byte 6 | CRC H		| mbus.Reg_R[11] L 	| byte 6 | VAL1 L |
| byte 7 | CRC L 	| mbus.Coil_R[6] 	| byte 7 | CRC L		| mbus.Reg_R[12] H 	| byte 7 | VAL2 H |
| byte 8 | 		| CRC H 		| byte 8 |			| mbus.Reg_R[12] L 	| byte 8 | VAL2 L |
| byte 9 | 		| CRC L 		| byte 9 |			| CRC H 		| byte 9 | CRC H |
| byte 10 | 		|			|	 |			| CRC L 		| byte 10| CRC L |


#### Example of use

```c
// First command - Read Coil_R from the device with ID=100 from address 0 to 9 inclusive
mbus_master_prepare_Read_Coil_R(0, 100, 0, 10);

// Second command - Read Reg_R from device with ID=120 from address 3 to 5 inclusive
mbus_master_prepare_Read_Reg_R(1, 120, 3, 3);

//Third command - Write data to Reg_RW of the device with ID=34 starting from address 10
uint16_t data[] = {0x2B10, 0xC63F, 0xA300, 0x897C, 0x0008};
mbus_master_prepare_Write_Reg_RW(2, 34, 10, data, 5);
```

**To send data use this function:**
```c
mbus_new_connection();
```
If no other device is currently transmitting, an arbitration process begins to prevent situations where two devices attempt to initiate transmission simultaneously. Upon winning the arbitration, the data from the transmit buffer is sent.
You can check the current transmission status using the function:
```c
// getting status
uint8_t status = mbus_get_connection_status();
// or wait until the transmission is completed.
while(mbus_get_connection_status());
```

The function returns `MBUS_CONNECTION_REQUEST` (0x01) if the data has not been sent yet (i.e., either it is being transmitted or another device is still communicating) or returns `MBUS_CONNECTION_FREE` (0x00) if the data has already been sent and the transmission is complete.

The received data from the slave is stored in `mbus_mRSP[tab].data[]`.

`mbus_mRSP[tab].completed` - Flag indicating that the data has been fully received.

`mbus_mRSP[tab].data_len` - Number of bytes received.

`mbus_TX[tab].status` ` Flag indicating a response from the slave.

List of statuses:

`MBUS_CALL_BUF_FREE` - The data has been sent, and no response was required.

`MBUS_CALL_BUF_RESP` - The response from the slave has been received and is stored in the buffer mbus_mRSP[tab].data[]

`MBUS_CALL_BUF_ACK` - The data has been received and acknowledged by the slave.

`MBUS_CALL_BUF_ERR` - The slave does not respond, and the maximum number of attempts has been exceeded.

## Adding new functions

**1. Add the function name to the `enum` in `mbus.h`**
```c
enum
{
	Read_Coil_R = 0x01,
	Read_Coil_RW = 0x02,
	Read_Reg_R = 0x11,
	Read_Reg_RW = 0x12,
	Write_Coil_RW = 0x22,
	Write_Reg_RW = 0x32,
	Command_0 = 0x50,
	Command_1 = 0x51,
	/*
	 * You can add here your functions.
	 */
	YOUR_FUNCTION_NAME = YOUR_FUNCTION_VALUE
};
```

**2. Prepare a function that executes your feature.**

If you need to response with data:
```c
void exec_YOUR_FUNCTION(void) {
	// If your function needs to respod master prepare data to transmit
	mbus_RSP.data[0] = device_id;		// Set device ID
	mbus_RSP.data[1] = mbus_RX.data[0];	// Set master address
	mbus_RSP.data[2] = //LENGHT// n+3 //

	// Put your data here
	mbus_RSP.data[3] .... mbus_RSP.data[n] = //YOUR DATA//

	// Calculate CRC
	uint16_t CRC_calc = mbus_crc(mbus_RSP.data, mbus_RSP.data[2]-2);
	CRCH = CRC_calc >> 8;
	CRCL = CRC_calc & 0x00FF;

	// Set CRC values in the response packet
	mbus_RSP.data[n+1] = CRCH
	mbus_RSP.data[n+2] = CRCL;

	// Set response ready flag
	rsp_ready_flag = MBUS_RESPONSE_READY;
}
```

If you need to respose with ACK:
```c
void exec_YOUR_FUNCTION(void) {
	// Send ACK to master
	mbus_slave_ack();

	/*
	*	Received data from master stored shortly in mbus_RX.data[]
	*/
}
```

**3. Add a new entry in the switch loop in the function `void mbus_exec_frame(void)` in `mbus.c`**

```c
switch(function_code) {
	case Read_Coil_R:
		if((mbus_RX.data[4] + mbus_RX.data[2] - 7) <= MBUS_COIL_R_COUNT) mbus_exec_Read_Coil_R();
		break;
	case Read_Coil_RW:
		if((mbus_RX.data[4] + mbus_RX.data[2] - 7) <= MBUS_COIL_RW_COUNT) mbus_exec_Read_Coil_RW();
		break;
	case Read_Reg_R:
		if((mbus_RX.data[4] + mbus_RX.data[2] - 7) <= MBUS_REG_R_COUNT) mbus_exec_Read_Reg_R();
		break;
	case Read_Reg_RW:
		if((mbus_RX.data[4] + mbus_RX.data[2] - 7) <= MBUS_REG_RW_COUNT) mbus_exec_Read_Reg_RW();
		break;
	case Write_Coil_RW:
		if((mbus_RX.data[4] + mbus_RX.data[2] - 7) <= MBUS_COIL_RW_COUNT) mbus_exec_Write_Coil_RW();
		break;
	case Write_Reg_RW:
		if((mbus_RX.data[4] + mbus_RX.data[2] - 7) <= MBUS_REG_RW_COUNT) mbus_exec_Write_Reg_RW();
		break;
	/*
	 * You can add here your functions
	 */

	case YOUR_FUNCTION_NAME:
		exec_YOUR_FUNCTION();
		break;

	}
```

In this case code `if((mbus_RX.data[4] + mbus_RX.data[2] - 7) <= MBUS_COIL_R_COUNT)` checks if the requested operation does not exceed the bounds of the data array.

**4. Prepare a function that will prepare the data for transmission.**
```c
void master_prepare_YOUR_FUNCTION_NAME(uint8_t tab, uint8_t id, YOUR VARIABLES) {
	mbus_TX[tab].data[0] = device_id;
	mbus_TX[tab].data[1] = id;
	mbus_TX[tab].data[2] = // LENGHT // n+3 //
	mbus_TX[tab].data[3] = YOUR_FUNCTION_NAME;

	//If necessary, you can insert your variables here
	mbus_TX[tab].data[4] ... mbus_TX[tab].data[n]

	// Calculate CRC
	uint16_t CRC_calc = mbus_crc(mbus_TX[tab].data, mbus_TX[tab].data[2]-2);
	CRCH = CRC_calc >> 8;
	CRCL = CRC_calc & 0x00FF;

	// Set CRC values in the response packet
	mbus_TX[tab].data[n+1] = CRCH
	mbus_TX[tab].data[n+2] = CRCL;

	//Choose the slave's response
	mbus_TX[tab].rsp = MBUS_RSP_DATA;
	//mbus_TX[tab].rsp = MBUS_RSP_ACK;
	//mbus_TX[tab].rsp = MBUS_RSP_NONE;

	//Set the frame status to ready for transmission
	mbus_TX[tab].status = MBUS_CALL_BUF_READY;
}
```
