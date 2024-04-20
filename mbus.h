/*
 * 	mbus.h
 *
 *		Header file for mbus communication
 *
 *		Version 1.0
 *  	Created on: Apr 18, 2024
 *     Author: Mateusz Synak
 */

#ifndef INC_MBUS_H_
#define INC_MBUS_H_

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "mbus_config.h"
#include "mbus_device_config.h"

enum
{
	Read_Coil_R = 0x01,
	Read_Coil_RW = 0x02,
	Read_Reg_R = 0x11,
	Read_Reg_RW = 0x12,
	Write_Coil_RW = 0x22,
	Write_Reg_RW = 0x32,
	Command_0 = 0x50,
	Command_1 = 0x51
	/*
	 * You can add here your functions.
	 */
};

//Line statuses
#define MBUS_STATUS_READY										0		// Line is free
#define MBUS_STATUS_ARBITRAGE								1		// Device is participating in arbitration
#define MBUS_STATUS_BUSY											2		// Line is occupied by another device
#define MBUS_STATUS_MASTER										3		// This device is in master mode

//Master TX processes
#define MBUS_CONNECTION_FREE								0		// No active connection
#define MBUS_CONNECTION_REQUEST						1		// Connection is in progress

#define MBUS_MASTER_PROCESS_ENABLE_TX							0
#define MBUS_MASTER_PROCESS_TX_DATA									(MBUS_RS485_TX_ENABLE_TIME_MS)
#define MBUS_MASTER_PROCESS_TX_WAIT									(MBUS_RS485_TX_ENABLE_TIME_MS + 1)
#define MBUS_MASTER_PROCESS_WAIT_DATA							7
#define MBUS_MASTER_PROCESS_DATA_RSP_TIMEOUT			(1000 + MBUS_SLAVE_DATA_RSP_TIMEOUT_MS)
#define MBUS_MASTER_PROCESS_DATA_DELAY_START			(1001 +MBUS_SLAVE_DATA_RSP_TIMEOUT_MS)
#define MBUS_MASTER_PROCESS_DATA_DELAY_END				(1001 +MBUS_SLAVE_DATA_RSP_TIMEOUT_MS + MBUS_DELAY_BETWEEN_CALLS)
#define MBUS_MASTER_PROCESS_WAIT_RSP								5000
#define MBUS_MASTER_PROCESS_WAIT_RSP_TIMEOUT			(5000 + MBUS_SLAVE_ACK_TIME_MS)
#define MBUS_MASTER_PROCESS_RSP_DELAY_START				(5001 + MBUS_SLAVE_ACK_TIME_MS)
#define MBUS_MASTER_PROCESS_RSP_DELAY_END					(5001 + MBUS_SLAVE_ACK_TIME_MS + MBUS_DELAY_BETWEEN_CALLS)
#define MBUS_MASTER_PROCESS_DELAY_START						9000
#define MBUS_MASTER_PROCESS_DELAY_END							(9000 + MBUS_DELAY_BETWEEN_CALLS)
#define MBUS_MASTER_PROCESS_DELAY_START_R					12000
#define MBUS_MASTER_PROCESS_DELAY_END_R						(12000 + MBUS_DELAY_BETWEEN_CALLS)
#define MBUS_MASTER_PROCESS_DELAY_START_E					15000
#define MBUS_MASTER_PROCESS_DELAY_END_E						(15000 + MBUS_DELAY_BETWEEN_CALLS)

// Calls status
#define MBUS_CALL_BUF_FREE										0			// Call status: ready to receive data
#define MBUS_CALL_BUF_READY									1			// Call status: loaded with data
#define MBUS_CALL_BUF_RESP										2			// Call status: data received
#define MBUS_CALL_BUF_ACK										3			// Call status: ACK received
#define MBUS_CALL_BUF_ERR										4			// Call status: no response from slave
// Response types
#define MBUS_RSP_NONE												0			// No response from slaves
#define MBUS_RSP_ACK													1			// Response from slave holding the line
#define MBUS_RSP_DATA													2			// Response from slave as data


// Device structures
typedef struct {
	uint8_t Coil_R[MBUS_COIL_R_COUNT];				// Read-only coils
	uint8_t Coil_RW[MBUS_COIL_RW_COUNT];		// Read-write coils
	uint16_t Reg_R[MBUS_REG_R_COUNT];			// Read-only registers
	uint16_t Reg_RW[MBUS_REG_RW_COUNT];	// Read-write registers
} mbus_Registers;
extern mbus_Registers mbus;									// Global instance of device registers

// Slave structures
typedef struct {
	uint8_t data[MBUS_CALL_BUF_SIZE];					// Received data buffer
	uint8_t pointer;															// Pointer to the next position in the buffer
	uint8_t data_len;														// Length of the receiving data
} mbus_RX_struct;
extern mbus_RX_struct mbus_RX;							// Global instance of slave receive buffer

typedef struct {
	uint8_t data[MBUS_RSP_BUF_SIZE];					// Response data buffer
} mbus_Response_struct;
extern mbus_Response_struct mbus_RSP;			// Global instance of response buffer

// Master structures
typedef struct {
	uint8_t data[MBUS_CALL_BUF_SIZE];					// Transmit data buffer
	uint8_t status;															// Status of the transmission buffer
	uint8_t rsp;																	// Type of response expected
	uint8_t attempt;														// Counter for resend attempts in case of no ACK
	uint16_t process;														// Current state of the transmission process
} mbus_TX_struct;
extern mbus_TX_struct mbus_TX[MBUS_MASTER_CALLS_COUNT];	// Global array of master transmit buffers

typedef struct {
	uint8_t data[MBUS_RSP_BUF_SIZE];					// Response data buffer
	uint8_t pointer;															// Pointer to the next position in the buffer
	uint8_t data_len;														// Length of the response data
	uint8_t completed;													// Flag indicating whether a complete frame has been received
} mbus_Master_RSP;
extern mbus_Master_RSP mbus_mRSP[MBUS_MASTER_CALLS_COUNT];		// Global array of master response buffers


/**********************************************
 * User-defined functions to be implemented
 **********************************************/
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


/**********************************************
 * Functions for internal use (not editable)
 **********************************************/
void mbus_init(uint8_t id, uint8_t group, uint8_t wait_to_sync);
uint8_t mbus_check_status(void);
void mbus_TX_interrupt(void);
void mbus_RX_interrupt(void);
void mbus_line0_interrupt(void);
void mbus_period_int(void);
void mbus_delay_500us(void);
void mbus_reset_RX_buf(void);
void mbus_master_TX_calls(void);
void mbus_close_connection(void);
uint16_t mbus_crc(uint8_t *buf, uint8_t len);
void mbus_slave_ack(void);
void mbus_new_connection(void);
uint8_t mbus_get_connection_status(void);
void mbus_exec_frame(void);

/**********************************************
 * Functions for execute incoming functions
 **********************************************/
void mbus_exec_Read_Coil_R(void);
void mbus_exec_Read_Coil_RW(void);
void mbus_exec_Read_Reg_R(void);
void mbus_exec_Read_Reg_RW(void);
void mbus_exec_Write_Coil_RW(void);
void mbus_exec_Write_Reg_RW(void);
void mbus_exec_Command_0(void);

/**********************************************
 * Functions for Master Request Preparation
 **********************************************/
void mbus_master_prepare_Read_Coil_R(uint8_t tab, uint8_t id, uint8_t address, uint8_t count);
void mbus_master_prepare_Read_Coil_RW(uint8_t tab, uint8_t id, uint8_t address, uint8_t count);
void mbus_master_prepare_Read_Reg_R(uint8_t tab, uint8_t id, uint8_t address, uint8_t count);
void mbus_master_prepare_Read_Reg_RW(uint8_t tab, uint8_t id, uint8_t address, uint8_t count);
void mbus_master_prepare_Write_Coil_RW(uint8_t tab, uint8_t id, uint8_t address, uint8_t *buf, uint8_t len);
void mbus_master_prepare_Write_Reg_RW(uint8_t tab, uint8_t id, uint8_t address, uint16_t *buf, uint8_t len);
void mbus_master_prepare_Command_0(uint8_t tab, uint8_t id);

#endif /* INC_MBUS_H_ */
