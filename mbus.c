/*
 * 	mbus.c
 *
 *  	This file contains the implementation of the MBUS communication protocol.
 *
 *		Version 1.0
 *  	Created on: Apr 18, 2024
 *     Author: Mateusz Synak
 */

#include "mbus.h"

#define MBUS_RESPONSE_READY			(MBUS_RS485_TX_ENABLE_TIME_MS + 1)

volatile uint8_t device_id;
volatile uint8_t device_group;
volatile uint16_t timeout_counter = 0;
volatile uint8_t sync_flag = 0;
uint8_t line0_int_enable = 0;
uint8_t RX_byte;
volatile uint8_t data_sent_flag;
uint8_t ack_counter = 0;
uint8_t master_block_counter = 0;
mbus_Registers mbus;
mbus_RX_struct mbus_RX;
mbus_Response_struct mbus_RSP;
uint8_t rsp_ready_flag = 0;
volatile uint8_t line_status;
mbus_TX_struct mbus_TX[MBUS_MASTER_CALLS_COUNT];
mbus_Master_RSP mbus_mRSP[MBUS_MASTER_CALLS_COUNT];
volatile uint8_t connection_status = 0;
volatile uint8_t acc_tab = 0;
uint8_t status_buf;


/*
 *   	Function to check the state of the arbitration line.
 *
 *   	Returns 1 if the line is in a low state, and 0 if it is in a high state.
 *   	User should insert their custom code here.
 */
uint8_t mbus_check_line0(void) {
	if(HAL_GPIO_ReadPin(INT0_R_GPIO_Port , INT0_R_Pin)) return 0;
	else return 1;
}

/*
 *  	Function to set the arbitration line to a low state.
 *
 *   	User should insert their custom code here to set the line to a low state.
 */
void mbus_line0_hold(void) {
	HAL_GPIO_WritePin(INT0_W_GPIO_Port, INT0_W_Pin, GPIO_PIN_SET);
}

/*
 *  	Function to set the arbitration line to a high state.
 *
 *   	User should insert their custom code here to set the line to a high state.
 */
void mbus_line0_release(void) {
	HAL_GPIO_WritePin(INT0_W_GPIO_Port, INT0_W_Pin, GPIO_PIN_RESET);
}

/*
 *   	Function to enable the RS485 transmitter.
 *
 *   	User should insert their custom code here to enable the RS485 transmitter.
 */
void mbus_RS485_TX_enable(void) {

}

/*
 *   	Function to disable the RS485 transmitter.
 *
 *   	User should insert their custom code here to disable the RS485 transmitter.
 */
void mbus_RS485_TX_disable(void) {

}

/*
 *   	Function to set the timer value to 0 and start the timer.
 *
 *   	User should insert their custom code here to set the timer value to 0 and start the timer.
 *   	They can use the "MBUS_TIMER" macro to specify the timer to be used.
 */
void mbus_timer_start(void) {
	__HAL_TIM_SET_COUNTER(MBUS_TIMER, 0);
	HAL_TIM_Base_Start_IT(MBUS_TIMER);
}

/*
 *   	Function to stop the timer.
 *
 *   	User should insert their custom code here to stop the timer.
 *   	They can use the "MBUS_TIMER" macro to specify the timer to be stopped.
 */
void mbus_timer_stop(void) {
	HAL_TIM_Base_Stop_IT(MBUS_TIMER);
}

/*
 *   	Function to check if the timer is counting.
 *
 *   	User should insert their custom code here to check if the timer specified by "MBUS_TIMER" macro is counting.
 *   	The function should return a value indicating whether the timer is counting or not.
 */
uint8_t mbus_timer_check(void) {
	return HAL_TIM_Base_GetState(MBUS_TIMER);
}

/*
 *   	Function to enable UART reception.
 *
 *   	User should insert their custom code here to enable UART reception for the specified UART interface.
 *   	Newly received data should be stored in the global variable RX_byte.
 */
void mbus_enable_UART_reception(void) {
	HAL_UART_Receive_IT(MBUS_UART, &RX_byte, 1);
}

/*
 *		Enter code here to transmit with interrupt data in "mbus_RSP.data"
 *		Bytes to transmit is stored in "mbus_RSP.data[2]"
 *		and put "data_sent_flag = 0;"
 */
void mbus_transmit_data_with_interrupt(void) {
	HAL_UART_Transmit_IT(MBUS_UART, mbus_RSP.data, mbus_RSP.data[2]);
	data_sent_flag = 0;
}

/*
 *		Enter code here to transmit with interrupt data in "mbus_TX[acc_tab].data"
 *		Bytes to transmit is stored in "mbus_TX[acc_tab].data[2]"
 *		and put "data_sent_flag = 0;"
 */
void mbus_transmit_data_with_interrupt_master_call(void) {
	HAL_UART_Transmit_IT(MBUS_UART, mbus_TX[acc_tab].data, mbus_TX[acc_tab].data[2]);
	data_sent_flag = 0;
}

/************************************
 *  			Non-Editable Functions    		*
 *  		Do not edit functions below 		*
 ************************************/

/*
 *   	Function to transmit response data from the slave to the master.
 *
 *   	This function checks if the response data is ready to be transmitted.
 *   	If the response data is ready (rsp_ready_flag == MBUS_RESPONSE_READY), it enables the RS485 transmitter and decrements the rsp_ready_flag.
 *   	Otherwise, it decrements the rsp_ready_flag and checks if it equals 0 to trigger the transmission of response data using the specified UART interface.
 */
void mbus_TX_RSP(void) {
	if(rsp_ready_flag != 0) {
		if(rsp_ready_flag == MBUS_RESPONSE_READY) {
			mbus_RS485_TX_enable();
			rsp_ready_flag--;
		}
		else {
			rsp_ready_flag--;
			if(rsp_ready_flag == 0) mbus_transmit_data_with_interrupt();
		}
	}
}

/*
 * @brief Function to handle the interrupt indicating that data transmission has been completed.
 *        This function sets the data_sent_flag to the specified transmission time.
 */
void mbus_TX_interrupt(void) {
	data_sent_flag = MBUS_ONE_BYTE_TX_TIME_MS;
}

/*
 * @brief Function to handle the interrupt indicating that data reception has occurred.
 *        This function processes received data based on the device status (slave or master).
 *        For slave devices, it stores the received byte and decodes the frame when all bytes are collected.
 *        For master devices, it stores the received byte and sets the completion flag when all bytes are received.
 */
void mbus_RX_interrupt(void) {
	timeout_counter = MBUS_TIMEOUT_MS;

	//For Slave
	if(line_status == MBUS_STATUS_BUSY) {
		mbus_RX.data[mbus_RX.pointer] = RX_byte;		// Store received byte
		if(mbus_RX.pointer == 2) {
			mbus_RX.data_len = RX_byte;		// Set frame length
		}
		mbus_RX.pointer++;			// Increment pointer
		if(mbus_RX.pointer == MBUS_CALL_BUF_SIZE) {
			mbus_RX.pointer--;			// Decrement pointer when if it exceeds the array bounds
		}
		else if(mbus_RX.pointer == mbus_RX.data_len) {		// All bytes collected
			// Decode data and reset RX buffer
			mbus_exec_frame();
			mbus_reset_RX_buf();
		}
	}
	// For Master
	else if (line_status == MBUS_STATUS_MASTER) {
		mbus_mRSP[acc_tab].data[mbus_mRSP[acc_tab].pointer] = RX_byte;
		if(mbus_mRSP[acc_tab].pointer == 2) {
			mbus_mRSP[acc_tab].data_len = RX_byte;
		}
		mbus_mRSP[acc_tab].pointer++;
		if(mbus_mRSP[acc_tab].pointer == MBUS_RSP_BUF_SIZE) {
			mbus_mRSP[acc_tab].pointer--;
		}
		if(mbus_mRSP[acc_tab].pointer == mbus_mRSP[acc_tab].data_len) {
			mbus_mRSP[acc_tab].completed = 1;
		}
	}

	mbus_enable_UART_reception();		//Watch for next RX byte
}

/*
 * @brief Initializes the MBUS module.
 * @param id: The ID of the device.
 * @param group: The Group of the device.
 * @param wait_to_sync: Set to 1 to wait for synchronization.
 *        If set to 0, synchronization will not be waited for.
 */
void mbus_init(uint8_t id, uint8_t group, uint8_t wait_to_sync) {
	device_id = id;
	device_group = group;
	timeout_counter = 0;
	mbus_timer_start();
	if(wait_to_sync) {
		while(!sync_flag);		// Wait for synchronization
	}
}

/*
 * @brief Checks the status of synchronization.
 * @return 1 if synchronized, otherwise 0.
 */
uint8_t mbus_check_status(void) {
	return sync_flag;
}

/*
 * @brief Handles the interrupt triggered by line 0 (GPIO falling edge).
 *        Starts a new incoming connection and sets the appropriate flags.
 */
void mbus_line0_interrupt(void) {
	if(line0_int_enable) {
		//New incoming connection
		mbus_timer_start();
		line0_int_enable = 0;
		line_status = MBUS_STATUS_BUSY;
		timeout_counter = MBUS_ARBITRAGE_TIME_MS + MBUS_TIMEOUT_MS;
		mbus_enable_UART_reception();
	}
}

/*
 * @brief Handles periodic interrupt from the timer.
 *        Manages various tasks such as data transmission, arbitration, and synchronization.
 */
void mbus_period_int(void) {
	if(sync_flag) {
		// After synchronization

		// Auto-disable RS485 after data transmission
		if(data_sent_flag != 0) {
			data_sent_flag--;
			if(data_sent_flag == 0) mbus_RS485_TX_disable();
		}

		// Transmit response data if ready
		mbus_TX_RSP();

		// Slave - status BUSY
		if(line_status == MBUS_STATUS_BUSY) {
			// ACK handling
			if(ack_counter != 0) {
				if(ack_counter == (MBUS_SLAVE_ACK_TIME_MS + 1)) {
					mbus_line0_hold();
					ack_counter--;
				}
				if(ack_counter == 0) mbus_line0_release();
			}
			// Timeout handling
			if(timeout_counter == 0) {
				if(mbus_check_line0() == 0) {
					line_status = MBUS_STATUS_READY;
					mbus_reset_RX_buf();
					line0_int_enable = 1;
				}
			}
			else timeout_counter--;
		}

		// Master
		else if(line_status == MBUS_STATUS_MASTER) {
			mbus_master_TX_calls();
		}
		// Arbitration
		else if(line_status == MBUS_STATUS_ARBITRAGE) {
			timeout_counter--;
			if(timeout_counter == 0) {
				// Check arbitration result
				mbus_line0_release();
				// Wait 500us
				mbus_delay_500us();
				if(mbus_check_line0()) {
					// Lose
					line_status = MBUS_STATUS_BUSY;
					timeout_counter = device_id + MBUS_TIMEOUT_MS;
					mbus_enable_UART_reception();
				}
				else {
					// Win
					mbus_line0_hold();
					line_status = MBUS_STATUS_MASTER;
					mbus_enable_UART_reception();
				}
			}
		}
		else if(line_status == MBUS_STATUS_READY) {
			if((connection_status == MBUS_CONNECTION_REQUEST) && (master_block_counter == 0)) {
				if(!mbus_check_line0()) {
					mbus_line0_hold();
					line0_int_enable = 0;
					line_status = MBUS_STATUS_ARBITRAGE;
					timeout_counter = MBUS_ARBITRAGE_TIME_MS - device_id;
				}
			}
			// Turning off timer
			else if(master_block_counter == 0) {
				if(!mbus_check_line0()) mbus_timer_stop();
			}
			else master_block_counter--;
		}
	}
	else {
		// Before synchronization
		if(mbus_check_line0() == 0) {
			timeout_counter++;
			if(timeout_counter == MBUS_SYNC_TIME_MS) {
				sync_flag = 1;
				line0_int_enable = 1;
				if(mbus_check_line0() == 0) mbus_timer_stop();
			}
		}
		else {
			timeout_counter = 0;
		}
	}
}

/*
 * Delays the execution by approximately 500 microseconds.
 * Uses the SysTick timer for timing.
 */
void mbus_delay_500us(void) {
	uint32_t exit_time = SysTick->VAL;
	if(exit_time > (SysTick->LOAD / 2)) {
		exit_time -= (SysTick->LOAD / 2);
	}
	else {
		exit_time = (SysTick->LOAD / 2) + exit_time;
	}

	while(SysTick->VAL < exit_time);
	while(SysTick->VAL > exit_time);
}

/*
 * Resets the receive buffer used by the MBUS protocol.
 * Sets the pointer to the beginning of the buffer and resets the data length.
 */
void mbus_reset_RX_buf(void) {
	mbus_RX.pointer = 0;		// Reset the pointer to the beginning of the buffer
	mbus_RX.data_len = 0xFF;		// Reset the data length to an initial value (0xFF in this case)
}

/*
 *  Master function for transmitting calls.
 *  This function handles the transmission of master calls to the slave devices.
 *  It processes the call buffer and manages the transmission process.
 */
void mbus_master_TX_calls(void) {
	for(uint8_t x = 0; x<(MBUS_MASTER_CALLS_COUNT + 1); x++) {
		if(x == MBUS_MASTER_CALLS_COUNT) {
			mbus_close_connection();
			break;
		}
		if(mbus_TX[x].status == MBUS_CALL_BUF_READY) {
			if(mbus_TX[x].attempt < MBUS_MASTER_TX_ATTEMPTS_COUNT + 1) {
				acc_tab = x;

				// Enable RS485 Transmitter
				if(mbus_TX[x].process == MBUS_MASTER_PROCESS_ENABLE_TX) {
					mbus_RS485_TX_enable();
					mbus_TX[x].process++;
				}

				// Transmit data
				else if(mbus_TX[x].process == MBUS_MASTER_PROCESS_TX_DATA) {
					mbus_transmit_data_with_interrupt_master_call();
					mbus_TX[x].process = MBUS_MASTER_PROCESS_TX_WAIT;
				}

				// Wait for data transmitted - Transmitter will turn off automatically later
				else if(mbus_TX[x].process == MBUS_MASTER_PROCESS_TX_WAIT) {
					if(data_sent_flag) {
						mbus_reset_RX_buf();

						if(mbus_TX[x].rsp == MBUS_RSP_DATA) {
							// Wait for data with timeout
							mbus_TX[x].process = MBUS_MASTER_PROCESS_WAIT_DATA;
						}
						else if(mbus_TX[x].rsp == MBUS_RSP_ACK) {
							// Check ACK with timeout
							mbus_line0_release();
							mbus_TX[x].process = MBUS_MASTER_PROCESS_WAIT_RSP;
						}
						else if(mbus_TX[x].rsp == MBUS_RSP_NONE) {
							// Only wait time between calls
							mbus_TX[x].process = MBUS_MASTER_PROCESS_DELAY_START;
						}
					}
				}

				// Waiting for data and timeout
				else if( (mbus_TX[x].process >= MBUS_MASTER_PROCESS_WAIT_DATA) && (mbus_TX[x].process <= MBUS_MASTER_PROCESS_DATA_RSP_TIMEOUT) ) {
					if(mbus_mRSP[x].completed) {
						// Response received
						mbus_TX[x].process = MBUS_MASTER_PROCESS_DATA_DELAY_START;
					}
					else if(mbus_TX[x].process == MBUS_MASTER_PROCESS_DATA_RSP_TIMEOUT) {
						// New attempt
						if(mbus_TX[x].attempt < MBUS_MASTER_TX_ATTEMPTS_COUNT - 1) {
							mbus_mRSP[x].pointer = 0;
							mbus_TX[x].attempt++;
							mbus_TX[x].process = MBUS_MASTER_PROCESS_DELAY_START_R;
						}
						else {
							mbus_TX[x].process = MBUS_MASTER_PROCESS_DELAY_START_E;
						}
					}
					else mbus_TX[x].process++;
				}

				// Wait between frames - received data from slave
				else if( (mbus_TX[x].process >= MBUS_MASTER_PROCESS_DATA_DELAY_START) && (mbus_TX[x].process <= MBUS_MASTER_PROCESS_DATA_DELAY_END) ) {
					if(mbus_TX[x].process == MBUS_MASTER_PROCESS_DATA_DELAY_END) {
						mbus_TX[x].status = MBUS_CALL_BUF_RESP;
					}
					else mbus_TX[x].process++;
				}

				// Wait for ACK and timeout
				else if( (mbus_TX[x].process >= MBUS_MASTER_PROCESS_WAIT_RSP) && (mbus_TX[x].process <= MBUS_MASTER_PROCESS_WAIT_RSP_TIMEOUT) ) {
					if(mbus_check_line0()) {
						// ACK received
						mbus_TX[x].process = MBUS_MASTER_PROCESS_RSP_DELAY_START;
						mbus_line0_hold();
					}
					else {
						if(mbus_TX[x].process == MBUS_MASTER_PROCESS_WAIT_RSP_TIMEOUT) {
							// New attempt
							mbus_line0_hold();
							if(mbus_TX[x].attempt < MBUS_MASTER_TX_ATTEMPTS_COUNT - 1) {
								mbus_TX[x].attempt++;
								mbus_TX[x].process = MBUS_MASTER_PROCESS_DELAY_START_R;
							}
							else {
								mbus_TX[x].process = MBUS_MASTER_PROCESS_DELAY_START_E;
								mbus_line0_hold();
							}
						}
						else mbus_TX[x].process++;
					}
				}

				// Wait between frames after receiving ACK from slave
				else if( (mbus_TX[x].process >= MBUS_MASTER_PROCESS_RSP_DELAY_START) && (mbus_TX[x].process <= MBUS_MASTER_PROCESS_RSP_DELAY_END) ) {
					if(mbus_TX[x].process == MBUS_MASTER_PROCESS_RSP_DELAY_END) {
						mbus_TX[x].status = MBUS_CALL_BUF_ACK;
					}
					else mbus_TX[x].process++;
				}

				// Wait between frames when no ACK needed
				else if( (mbus_TX[x].process >= MBUS_MASTER_PROCESS_DELAY_START) && (mbus_TX[x].process <= MBUS_MASTER_PROCESS_DELAY_END) ) {
					if(mbus_TX[x].process == MBUS_MASTER_PROCESS_DELAY_END) {
						mbus_TX[x].status = MBUS_CALL_BUF_FREE;
					}
					else mbus_TX[x].process++;
				}

				// Wait between frames and reset process
				else if((mbus_TX[x].process >= MBUS_MASTER_PROCESS_DELAY_START_R) && (mbus_TX[x].process <= MBUS_MASTER_PROCESS_DELAY_END_R)) {
					if(mbus_TX[x].process == MBUS_MASTER_PROCESS_DELAY_END_R) {
						mbus_TX[x].process = MBUS_MASTER_PROCESS_ENABLE_TX;
					}
					else mbus_TX[x].process++;
				}

				// Wait between frames and reset process
				else if((mbus_TX[x].process >= MBUS_MASTER_PROCESS_DELAY_START_E) && (mbus_TX[x].process <= MBUS_MASTER_PROCESS_DELAY_END_E)) {
					if(mbus_TX[x].process == MBUS_MASTER_PROCESS_DELAY_END_E) {
						mbus_TX[x].status = MBUS_CALL_BUF_ERR;
					}
					else mbus_TX[x].process++;
				}

				else mbus_TX[x].process++;
			}
			break;
		}
	}
}

/*
 *  Function to close the connection.
 *  This function releases the line and prepares the system for a new connection.
 */
void mbus_close_connection(void) {
	mbus_line0_release();
	line0_int_enable = 1;
	connection_status = MBUS_CONNECTION_FREE;
	line_status = MBUS_STATUS_READY;
	master_block_counter = MBUS_MASTER_BLOCK_TIME_MS;
}

/*
 *  Function to calculate CRC (Cyclic Redundancy Check) for the given buffer.
 *  This function takes a buffer and its length as input and returns the calculated CRC.
 */
uint16_t mbus_crc(uint8_t *buf, uint8_t len) {

	// CRC table
	static const uint16_t table[2] = { 0x0000, 0xA001 };
	uint16_t crc = 0xFFFF;
	// Loop variables
	unsigned int i = 0;
	char bit = 0;
	unsigned int xor = 0;

	// Iterate through each byte in the buffer
	for( i = 0; i < len; i++ )
	{
		// XOR the current byte with the CRC
		crc ^= buf[i];

		// Process each bit in the byte
		for( bit = 0; bit < 8; bit++ )
		{
			// Check the least significant bit
			xor = crc & 0x01;
			// Shift right by one bit
			crc >>= 1;
			// XOR with the appropriate value from the CRC table
			crc ^= table[xor];
		}
	}

	// Return the calculated CRC
	return crc;
}

/*
 *  Function to send acknowledgment (ACK) from the slave device.
 *  This function sets the acknowledgment counter based on the defined time duration.
 */
void mbus_slave_ack(void) {
	ack_counter = MBUS_SLAVE_ACK_TIME_MS + 1;
}

/*
 *  Function to initialize a new connection and reset transmission parameters.
 */
void mbus_new_connection(void) {
	// Reset transmission parameters
	for(uint8_t x = 0; x<MBUS_MASTER_CALLS_COUNT; x++) {
		mbus_TX[x].attempt = 0;
		mbus_TX[x].process = MBUS_MASTER_PROCESS_ENABLE_TX;
		mbus_mRSP[x].completed = 0;
		mbus_mRSP[x].data_len = 0xFF;
		mbus_mRSP[x].pointer = 0;
	}

	// Set connection status to request
	connection_status = MBUS_CONNECTION_REQUEST;

	// Start timer if is not running
	if(mbus_timer_check() == 1 ) {
		mbus_timer_start();
	}
}

/*
 * Get the current connection status.
 * Returns:
 *   - MBUS_CONNECTION_FREE if the connection is free.
 *   - MBUS_CONNECTION_REQUEST if there is a connection request in progress.
 */
uint8_t mbus_get_connection_status(void) {
	return connection_status;
}

/*
 *  Function to execute the received MBUS frame.
 *  This function verifies the CRC, extracts frame details, and performs corresponding actions.
 */
void mbus_exec_frame(void) {
	/*
	 *  Reading coil/reg:								Updating coil:									Updating reg:
	 *  data[0] - Master address				data[0] - Master address			data[0] - Master address
	 *  data[1] - Slave address					data[1] - Slave address				data[1] - Slave address
	 *  data[2] - Frame lenght					data[2] - Frame lenght				data[2] - Frame lenght
	 *  data[3] - Function								data[3] - Function							data[3] - Function
	 *  data[4] - Start address					data[4] - Start address				data[4] - Start address
	 *  data[5] - Address count					data[5] - Val 1									data[5] - Val 1H
	 *  data[6] - CRC H									data[6] - Val 2									data[6] - Val 1L
	 *  data[7] - CRC L									data[7] - Val3									data[7] - Val 2H
	 *  																data[8] - CRC H								data[8] - Val 2L
	 *  																data[9] - CRC L								data[9] - CRC H
	 *  																															data[10] - CRC L
	 */
	if((mbus_RX.data[1] == 0) || (mbus_RX.data[1] == device_id) || (mbus_RX.data[1] == device_group) ) {
		// Calculate CRC
		uint16_t crc_calc = mbus_crc(mbus_RX.data, mbus_RX.pointer - 2);
		uint16_t crc_received = (mbus_RX.data[mbus_RX.pointer - 2] << 8) + mbus_RX.data[mbus_RX.pointer - 1];

		// Verify CRC
		if(crc_calc == crc_received) {

			// Extract function code
			uint8_t function_code = mbus_RX.data[3];

			// Perform action based on function code

			/*
			 * 		if((mbus_RX.data[4] + mbus_RX.data[2] - 7) <= MBUS_COIL_R_COUNT)
			 *
			 * 		This code checks if the requested operation does not exceed the bounds of  the data array
			 */
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
			}
		}
	}
}

/*******************************
 * Prepare response packets for various MBUS operations.
 ******************************/

/*
 *  Function to execute Read_Coil_R function.
 */
void mbus_exec_Read_Coil_R(void) {
	mbus_RSP.data[0] = device_id;		// Set device ID
	mbus_RSP.data[1] = mbus_RX.data[0];		// Set master address
	mbus_RSP.data[2] = mbus_RX.data[5] + 5;		// Set frame length

	// Copy coil data to response packet
	for(uint8_t x = 0; x<mbus_RX.data[5]; x++) {
		mbus_RSP.data[3+x] = mbus.Coil_R[x + mbus_RX.data[4]];
	}

	// Calculate CRC
	uint16_t CRC_calc = mbus_crc(mbus_RSP.data, mbus_RSP.data[2]-2);
	uint16_t CRCH = CRC_calc >> 8;
	uint16_t CRCL = CRC_calc & 0x00FF;

	// Set CRC values in the response packet
	mbus_RSP.data[mbus_RX.data[5]+3] = CRCH;
	mbus_RSP.data[mbus_RX.data[5]+4] = CRCL;

	// Set response ready flag
	rsp_ready_flag = MBUS_RESPONSE_READY;
}

/*
 *  Function to execute Read_Coil_RW function.
 */
void mbus_exec_Read_Coil_RW(void) {
	mbus_RSP.data[0] = device_id;		// Set device ID
	mbus_RSP.data[1] = mbus_RX.data[0];		// Set master address
	mbus_RSP.data[2] = mbus_RX.data[5] + 5;		// Set frame length

	// Copy coil data to response packet
	for(uint8_t x = 0; x<mbus_RX.data[5]; x++) {
		mbus_RSP.data[3+x] = mbus.Coil_RW[x + mbus_RX.data[4]];
	}

	// Calculate CRC
	uint16_t CRC_calc = mbus_crc(mbus_RSP.data, mbus_RSP.data[2]-2);
	uint16_t CRCH = CRC_calc >> 8;
	uint16_t CRCL = CRC_calc & 0x00FF;

	// Set CRC values in the response packet
	mbus_RSP.data[mbus_RX.data[5]+3] = CRCH;
	mbus_RSP.data[mbus_RX.data[5]+4] = CRCL;

	// Set response ready flag
	rsp_ready_flag = MBUS_RESPONSE_READY;
}

/*
 *  Function to execute Read_Reg_R function.
 */
void mbus_exec_Read_Reg_R(void) {
	mbus_RSP.data[0] = device_id;		// Set device ID
	mbus_RSP.data[1] = mbus_RX.data[0];		// Set master address
	mbus_RSP.data[2] = 5 + (mbus_RX.data[5] * 2);		// Set frame length

	// Copy coil data to response packet
	for(uint8_t x = 0; x<mbus_RX.data[5]; x++) {
		mbus_RSP.data[3+(2*x)] = mbus.Reg_R[mbus_RX.data[4] + x] >> 8;
		mbus_RSP.data[3+(2*x)+1] = mbus.Reg_R[mbus_RX.data[4] + x];
	}

	// Calculate CRC
	uint16_t CRC_calc = mbus_crc(mbus_RSP.data, mbus_RSP.data[2]-2);
	uint16_t CRCH = CRC_calc >> 8;
	uint16_t CRCL = CRC_calc & 0x00FF;

	// Set CRC values in the response packet
	mbus_RSP.data[(mbus_RX.data[5]*2)+3] = CRCH;
	mbus_RSP.data[(mbus_RX.data[5]*2)+4] = CRCL;

	// Set response ready flag
	rsp_ready_flag = MBUS_RESPONSE_READY;
}

/*
 *  Function to execute Read_Reg_RW function.
 */
void mbus_exec_Read_Reg_RW(void) {
	mbus_RSP.data[0] = device_id;		// Set device ID
	mbus_RSP.data[1] = mbus_RX.data[0];		// Set master address
	mbus_RSP.data[2] = 5 + (mbus_RX.data[5] * 2);		// Set frame length

	// Copy coil data to response packet
	for(uint8_t x = 0; x<mbus_RX.data[5]; x++) {
		mbus_RSP.data[3+(2*x)] = mbus.Reg_RW[mbus_RX.data[4] + x] >> 8;
		mbus_RSP.data[3+(2*x)+1] = mbus.Reg_RW[mbus_RX.data[4] + x];
	}

	// Calculate CRC
	uint16_t CRC_calc = mbus_crc(mbus_RSP.data, mbus_RSP.data[2]-2);
	uint16_t CRCH = CRC_calc >> 8;
	uint16_t CRCL = CRC_calc & 0x00FF;

	// Set CRC values in the response packet
	mbus_RSP.data[(mbus_RX.data[5]*2)+3] = CRCH;
	mbus_RSP.data[(mbus_RX.data[5]*2)+4] = CRCL;

	// Set response ready flag
	rsp_ready_flag = MBUS_RESPONSE_READY;
}

/*
 *  Function to execute Write_Coil_RW function and update coil values accordingly.
 */
void mbus_exec_Write_Coil_RW(void) {
	// Send ACK to master
	mbus_slave_ack();

	// Update coil values
	for(uint8_t x = 0; x<(mbus_RX.data[2]-7); x++) {
		mbus.Coil_RW[x + mbus_RX.data[4]] = mbus_RX.data[x+5];
	}

}

/*
 *  Function to execute Write_Reg_RW function and update register values accordingly.
 */
void mbus_exec_Write_Reg_RW(void) {
	// Send ACK to master
	mbus_slave_ack();

	// Update register values
	for(uint8_t x = 0; x<((mbus_RX.data[2]-7) / 2); x++) {
		mbus.Coil_RW[x + mbus_RX.data[4]] = (mbus_RX.data[(x*2)+5] << 8) + mbus_RX.data[(x*2)+6];
	}
}

/*
 * 	Function to execute Command_0 Function
 */
void mbus_exec_Command_0(void) {

}



/*************************************************************
 * Prepare a packet for a Read Coil operation.
 *
 * Parameters:
 *   tab: Index of the transmission buffer.
 *   id: ID of the slave device.
 *   address: Start address of the coils to read.
 *   count: Number of coils to read.
 *************************************************************/
void mbus_master_prepare_Read_Coil_R(uint8_t tab, uint8_t id, uint8_t address, uint8_t count) {
	mbus_TX[tab].data[0] = device_id;
	mbus_TX[tab].data[1] = id;
	mbus_TX[tab].data[2] = 8;
	mbus_TX[tab].data[3] = Read_Coil_R;
	mbus_TX[tab].data[4] = address;
	mbus_TX[tab].data[5] = count;

	uint16_t crc_calc = mbus_crc(mbus_TX[tab].data, 6);

	mbus_TX[tab].data[6] = (crc_calc >> 8);
	mbus_TX[tab].data[7] = crc_calc;

	mbus_TX[tab].rsp = MBUS_RSP_DATA;
	mbus_TX[tab].status = MBUS_CALL_BUF_READY;
}

/*************************************************************
 * Prepare a packet for Read Coil operation with read-write permission.
 *
 * Parameters:
 *   tab: Index of the transmission buffer.
 *   id: ID of the slave device.
 *   address: Start address of the coils to read.
 *   count: Number of coils to read.
 *************************************************************/
void mbus_master_prepare_Read_Coil_RW(uint8_t tab, uint8_t id, uint8_t address, uint8_t count) {
	mbus_TX[tab].data[0] = device_id;
	mbus_TX[tab].data[1] = id;
	mbus_TX[tab].data[2] = 8;
	mbus_TX[tab].data[3] = Read_Coil_RW;
	mbus_TX[tab].data[4] = address;
	mbus_TX[tab].data[5] = count;

	uint16_t crc_calc = mbus_crc(mbus_TX[tab].data, 6);

	mbus_TX[tab].data[6] = (crc_calc >> 8);
	mbus_TX[tab].data[7] = crc_calc;

	mbus_TX[tab].rsp = MBUS_RSP_DATA;
	mbus_TX[tab].status = MBUS_CALL_BUF_READY;
}

/*************************************************************
 * Prepare a packet for Read Register operation.
 *
 * Parameters:
 *   tab: Index of the transmission buffer.
 *   id: ID of the slave device.
 *   address: Start address of the registers to read.
 *   count: Number of registers to read.
 *************************************************************/
void mbus_master_prepare_Read_Reg_R(uint8_t tab, uint8_t id, uint8_t address, uint8_t count) {
	mbus_TX[tab].data[0] = device_id;
	mbus_TX[tab].data[1] = id;
	mbus_TX[tab].data[2] = 8;
	mbus_TX[tab].data[3] = Read_Reg_R;
	mbus_TX[tab].data[4] = address;
	mbus_TX[tab].data[5] = count;

	uint16_t crc_calc = mbus_crc(mbus_TX[tab].data, 6);

	mbus_TX[tab].data[6] = (crc_calc >> 8);
	mbus_TX[tab].data[7] = crc_calc;

	mbus_TX[tab].rsp = MBUS_RSP_DATA;
	mbus_TX[tab].status = MBUS_CALL_BUF_READY;
}

/*************************************************************
 * Prepare a packet for Read-Write Register operation.
 *
 * Parameters:
 *   tab: Index of the transmission buffer.
 *   id: ID of the slave device.
 *   address: Start address of the registers to read/write.
 *   count: Number of registers to read/write.
 *************************************************************/
void mbus_master_prepare_Read_Reg_RW(uint8_t tab, uint8_t id, uint8_t address, uint8_t count) {
	mbus_TX[tab].data[0] = device_id;
	mbus_TX[tab].data[1] = id;
	mbus_TX[tab].data[2] = 8;
	mbus_TX[tab].data[3] = Read_Reg_RW;
	mbus_TX[tab].data[4] = address;
	mbus_TX[tab].data[5] = count;

	uint16_t crc_calc = mbus_crc(mbus_TX[tab].data, 6);

	mbus_TX[tab].data[6] = (crc_calc >> 8);
	mbus_TX[tab].data[7] = crc_calc;

	mbus_TX[tab].rsp = MBUS_RSP_DATA;
	mbus_TX[tab].status = MBUS_CALL_BUF_READY;
}

/*************************************************************
 * Prepare a packet for Write Coil operation.
 *
 * Parameters:
 *   tab: Index of the transmission buffer.
 *   id: ID of the slave device.
 *   address: Start address of the coils to write.
 *   buf: Pointer to the buffer containing data to be written.
 *   len: Length of the data buffer.
 *************************************************************/
void mbus_master_prepare_Write_Coil_RW(uint8_t tab, uint8_t id, uint8_t address, uint8_t *buf, uint8_t len) {
	mbus_TX[tab].data[0] = device_id;
	mbus_TX[tab].data[1] = id;
	mbus_TX[tab].data[2] = 7+len;
	mbus_TX[tab].data[3] = Write_Coil_RW;
	mbus_TX[tab].data[4] = address;

	for(uint8_t x = 0; x<len; x++) {
		mbus_TX[tab].data[5+x] = buf[x];
	}

	uint16_t crc_calc = mbus_crc(mbus_TX[tab].data, 5+len);
	mbus_TX[tab].data[5+len] = (crc_calc >> 8);
	mbus_TX[tab].data[6+len] = crc_calc;

	mbus_TX[tab].rsp = MBUS_RSP_ACK;
	mbus_TX[tab].status = MBUS_CALL_BUF_READY;
}

/*************************************************************
 * Prepare a packet for Write Register operation.
 *
 * Parameters:
 *   tab: Index of the transmission buffer.
 *   id: ID of the slave device.
 *   address: Start address of the registers to write.
 *   buf: Pointer to the buffer containing data to be written.
 *   len: Length of the data buffer.
 *************************************************************/
void mbus_master_prepare_Write_Reg_RW(uint8_t tab, uint8_t id, uint8_t address, uint16_t *buf, uint8_t len) {
	mbus_TX[tab].data[0] = device_id;
	mbus_TX[tab].data[1] = id;
	mbus_TX[tab].data[2] = 7+(len*2);
	mbus_TX[tab].data[3] = Write_Reg_RW;
	mbus_TX[tab].data[4] = address;

	for(uint8_t x = 0; x<len; x++) {
		mbus_TX[tab].data[5+(2*x)] = (buf[x] >> 8);
		mbus_TX[tab].data[6+(2*x)] = buf[x];
	}

	uint16_t crc_calc = mbus_crc(mbus_TX[tab].data, (5+(2*len)));
	mbus_TX[tab].data[5+(2*len)] = (crc_calc >> 8);
	mbus_TX[tab].data[6+(2*len)] = crc_calc;

	mbus_TX[tab].rsp = MBUS_RSP_ACK;
	mbus_TX[tab].status = MBUS_CALL_BUF_READY;
}

/*************************************************************
 * Prepare a packet for Command_0 operation.
 *
 * Parameters:
 *   tab: Index of the transmission buffer.
 *   id: ID of the slave device / group..
 *************************************************************/
void mbus_master_prepare_Command_0(uint8_t tab, uint8_t id) {
	mbus_TX[tab].data[0] = device_id;
	mbus_TX[tab].data[1] = id;
	mbus_TX[tab].data[2] = 6;
	mbus_TX[tab].data[3] = Command_0;

	uint16_t crc_calc = mbus_crc(mbus_TX[tab].data, 4);
	mbus_TX[tab].data[4] = (crc_calc >> 8);
	mbus_TX[tab].data[5] = crc_calc;

	mbus_TX[tab].rsp = MBUS_RSP_NONE;
	mbus_TX[tab].status = MBUS_CALL_BUF_READY;
}
