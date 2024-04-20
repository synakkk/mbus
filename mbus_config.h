/*
 * 	mbus_config.h
 *
 *  	This file contains configuration parameters for the main transmission settings.
 *
 *		Version 1.0
 *  	Created on: Apr 19, 2024
 *     Author: Mateusz Synak
 */

#ifndef INC_MBUS_CONFIG_H_
#define INC_MBUS_CONFIG_H_

/**
  * @brief Timeout period in milliseconds.
  *        Recommended timeout is 10ms.
  */
#define MBUS_TIMEOUT_MS		10

/**
  * @brief Time for the slave to acknowledge the master's request in milliseconds.
  *        Recommended ACK time is 5ms.
  */
#define MBUS_SLAVE_ACK_TIME_MS		5

/**
  * @brief Timeout for the slave to respond with requested data to the master in milliseconds.
  *        This parameter determines how long the master waits for the slave to respond with requested data.
  *        Recommended slave response timeout is 20ms.
  */
#define MBUS_SLAVE_DATA_RSP_TIMEOUT_MS		20

/**
  * @brief Time required for the RS485 transceiver to transmit one byte of data in milliseconds.
  *        Even if RS485 is not used, this time should be longer than the time to transmit one byte of data.
  *        Recommended time for transmitting one byte is 2ms.
  */
#define MBUS_ONE_BYTE_TX_TIME_MS			2

/**
  * @brief Time required for the RS485 transceiver to enable the transmitter in milliseconds.
  *        Recommended time for enabling transmitter is 1ms.
  */
#define MBUS_RS485_TX_ENABLE_TIME_MS			1

/**
  * @brief Number of attempts made by the master to send a request to the slave.
  *        If the slave responds after the first attempt, no further attempts are made.
  *        Recommended number of attempts is 2.
  */
#define MBUS_MASTER_TX_ATTEMPTS_COUNT		2

/**
  * @brief Time to block the device that was previously a master and has ended its session.
  *        This parameter blocks the possibility of executing another session to allow other devices to become masters.
  *        Recommended blocking time is 100ms.
  */
#define MBUS_MASTER_BLOCK_TIME_MS			100

/**
  * @brief Time required for device synchronization, i.e., how long the device must see the line in a high state to synchronize.
  *        Recommended synchronization time is longer than the arbitration time.
  *        Recommended synchronization time is 300ms.
  */
#define MBUS_SYNC_TIME_MS			300

/**
  * @brief Arbitration time required for device arbitration.
  *        This parameter must be longer than the highest ID of the device connected to the mbus.
  *        Recommended arbitration time is 257ms.
  */
#define MBUS_ARBITRAGE_TIME_MS		257

/**
  * @brief Delay time between master requests.
  *        This parameter should be longer than the timeout period.
  *        Recommended delay time is 30ms.
  */
#define MBUS_DELAY_BETWEEN_CALLS			30

#endif /* INC_MBUS_CONFIG_H_ */
