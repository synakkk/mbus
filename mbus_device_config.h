/*
 * 	mbus_device_config.h
 *
 *  	This file contains general configuration parameters for the MBUS device.
 *
 *  	Version 1.0
 *  	Created on: Apr 19, 2024
 *     Author: Mateusz Synak
 */

#ifndef INC_MBUS_DEVICE_CONFIG_H_
#define INC_MBUS_DEVICE_CONFIG_H_


/**
  * @brief Timer used by the MBUS for timing operations.
  *        This macro allows the user to select a specific timer for MBUS operations.
  *        It is recommended to configure the selected timer appropriately before use.
  */
#define MBUS_TIMER  &htim17  // You can replace htim17 with the desired timer handle

/**
  * @brief UART interface used by the MBUS module for communication.
  *        This macro allows the user to select a specific UART interface for MBUS communication.
  *        It is recommended to configure the selected UART interface appropriately before use.
  */
#define MBUS_UART  &huart2  // You can replace huart2 with the desired UART handle

/**
  * @brief Maximum number of calls that can be made during a single session/connection when the device becomes a master.
  *        This parameter limits the number of queries/commands that can be executed during one session when the device is acting as a master.
  *        Users can adjust this value according to their application requirements.
  */
#define MBUS_MASTER_CALLS_COUNT  4

/**
  * @brief Size of the buffer used to store received data from UART by the slave and transmitted by the master.
  *        This parameter determines the size of the buffer used to store incoming data received via UART by the slave,
  *        and data transmitted by the master during communication with the MBUS protocol.
  *        Users can adjust this value according to their application requirements, considering the expected data size and memory constraints.
  */
#define MBUS_CALL_BUF_SIZE  40

/**
  * @brief Size of the buffer used for data sent by the slave to the master and received by the master.
  *        This parameter determines the size of the buffer used for storing data transmitted by the slave to the master,
  *        and the buffer size used by the master to hold this data during communication with the MBUS protocol.
  *        Users can adjust this value according to their application requirements, considering the expected data size and memory constraints.
  */
#define MBUS_RSP_BUF_SIZE  40


/**
  * @brief Number of 8-bit coils (read-only) supported by the MBUS device.
  *        This parameter defines the size of the array 'Coil_R' in the 'mbus_Registers' structure,
  *        representing the read-only coils used by the device.
  *        Users can adjust this value according to the number of 8-bit read-only coils supported by their device.
  */
#define MBUS_COIL_R_COUNT  20

/**
  * @brief Number of 8-bit coils (read-write) supported by the MBUS device.
  *        This parameter defines the size of the array 'Coil_RW' in the 'mbus_Registers' structure,
  *        representing the read-write coils used by the device.
  *        Users can adjust this value according to the number of 8-bit read-write coils supported by their device.
  */
#define MBUS_COIL_RW_COUNT  20

/**
  * @brief Number of 16-bit registers (read-only) supported by the MBUS device.
  *        This parameter defines the size of the array 'Reg_R' in the 'mbus_Registers' structure,
  *        representing the read-only registers used by the device.
  *        Users can adjust this value according to the number of 16-bit read-only registers supported by their device.
  */
#define MBUS_REG_R_COUNT  20

/**
  * @brief Number of 16-bit registers (read-write) supported by the MBUS device.
  *        This parameter defines the size of the array 'Reg_RW' in the 'mbus_Registers' structure,
  *        representing the read-write registers used by the device.
  *        Users can adjust this value according to the number of 16-bit read-write registers supported by their device.
  */
#define MBUS_REG_RW_COUNT  20



#endif /* INC_MBUS_DEVICE_CONFIG_H_ */
