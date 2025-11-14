/*
 * modbus_crc.h
 *
 *  Created on: Nov 11, 2025
 *      Author: DELL
 */

#ifndef INC_MODBUS_CRC_H_
#define INC_MODBUS_CRC_H_

#include "main.h"

/**
 * @brief  Tính toán CRC-16/MODBUS.
 * @param  buffer: Con trỏ đến mảng dữ liệu.
 * @param  length: Độ dài của dữ liệu (không bao gồm 2 byte CRC).
 * @retval Giá trị CRC 16-bit.
 */
uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length);

/**
 * @brief  Xác thực CRC của một gói tin Modbus RTU đã nhận.
 * @param  buffer: Con trỏ đến toàn bộ gói tin nhận được (bao gồm 2 byte CRC ở cuối).
 * @param  length: Tổng độ dài của gói tin (bao gồm 2 byte CRC).
 * @retval 1 nếu CRC hợp lệ, 0 nếu CRC sai.
 */
uint8_t Modbus_Validate_CRC(uint8_t *buffer, uint16_t length);

#endif /* INC_MODBUS_CRC_H_ */
