/*
 * modbus_master.h
 *
 *  Created on: Nov 11, 2025
 *      Author: DELL
 */

#ifndef INC_MODBUS_MASTER_H_
#define INC_MODBUS_MASTER_H_

#include "main.h"

/**
 * @brief  Kích hoạt chế độ GỬI (TX) trên RS485.
 * (Kéo chân DE/RE lên mức CAO).
 */
void RS485_TX_Enable(void);

/**
 * @brief  Kích hoạt chế độ NHẬN (RX) trên RS485.
 * (Kéo chân DE/RE xuống mức THẤP).
 */
void RS485_RX_Enable(void);

/**
 * @brief  Gửi yêu cầu Đọc Thanh ghi (0x03) và nhận phản hồi thô.
 * @param  huart: Handle của UART (ví dụ: &huart3).
 * @param  slave_id: ID của Slave (1-255).
 * @param  reg_addr: Địa chỉ thanh ghi bắt đầu đọc.
 * @param  num_regs: Số lượng thanh ghi muốn đọc.
 * @param  parsed_data_dest: Con trỏ đến mảng để lưu dữ liệu đã phân tích (16-bit).
 * @param  raw_response_dest: Con trỏ đến mảng để lưu gói tin phản hồi thô (8-bit).
 * @retval HAL_StatusTypeDef (HAL_OK, HAL_ERROR, HAL_TIMEOUT).
 */
HAL_StatusTypeDef Modbus_Read_Registers_Raw(UART_HandleTypeDef *huart,
                                            uint8_t slave_id,
                                            uint16_t reg_addr,
                                            uint16_t num_regs,
                                            int16_t *parsed_data_dest,
                                            uint8_t *raw_response_dest);

/**
 * @brief  Gửi yêu cầu Ghi 1 Thanh ghi (0x06) và nhận phản hồi thô.
 * @param  huart: Handle của UART (ví dụ: &huart3).
 * @param  slave_id: ID của Slave (1-255).
 * @param  reg_addr: Địa chỉ thanh ghi muốn ghi.
 * @param  value: Giá trị 16-bit muốn ghi.
 * @param  raw_response_dest: Con trỏ đến mảng để lưu gói tin phản hồi thô (echo).
 * @retval HAL_StatusTypeDef (HAL_OK, HAL_ERROR, HAL_TIMEOUT).
 */
HAL_StatusTypeDef Modbus_Write_Register_Raw(UART_HandleTypeDef *huart,
                                            uint8_t slave_id,
                                            uint16_t reg_addr,
                                            int16_t value,
                                            uint8_t *raw_response_dest);

#endif /* INC_MODBUS_MASTER_H_ */
