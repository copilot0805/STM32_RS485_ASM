/*
 * modbus_master.c
 *
 *  Created on: Nov 11, 2025
 *      Author: DELL
 */

#include "modbus_master.h"
#include "modbus_crc.h"

extern UART_HandleTypeDef huart3;

#define RS485_PORT RS485_EN_GPIO_Port
#define RS485_PIN  RS485_EN_Pin

/**
 * @brief  Kích hoạt chế độ GỬI (TX) trên RS485.
 */
void RS485_TX_Enable(void) {
    HAL_GPIO_WritePin(RS485_PORT, RS485_PIN, GPIO_PIN_SET);
}

/**
 * @brief  Kích hoạt chế độ NHẬN (RX) trên RS485.
 */
void RS485_RX_Enable(void) {
    HAL_GPIO_WritePin(RS485_PORT, RS485_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  Gửi yêu cầu Đọc Thanh ghi (0x03) và nhận phản hồi thô.
 */
HAL_StatusTypeDef Modbus_Read_Registers_Raw(UART_HandleTypeDef *huart,
                                            uint8_t slave_id,
                                            uint16_t reg_addr,
                                            uint16_t num_regs,
                                            int16_t *parsed_data_dest,
                                            uint8_t *raw_response_dest)
{
    uint8_t txData[8]; // Gói tin yêu cầu 0x03 luôn là 8 byte

    // Tính số byte phản hồi mong đợi:
    // 1(ID) + 1(FC) + 1(Byte Count) + N*2(Data) + 2(CRC)
    uint8_t rx_buffer_size = 5 + (num_regs * 2);

    // 1. Tạo gói tin Gửi (TX)
    txData[0] = slave_id;
    txData[1] = 0x03; // Function Code
    txData[2] = (reg_addr >> 8) & 0xFF;   // Địa chỉ High
    txData[3] = reg_addr & 0xFF;          // Địa chỉ Low
    txData[4] = (num_regs >> 8) & 0xFF;   // Số lượng High
    txData[5] = num_regs & 0xFF;          // Số lượng Low

    uint16_t crc = Modbus_CRC16(txData, 6); // Tính CRC cho 6 byte đầu
    txData[6] = crc & 0xFF;       // CRC Low
    txData[7] = (crc >> 8) & 0xFF;  // CRC High

    // 2. Gửi gói tin
    RS485_TX_Enable();
    if (HAL_UART_Transmit(huart, txData, 8, 100) != HAL_OK) {
        RS485_RX_Enable(); // Luôn chuyển về RX dù lỗi
        return HAL_ERROR;
    }
    RS485_RX_Enable(); // Chuyển về chế độ Nhận ngay lập tức

    // 3. Nhận phản hồi
    if (HAL_UART_Receive(huart, raw_response_dest, rx_buffer_size, 200) != HAL_OK) {
        return HAL_TIMEOUT; // Lỗi Timeout
    }

    // 4. Xác thực phản hồi
    if (raw_response_dest[0] != slave_id || raw_response_dest[1] != 0x03) {
        return HAL_ERROR; // Không phải phản hồi mong đợi
    }

    if (Modbus_Validate_CRC(raw_response_dest, rx_buffer_size) != 1) {
        return HAL_ERROR; // Lỗi CRC
    }

    // 5. Phân tích (Parse) dữ liệu
    for (int i = 0; i < num_regs; i++) {
        // Dữ liệu bắt đầu từ byte 3 (sau ID, FC, Byte Count)
        // Dữ liệu là Big-Endian (High byte first)
        parsed_data_dest[i] = (raw_response_dest[3 + i*2] << 8) | raw_response_dest[4 + i*2];
    }

    return HAL_OK;
}

/**
 * @brief  Gửi yêu cầu Ghi 1 Thanh ghi (0x06) và nhận phản hồi thô.
 */
HAL_StatusTypeDef Modbus_Write_Register_Raw(UART_HandleTypeDef *huart,
                                            uint8_t slave_id,
                                            uint16_t reg_addr,
                                            int16_t value,
                                            uint8_t *raw_response_dest)
{
    uint8_t txData[8]; // Gói tin 0x06 luôn là 8 byte

    // 1. Tạo gói tin Gửi (TX)
    txData[0] = slave_id;
    txData[1] = 0x06; // Function Code
    txData[2] = (reg_addr >> 8) & 0xFF;   // Địa chỉ High
    txData[3] = reg_addr & 0xFF;          // Địa chỉ Low
    txData[4] = (value >> 8) & 0xFF;      // Giá trị High
    txData[5] = value & 0xFF;             // Giá trị Low

    uint16_t crc = Modbus_CRC16(txData, 6);
    txData[6] = crc & 0xFF;       // CRC Low
    txData[7] = (crc >> 8) & 0xFF;  // CRC High

    // 2. Gửi gói tin
    RS485_TX_Enable();
    if (HAL_UART_Transmit(huart, txData, 8, 100) != HAL_OK) {
        RS485_RX_Enable();
        return HAL_ERROR;
    }
    RS485_RX_Enable();

    // 3. Nhận phản hồi (Phản hồi 0x06 là một bản echo, 8 byte)
    if (HAL_UART_Receive(huart, raw_response_dest, 8, 200) != HAL_OK) {
        return HAL_TIMEOUT;
    }

    // 4. Xác thực phản hồi
    // (Kiểm tra xem nó có echo đúng không)
    for(int i = 0; i < 8; i++) {
        if(txData[i] != raw_response_dest[i]) {
            return HAL_ERROR; // Lỗi Echo
        }
    }

    // Kiểm tra CRC
    if (Modbus_Validate_CRC(raw_response_dest, 8) != 1) {
        return HAL_ERROR; // Lỗi CRC
    }

    return HAL_OK;
}
