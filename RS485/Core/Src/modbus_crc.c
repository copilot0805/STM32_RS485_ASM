/*
 * modbus_crc.c
 *
 *  Created on: Nov 11, 2025
 *      Author: DELL
 */

#include "modbus_crc.h"

/**
 * @brief  Tính toán CRC-16/MODBUS.
 * @param  buffer: Con trỏ đến mảng dữ liệu.
 * @param  length: Độ dài của dữ liệu (KHÔNG bao gồm 2 byte CRC).
 * @retval Giá trị CRC 16-bit.
 */
uint16_t Modbus_CRC16(uint8_t *buffer, uint16_t length) { //CRC-16/MODBUS bitwise (LSB-first)
    uint16_t crc = 0xFFFF; // Giá trị khởi tạo

    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos]; // XOR byte dữ liệu

        for (uint8_t i = 8; i != 0; i--) { // Lặp 8 lần cho 8 bit
            if ((crc & 0x0001) != 0) { // Nếu bit LSB là 1
                crc >>= 1;            // Dịch phải
                crc ^= 0xA001;        // XOR với đa thức (polynomial)
            } else {                  // Nếu bit LSB là 0
                crc >>= 1;            // Chỉ dịch phải
            }
        }
    }

    return crc;
}

/**
 * @brief  Xác thực CRC của một gói tin Modbus RTU đã nhận.
 * @param  buffer: Con trỏ đến toàn bộ gói tin nhận được (bao gồm 2 byte CRC ở cuối).
 * @param  length: Tổng độ dài của gói tin (bao gồm 2 byte CRC).
 * @retval 1 nếu CRC hợp lệ, 0 nếu CRC sai.
 */
uint8_t Modbus_Validate_CRC(uint8_t *buffer, uint16_t length) {
    if (length < 2) {
        return 0; // Gói tin quá ngắn để chứa CRC
    }

    // 1. Lấy 2 byte CRC từ gói tin nhận được
    //    (Modbus RTU gửi byte Thấp (Low) trước, byte Cao (High) sau)
    uint16_t crc_received = buffer[length - 2] | (buffer[length - 1] << 8);

    // 2. Tính toán CRC cho phần dữ liệu
    //    (Tính trên toàn bộ gói tin TRỪ 2 byte CRC cuối)
    uint16_t crc_calculated = Modbus_CRC16(buffer, length - 2);

    // 3. So sánh
    return (crc_calculated == crc_received);
}
