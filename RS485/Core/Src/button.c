/*
 * button.c
 *
 *  Created on: Nov 11, 2025
 *      Author: DELL
 */

/* Includes */
#include "button.h"

/* Variables */
// Khai báo handle SPI từ file main.c
extern SPI_HandleTypeDef hspi1;

// Định nghĩa biến đếm (chống dội)
uint16_t button_count[16] = {0};

// Biến nội bộ để lưu buffer SPI
static uint16_t button_spi_buffer = 0xFFFF; // Mặc định là 1 (không nhấn)

/* Functions */

/**
 * @brief  	Khởi tạo ma trận phím
 * @param  	None
 * @retval 	None
 */
void button_init() {
	// Đặt chân LOAD (PL) lên mức CAO (chế độ chờ)
	HAL_GPIO_WritePin(BTN_LOAD_GPIO_Port, BTN_LOAD_Pin, GPIO_PIN_SET);
}

/**
 * @brief  	Quét ma trận phím (đọc 16 nút)
 * @param  	None
 * @note  	Nên được gọi định kỳ
 * @retval 	None
 */
void button_scan() {
	// 1. Chốt (latch) trạng thái 16 nút
	// Kéo chân LOAD (PL) của 74HC165 xuống THẤP
	HAL_GPIO_WritePin(BTN_LOAD_GPIO_Port, BTN_LOAD_Pin, GPIO_PIN_RESET);
	// Kéo chân LOAD (PL) lên CAO trở lại
	HAL_GPIO_WritePin(BTN_LOAD_GPIO_Port, BTN_LOAD_Pin, GPIO_PIN_SET);

	// 2. Dùng SPI để đọc 2 byte (16 bit)
	//    &hspi1            : Handle của SPI1
	//    (void*) &button_spi_buffer : Con trỏ tới nơi lưu dữ liệu
	//    2                 : Đọc 2 bytes (16 bits)
	//    10                : Timeout 10ms
	HAL_SPI_Receive(&hspi1, (uint8_t*) &button_spi_buffer, 2, 10);

	// 3. Xử lý logic chống dội và sắp xếp lại (re-mapping)
	int button_index = 0;
	uint16_t mask = 0x8000; // Bắt đầu từ bit cao nhất (MSB)

	for (int i = 0; i < 16; i++) {
		//Logic Re-mapping
		if (i >= 0 && i <= 3) {
			button_index = i + 4;
		} else if (i >= 4 && i <= 7) {
			button_index = 7 - i;
		} else if (i >= 8 && i <= 11) {
			button_index = i + 4;
		} else {
			button_index = 23 - i;
		}
		//Kết thúc Logic Re-mapping ---

		// Kiểm tra bit: Nút nhấn là active-low (nhấn = 0)
		if (button_spi_buffer & mask) {
			// Bit = 1 (Không nhấn)
			button_count[button_index] = 0; // Reset bộ đếm
		} else {
			// Bit = 0 (Đang nhấn)
			if (button_count[button_index] < 65535) { // Chống tràn
				button_count[button_index]++; // Tăng bộ đếm
			}
		}

		mask = mask >> 1; // Dịch mask sang bit tiếp theo
	}
}
