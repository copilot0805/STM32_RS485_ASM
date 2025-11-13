/*
 * button.h
 *
 *  Created on: Nov 11, 2025
 *      Author: DELL
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"

/* Biến toàn cục để lưu trạng thái đếm (chống dội) */
extern uint16_t button_count[16];

/* Khai báo hàm */
void button_init(void);
void button_scan(void);

#endif /* INC_BUTTON_H_ */
