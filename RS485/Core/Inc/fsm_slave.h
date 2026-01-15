/*
 * fsm_slave.h
 *
 *  Created on: Jan 15, 2026
 *      Author: DELL
 */

#ifndef FSM_SLAVE_H_
#define FSM_SLAVE_H_

#include "main.h"

// Các trạng thái của Slave
typedef enum {
    SLAVE_STATE_IDLE,       // Chế độ chạy bình thường (Hiển thị ID & Nhiệt độ)
    SLAVE_STATE_SETUP,      // Chế độ cài đặt ID (Dùng nút tăng giảm)
    SLAVE_STATE_SAVING      // Trạng thái lưu vào Flash (Hiện thông báo "Saving...")
} Slave_State_t;

// Biến toàn cục chứa ID hiện tại (để main.c sử dụng khi check gói tin Modbus)
extern uint8_t my_slave_id;

// Các hàm public
void FSM_Slave_Init(void);
void FSM_Slave_Run(void);
void FSM_Slave_Button_Handle(void);

#endif /* FSM_SLAVE_H_ */
