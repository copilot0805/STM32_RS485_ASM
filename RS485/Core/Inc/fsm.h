/*
 * fsm.h
 *
 *  Created on: Nov 11, 2025
 *      Author: DELL
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_

#include "main.h"

// Định nghĩa 3 trạng thái của FSM
typedef enum {
    STATE_AUTO,
    STATE_SELECTED_ID,
    STATE_EXECUTE
} FSM_State_t;

/**
 * @brief  Khởi tạo FSM.
 * (Hàm sẽ khởi tạo LCD và hiển thị layout AUTO ban đầu).
 */
void FSM_Init(void);

/**
 * @brief  Hàm chạy chính của FSM, được gọi liên tục trong while(1).
 * (Hàm chứa logic timer cho chế độ AUTO và timeout).
 */
void FSM_Run(void);

/**
 * @brief  Hàm xử lý sự kiện nút nhấn (được gọi mỗi 10ms).
 * (Hàm kiểm tra button_count[] và thực thi logic FSM).
 */
void FSM_Button_Handle(void);

#endif /* INC_FSM_H_ */
